/*
 * This file is part of the MicroPython project, http://micropython.org/
 * 
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Mike Teachman
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_task.h"
#include "py/obj.h"
#include "py/runtime.h"
#include "modmachine.h"
#include "mphalport.h"
#include "driver/i2s.h"

#define MEASURE_COPY_PERFORMANCE 1

#define I2S_TASK_PRIORITY        (ESP_TASK_PRIO_MIN + 1)
#define I2S_TASK_STACK_SIZE      (2048)

#define SIZEOF_DMA_BUFFER_IN_BYTES (256)  // TODO what is the minimal size for acceptable performance
#define QUEUE_CAPACITY (10)

typedef enum {
    MONO   = 0,
    STEREO = 1,
} machine_i2s_format_t;

STATIC portMUX_TYPE           queue_spinlock = portMUX_INITIALIZER_UNLOCKED;


//  ESP32 buffer formats for write() and readinto() methods:

//  notation:
//      mono formats:
//          Mn_Bx_y
//              Mn=sample number
//              Bx_y= byte order 
//                
//              Example:  M0_B0_7:  first sample in buffer, least significant byte

//      stereo formats:
//          Ln_Bx_y
//              Ln=left channel sample number
//              Bx_y= byte order
//          similar for Right channel
//              Rn_Bx_y
//
//              Example:  R0_B24_31:  first right channel sample in buffer, most significant byte for 32 bit sample
//
//  samples are represented as little endian
//
// 16 bit mono
//   [M0_B0_7, M0_B8_15, M1_B0_7, M1_B8_15, ...] 
// 32 bit mono
//   [M0_B0_7, M0_B8_15, M0_B16_23, M0_B24_31, M1_B0_7, M1_B8_15, M1_B16_23, M1_B24_31, ...]
// 16 bit stereo
//   [L0_B0_7, L0_B8_15, R0_B0_7, R0_B8_15, L1_B0_7, L1_B8_15, R1_B0_7, R1_B8_15, ...]
// 32 bit stereo
//   [L0_B0_7, L0_B8_15, L0_B16_23, L0_B24_31, R0_B0_7, R0_B8_15, R0_B16_23, R0_B24_31, 
//    L1_B0_7, L1_B8_15, L1_B16_23, L1_B24_31, R1_B0_7, R1_B8_15, R1_B16_23, R1_B24_31, ...]


//  ESP32 buffer formats for read_into() method:



// TODO implement TVE suggestions

// Notes on naming conventions:
// 1. "id" versus "port"
//    The MicroPython API identifies instances of a peripheral using "id", while the ESP-IDF uses "port".
//    - for example, the first I2S peripheral on the ESP32 would be indicated by id=0 in MicroPython
//      and port=0 in ESP-IDF
// 2. any C type, macro, or function prefaced by "i2s" is associated with an ESP-IDF I2S interface definition
// 3. any C type, macro, or function prefaced by "machine_i2s" is associated with the MicroPython implementation of I2S

typedef struct _machine_i2s_queue_t {
    mp_obj_t                buffers[QUEUE_CAPACITY];
    int16_t                 head;
    int16_t                 tail;
    int16_t                 size; // TODO is this needed?
} machine_i2s_queue_t;

typedef struct _machine_i2s_obj_t {
    mp_obj_base_t          base;
    i2s_port_t             id;
    mp_obj_t               callback;
    mp_obj_t               active_buffer;
    uint32_t               active_buffer_index;
    machine_i2s_queue_t    active_queue;
    machine_i2s_queue_t    idle_queue;
    int8_t                 sck;
    int8_t                 ws;
    int8_t                 sd;
    uint8_t                mode;
    i2s_bits_per_sample_t  bits;
    i2s_channel_fmt_t      format;
    int32_t                rate;
    bool                   used;
    volatile TaskHandle_t  client_task_handle;
} machine_i2s_obj_t;

// Static object mapping to I2S peripherals  TODO change to root pointer? 
//   note:  I2S implementation makes use of the following mapping between I2S peripheral and I2S object
//      I2S peripheral 1:  machine_i2s_obj[0]
//      I2S peripheral 2:  machine_i2s_obj[1]
STATIC machine_i2s_obj_t machine_i2s_obj[I2S_NUM_MAX] = {
        [0].used = false,
        [1].used = false };

xQueueHandle i2s_event_queue; // TODO for each instance of I2S?

//  For 32-bit stereo, the ESP-IDF API has a channel convention of R, L channel ordering
//  The following function takes a buffer having L,R channel ordering and swaps channels   
//  to work with the ESP-IDF ordering R, L
//  TODO rewrite
//
//  Example:
//
//   wav_samples[] = [L_0-7, L_8-15, L_16-23, L_24-31, R_0-7, R_8-15, R_16-23, R_24-31] = [Left channel, Right channel]           
//   i2s_samples[] = [R_0-7, R_8-15, R_16-23, R_24-31, L_0-7, L_8-15, L_16-23, L_24-31] = [Right channel, Left channel]
//
//   where:
//     L_0-7 is the least significant byte of the 32 bit sample in the Left channel 
//     L_24-31 is the most significant byte of the 32 bit sample in the Left channel 
//
//   wav_samples[] =  [0x44, 0x55, 0xAB, 0x77, 0x99, 0xBB, 0x11, 0x22] = [Left channel, Right channel]           
//   i2s_samples[] =  [0x99, 0xBB, 0x11, 0x22, 0x44, 0x55, 0xAB, 0x77] = [Right channel,  Left channel]
//   notes:
//       samples in wav_samples[] arranged in little endian format:  
//           0x77 is the most significant byte of the 32-bit sample
//           0x44 is the least significant byte of the 32-bit sample
//              and
//           RIGHT Channel = 0x44, 0x55, 0xAB, 0x77
//           LEFT Channel =  0x99, 0xBB, 0x11, 0x22
STATIC void machine_i2s_swap_32_bit_stereo_channels(mp_buffer_info_t *bufinfo) {
    int32_t swap_sample;
    int32_t *sample = bufinfo->buf;
    uint32_t num_samples = bufinfo->len / 4;
    for (uint32_t i=0; i<num_samples; i+=2) {
        swap_sample = sample[i+1];
        sample[i+1] = sample[i];
        sample[i] = swap_sample;
    }
}





//
//  circular queue containing references to MicroPython objects (e.g. bytearray) that hold audio samples
//

STATIC void queueInit(machine_i2s_queue_t *queue) {
    queue->tail = -1;
    queue->head = 0;
    queue->size = 0;
}


// note:  must check queue has room before calling TODO ...thread safe?
// TODO see dequeue() comments
STATIC void enqueue(machine_i2s_queue_t *queue, mp_obj_t item) {
    // TODO semaphore or mutex might be a better soln
    portENTER_CRITICAL(&queue_spinlock);
    
    static uint16_t junk = 0;  // part of test for reentrantcy
    junk++;
    queue->tail = (queue->tail + 1) % QUEUE_CAPACITY;
    queue->buffers[queue->tail] = item;
    queue->size = queue->size + 1;
    junk--;

    portEXIT_CRITICAL(&queue_spinlock);
    assert(junk==0);
    assert(queue->size<6);  // TODO fix bug.  occasional assert on first time thru
}

// note:  must check queue is not empty before calling TODO ...thread safe?
//  TODO return NULL when queue is empty
STATIC mp_obj_t dequeue(machine_i2s_queue_t *queue) {
    portENTER_CRITICAL(&queue_spinlock);
    
    static uint16_t junk = 0;
    junk++;
    mp_obj_t tmp = queue->buffers[queue->head];
    queue->head = (queue->head + 1) % QUEUE_CAPACITY;
    queue->size = queue->size - 1;
    junk--;
    
    portEXIT_CRITICAL(&queue_spinlock);
    assert(junk==0);
    assert(queue->size<6);
    return tmp;
}

STATIC bool isEmpty(machine_i2s_queue_t *queue) {
    return (bool)(queue->size == 0);
}


STATIC bool isFull(machine_i2s_queue_t *queue) {
    return (bool)(queue->size == QUEUE_CAPACITY);
}

// Simplying assumptions:
//   - size of sample buffers is an integer multiple of dma buffer size
//   -  TODO  note:  size of 1/2 dma buffer needs to be a multiple of 8 bytes  (so 1/2 of buffer always
//      contains integer number of complete stereo frames @ 32 bits/sample)
STATIC void machine_i2s_feed_dma(machine_i2s_obj_t *self) {
    // loop until all DMA buffers allocated for I2S are full 
    bool dmaFull = false;
    
    while (dmaFull == false) {
        mp_buffer_info_t bufinfo;
        
        // is a sample buffer actively being emptied?
        // if not try to pull one from the active queue
        
        if (self->active_buffer == NULL) {  // TODO rename active_buffer ?  
            // try to pull new sample buffer from the queue
            if (!isEmpty(&self->active_queue)) {  // TODO better name for active_queue?  sample_queue?
                mp_obj_t sample_buffer = dequeue(&self->active_queue);
                self->active_buffer = sample_buffer;
                self->active_buffer_index = 0;
                
                mp_get_buffer(self->active_buffer, &bufinfo, MP_BUFFER_WRITE);
                if ((self->bits == I2S_BITS_PER_SAMPLE_32BIT) && (self->format == I2S_CHANNEL_FMT_RIGHT_LEFT)) {
                    machine_i2s_swap_32_bit_stereo_channels(&bufinfo);
                }
            } else {
                // active queue empty, no samples to transmit
                // TODO (?) fill dma buffer with zeros as there was no sample data to fill it
                // this would produce silence audio when no more sample data is available
                //printf("End feed_dma() Queue empty\n");
                //printf("QE\n");
                return;
            }
        }
        
        mp_get_buffer(self->active_buffer, &bufinfo, MP_BUFFER_WRITE);

        uint8_t *active_buffer_p = &((uint8_t *)bufinfo.buf)[self->active_buffer_index];
        uint32_t num_bytes_written = 0;

        esp_err_t ret = i2s_write(self->id, active_buffer_p, SIZEOF_DMA_BUFFER_IN_BYTES, &num_bytes_written, 0);
        
        switch (ret) {
            case ESP_OK:
                break;
            case ESP_ERR_INVALID_ARG:
                mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("I2S write: Parameter error"));
                break;
            default:
                // this error not documented in ESP-IDF
                mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("I2S write: Undocumented error")); 
                break;
        }
        
        self->active_buffer_index += num_bytes_written;
        
        // TODO:  test that SIZEOF_DMA_BUFFER_IN_BYTES were written, or zero.  this is by design.  e.g.  buffers must be 
        // sized at an integer multiple of SIZEOF_DMA_BUFFER_IN_BYTES.  So, either SIZEOF_DMA_BUFFER_IN_BYTES or 0 bytes
        // are written into DMA
        
        if (num_bytes_written == 0) {
            dmaFull = true;
        }
        
        // has active buffer been emptied?
        if (self->active_buffer_index >= bufinfo.len) {
            // clear buffer and push to idle queue
            memset(bufinfo.buf, 0, bufinfo.len);
            enqueue(&self->idle_queue, self->active_buffer);
            self->active_buffer = NULL;
    #if 0        
            i2s_handle_mp_callback(self);  // TODO "handle" <-- get better word
    #endif        
        }
    }
}

static void i2s_client_task(void *self_in) {
    machine_i2s_obj_t *self = (machine_i2s_obj_t *)self_in;
    i2s_event_t i2s_event;
    
    for(;;) {
        if(xQueueReceive(i2s_event_queue, &i2s_event, portMAX_DELAY)) {
            switch(i2s_event.type) {
                case I2S_EVENT_DMA_ERROR:
                    printf("I2S_EVENT_DMA_ERROR\n");
                    break;
                case I2S_EVENT_TX_DONE:
                    // getting here means that at least one DMA buffer is now free
                    machine_i2s_feed_dma(self);
                    break;
                case I2S_EVENT_RX_DONE:
                    printf("I2S_EVENT_RX_DONE\n");
                    break;
                default:
                    printf("BOGUS!\n");
                    break;
            }
        }
    }
    
    self->client_task_handle = NULL;
    vTaskDelete(NULL);
}

STATIC void machine_i2s_init_helper(machine_i2s_obj_t *self, size_t n_pos_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {

    enum {
        ARG_sck,
        ARG_ws,
        ARG_sd,
        ARG_mode,
        ARG_bits,
        ARG_format,
        ARG_rate,
        ARG_buffers,
        ARG_callback,
    };

    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_sck,              MP_ARG_KW_ONLY | MP_ARG_REQUIRED | MP_ARG_OBJ,   {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_ws,               MP_ARG_KW_ONLY | MP_ARG_REQUIRED | MP_ARG_OBJ,   {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_sd,               MP_ARG_KW_ONLY | MP_ARG_REQUIRED | MP_ARG_OBJ,   {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_mode,             MP_ARG_KW_ONLY | MP_ARG_REQUIRED | MP_ARG_INT,   {.u_int = -1} },
        { MP_QSTR_bits,             MP_ARG_KW_ONLY | MP_ARG_REQUIRED | MP_ARG_INT,   {.u_int = -1} },
        { MP_QSTR_format,           MP_ARG_KW_ONLY | MP_ARG_REQUIRED | MP_ARG_INT,   {.u_int = -1} },
        { MP_QSTR_rate,             MP_ARG_KW_ONLY | MP_ARG_REQUIRED | MP_ARG_INT,   {.u_int = -1} },
        { MP_QSTR_buffers,          MP_ARG_KW_ONLY | MP_ARG_REQUIRED | MP_ARG_OBJ,   {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_callback,         MP_ARG_KW_ONLY |                   MP_ARG_OBJ,   {.u_obj = MP_OBJ_NULL} },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_pos_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);
    
    // TODO - in STM32 port the configuration settings data structure was zero'd ... same for ESP32?
    
    queueInit(&self->active_queue);
    queueInit(&self->idle_queue);
    
    //
    // ---- Check validity of arguments ----
    //

    // are I2S pin assignments valid?
    int8_t sck = args[ARG_sck].u_obj == MP_OBJ_NULL ? -1 : machine_pin_get_id(args[ARG_sck].u_obj);
    int8_t ws = args[ARG_ws].u_obj == MP_OBJ_NULL ? -1 : machine_pin_get_id(args[ARG_ws].u_obj);
    int8_t sd = args[ARG_sd].u_obj == MP_OBJ_NULL ? -1 : machine_pin_get_id(args[ARG_sd].u_obj);

    // is Mode valid?
    i2s_mode_t i2s_mode = args[ARG_mode].u_int;
    if ((i2s_mode != (I2S_MODE_MASTER | I2S_MODE_RX)) &&
        (i2s_mode != (I2S_MODE_MASTER | I2S_MODE_TX))) {
        mp_raise_ValueError(MP_ERROR_TEXT("Modes is not valid"));
    }

    // is Bits valid?
    i2s_bits_per_sample_t i2s_bits_per_sample = args[ARG_bits].u_int;
    if ((i2s_bits_per_sample != I2S_BITS_PER_SAMPLE_16BIT) &&
        (i2s_bits_per_sample != I2S_BITS_PER_SAMPLE_32BIT)) {
        mp_raise_ValueError(MP_ERROR_TEXT("Bits is not valid"));
    }

    // is Format valid?
    i2s_channel_fmt_t i2s_format = args[ARG_format].u_int;
    if ((i2s_format != I2S_CHANNEL_FMT_RIGHT_LEFT) &&
        (i2s_format != I2S_CHANNEL_FMT_ONLY_LEFT)) {
        mp_raise_ValueError(MP_ERROR_TEXT("Format is not valid"));
    }

    // is Sample Rate valid?
    // No validation done:  ESP-IDF API does not indicate a valid range for sample rate

    // are Buffers valid?
    // buffers are contained in a list or tuple
    // check that type is a list or tuple
    if (mp_obj_is_type(args[ARG_buffers].u_obj, &mp_type_tuple) || mp_obj_is_type(args[ARG_buffers].u_obj, &mp_type_list)) {
        mp_uint_t len = 0;
        mp_obj_t *elem;
        mp_obj_get_array(args[ARG_buffers].u_obj, &len, &elem);
        
        if (len > QUEUE_CAPACITY) {
            mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("Num buffers exceeded, max is %d"), QUEUE_CAPACITY);
        }
        
        // add buffers to the idle queue
        for (uint16_t i=0; i<len; i++) {
            // check for valid buffer
            mp_buffer_info_t bufinfo;
            mp_get_buffer_raise(elem[i], &bufinfo, MP_BUFFER_RW);
            
            // add buffer to idle queue 
            //printf("init IQ: ");
            enqueue(&self->idle_queue, elem[i]);
        }
    } else {
        mp_raise_TypeError(MP_ERROR_TEXT("Buffers must be contained in a list or tuple"));
    }

    // is callback valid?
    //printf("callable = %d\n", mp_obj_is_callable(args[ARG_callback].u_obj));  // TODO test with no callback
    // TODO raise exception if callback is bogus ?
    
    self->sck = sck;
    self->ws = ws;
    self->sd = sd;
    self->mode = args[ARG_mode].u_int;
    self->bits = args[ARG_bits].u_int;
    self->format = args[ARG_format].u_int;
    self->rate = args[ARG_rate].u_int;
    self->callback = args[ARG_callback].u_obj;
    
    uint16_t dma_buf_len = SIZEOF_DMA_BUFFER_IN_BYTES;
    dma_buf_len /= self->bits/8;
    
    if (self->format == I2S_CHANNEL_FMT_RIGHT_LEFT) {
        dma_buf_len /= 2;
    }
    
    i2s_config_t i2s_config;
    i2s_config.communication_format = I2S_COMM_FORMAT_I2S;
    i2s_config.mode = self->mode;
    i2s_config.bits_per_sample = self->bits;
    i2s_config.channel_format = self->format;
    i2s_config.sample_rate = self->rate;
    i2s_config.intr_alloc_flags = ESP_INTR_FLAG_LOWMED; // allows simultaneous use of both I2S channels
    i2s_config.dma_buf_count = 10;
    i2s_config.dma_buf_len = dma_buf_len;
    i2s_config.use_apll = false;

    // uninstall I2S driver when changes are being made to an active I2S peripheral
    if (self->used) {
        i2s_driver_uninstall(self->id);
    }

    esp_err_t ret = i2s_driver_install(self->id, &i2s_config, 1, &i2s_event_queue);  // TODO queue should be per instance
    switch (ret) {
        case ESP_OK:
            break;
        case ESP_ERR_INVALID_ARG:
            mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("I2S driver install:  Parameter error"));
            break;
        case ESP_ERR_NO_MEM:
            mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("I2S driver install:  Out of memory"));
            break;
        default:
            // this error not documented in ESP-IDF
            mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("I2S driver install:  Undocumented error")); 
            break;
    }

    i2s_pin_config_t pin_config;
    pin_config.bck_io_num = self->sck;
    pin_config.ws_io_num = self->ws;
    
    if (i2s_mode == (I2S_MODE_MASTER | I2S_MODE_RX)) {
        pin_config.data_in_num = self->sd;
        pin_config.data_out_num = -1;
    } else {
        pin_config.data_in_num = -1;
        pin_config.data_out_num = self->sd;
    }

    ret = i2s_set_pin(self->id, &pin_config);
    switch (ret) {
        case ESP_OK:
            break;
        case ESP_ERR_INVALID_ARG:
            mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("I2S set pin:  Parameter error"));
            break;
        case ESP_FAIL:
            mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("I2S set pin:  IO error"));
            break;
        default:
            // this error not documented in ESP-IDF
            mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("I2S set pin:  Undocumented error")); 
            break;
    }

    self->used = true;
    
    if (xTaskCreatePinnedToCore(i2s_client_task, "i2s", I2S_TASK_STACK_SIZE, self, I2S_TASK_PRIORITY, (TaskHandle_t *)&self->client_task_handle, MP_TASK_COREID) != pdPASS) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("failed to create I2S task"));
    }

}

/******************************************************************************/
// MicroPython bindings for I2S
STATIC void machine_i2s_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    machine_i2s_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_printf(print, "I2S(id=%u, sck=%d, ws=%d, sd=%d\n"
            "mode=%u,\n"
            "bits=%u, format=%u,\n"
            "rate=%d)",
            self->id, self->sck, self->ws, self->sd,
            self->mode,
            self->bits, self->format,
            self->rate
            );
}

STATIC mp_obj_t machine_i2s_make_new(const mp_obj_type_t *type, size_t n_pos_args, size_t n_kw_args, const mp_obj_t *args) {
    mp_arg_check_num(n_pos_args, n_kw_args, 1, MP_OBJ_FUN_ARGS_MAX, true);

    machine_i2s_obj_t *self;
    
    // note: it is safe to assume that the arg pointer below references a positional argument because the arg check above
    //       guarantees that at least one positional argument has been provided
    i2s_port_t i2s_id = mp_obj_get_int(args[0]);
    if (i2s_id == I2S_NUM_0) {
        self = &machine_i2s_obj[0];
    } else if (i2s_id == I2S_NUM_1) {
        self = &machine_i2s_obj[1];
    } else {
        mp_raise_ValueError(MP_ERROR_TEXT("I2S ID is not valid"));
    }

    self->base.type = &machine_i2s_type;
    self->id = i2s_id;

    // is I2S peripheral already in use?
    if (self->used) {
        mp_raise_ValueError(MP_ERROR_TEXT("I2S port is already in use"));
    }

    mp_map_t kw_args;
    mp_map_init_fixed_table(&kw_args, n_kw_args, args + n_pos_args);
    // note:  "args + 1" below has the effect of skipping over the ID argument
    machine_i2s_init_helper(self, n_pos_args - 1, args + 1, &kw_args);

    return MP_OBJ_FROM_PTR(self);
}

STATIC mp_obj_t machine_i2s_init(mp_uint_t n_pos_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    // note:  "pos_args + 1" below has the effect of skipping over "self"
    machine_i2s_init_helper(pos_args[0], n_pos_args - 1, pos_args + 1, kw_args);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(machine_i2s_init_obj, 1, machine_i2s_init);

// TODO is returning None "pythonic" when no buffer is available?
STATIC mp_obj_t machine_i2s_getbuffer(mp_obj_t self_in) {
    machine_i2s_obj_t *self = self_in;

    if (!self->used) {
        mp_raise_ValueError(MP_ERROR_TEXT("I2S port is not initialized")); // TODO needed?
    }
    
    //if (self->mode != I2S_MODE_MASTER_TX) {
    //    mp_raise_ValueError(MP_ERROR_TEXT("I2S not configured for write method"));
    //}
    
    // For TX mode, remove from idle queue. For RX mode, add to active queue 
    if (self->mode == (I2S_MODE_MASTER | I2S_MODE_TX)) {
        // remove from idle queue 
        // TODO change way of doing this ... try to remove with dequeue(), then test for true/false ...false = queue is full... can then 
        // eliminate isEmpty() routine
        if (isEmpty(&self->idle_queue)) {
            return mp_const_none;
        } else {
            mp_obj_t buffer = dequeue(&self->idle_queue);
            return buffer;
        }
    } else { // RX
        // remove from active queue 
        // TODO change way of doing this ... try to remove with dequeue(), then test for true/false ...false = queue is full... can then 
        // eliminate isEmpty() routine
        if (isEmpty(&self->active_queue)) {
            return mp_const_none;
        } else {

            mp_obj_t buffer = dequeue(&self->active_queue);
            return buffer;  
        }
    }
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(machine_i2s_getbuffer_obj, machine_i2s_getbuffer);



STATIC mp_obj_t machine_i2s_putbuffer(mp_uint_t n_pos_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_buf };
    // TODO check that buffer size is multiple of 1/2 dma buffer size e.g  128 bytes
    
    // TODO check:  is buffer passed in one of the buffer supplied in the constructor?
    STATIC const mp_arg_t allowed_args[] = {
        { MP_QSTR_buf, MP_ARG_REQUIRED | MP_ARG_OBJ,  {.u_obj = mp_const_none} }, 
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_pos_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(args), allowed_args, args);

    machine_i2s_obj_t *self = pos_args[0];

    if (!self->used) {
        mp_raise_ValueError(MP_ERROR_TEXT("I2S port is not initialized"));  // needed?
    }
    
    //if (self->mode != I2S_MODE_MASTER_TX) {
    //    mp_raise_ValueError(MP_ERROR_TEXT("I2S not configured for write method"));
    //}
    
    mp_buffer_info_t bufinfo;
    mp_get_buffer(args[ARG_buf].u_obj, &bufinfo, MP_BUFFER_WRITE);
    
    // For TX mode, add to active queue. For RX mode, add to inactive queue 
    if (self->mode == (I2S_MODE_MASTER | I2S_MODE_TX)) {
        // add buffer to queue 
        // TODO change way of doing this ... try to add with enqueue(), then test for true/false ...false = queue is full... can then 
        // eliminate isFull() routine
        if (isFull(&self->active_queue)) {
            mp_raise_ValueError(MP_ERROR_TEXT("Nogo - active queue is friggen full - end of the road bud"));
        }
        
        //printf("PB() AQ: ");
        enqueue(&self->active_queue, args[ARG_buf].u_obj);
    } else { // RX
        // add buffer to queue 
        // TODO change way of doing this ... try to add with enqueue(), then test for true/false ...false = queue is full... can then 
        // eliminate isFull() routine
        if (isFull(&self->idle_queue)) {
            mp_raise_ValueError(MP_ERROR_TEXT("Nogo - inactive queue is friggen full - end of the road bud"));
        }
        
        // TODO likely memset the buffer to all 0s ?
        enqueue(&self->idle_queue, args[ARG_buf].u_obj);
    }

    return mp_const_none;  
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(machine_i2s_putbuffer_obj, 2, machine_i2s_putbuffer);



#if 0
STATIC mp_obj_t machine_i2s_readinto(mp_uint_t n_pos_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_buf };
    STATIC const mp_arg_t allowed_args[] = {
        { MP_QSTR_buf,                      MP_ARG_REQUIRED | MP_ARG_OBJ,  {.u_obj = mp_const_none} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_pos_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(args), allowed_args, args);

    machine_i2s_obj_t *self = pos_args[0];
    
    if (!self->used) {
        mp_raise_ValueError(MP_ERROR_TEXT("I2S port is not initialized"));
    }

    if (self->mode != (I2S_MODE_MASTER | I2S_MODE_RX)) {
        mp_raise_ValueError(MP_ERROR_TEXT("I2S not configured for read method"));
    }

    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(args[ARG_buf].u_obj, &bufinfo, MP_BUFFER_WRITE);

    uint32_t num_bytes_read = 0;
    
    esp_err_t ret = i2s_read(self->id, bufinfo.buf, bufinfo.len, &num_bytes_read, portMAX_DELAY);
    switch (ret) {
        case ESP_OK:
            break;
        case ESP_ERR_INVALID_ARG:
            mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("I2S read:  Parameter error"));
            break;
        default:
            // this error not documented in ESP-IDF
            mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("I2S read:  Undocumented error")); 
            break;
    }
    
    if ((self->bits == I2S_BITS_PER_SAMPLE_32BIT) && (self->format == I2S_CHANNEL_FMT_RIGHT_LEFT)) {
        machine_i2s_swap_32_bit_stereo_channels(&bufinfo);
    }

    return mp_obj_new_int(num_bytes_read);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(machine_i2s_readinto_obj, 2, machine_i2s_readinto);
#endif

#if 0
STATIC mp_obj_t machine_i2s_write(mp_uint_t n_pos_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_buf };
    STATIC const mp_arg_t allowed_args[] = {
        { MP_QSTR_buf,                      MP_ARG_REQUIRED | MP_ARG_OBJ,  {.u_obj = mp_const_none} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_pos_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(args), allowed_args, args);

    machine_i2s_obj_t *self = pos_args[0];

    if (!self->used) {
        mp_raise_ValueError(MP_ERROR_TEXT("I2S port is not initialized"));
    }

    if (self->mode != (I2S_MODE_MASTER | I2S_MODE_TX)) {
        mp_raise_ValueError(MP_ERROR_TEXT("I2S not configured for write method"));
    }
    
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(args[ARG_buf].u_obj, &bufinfo, MP_BUFFER_WRITE);  // TODO  MP_BUFFER_READ ?
    
    if ((self->bits == I2S_BITS_PER_SAMPLE_32BIT) && (self->format == I2S_CHANNEL_FMT_RIGHT_LEFT)) {
        machine_i2s_swap_32_bit_stereo_channels(&bufinfo);
    }

    uint32_t num_bytes_written = 0;
    esp_err_t ret = i2s_write(self->id, bufinfo.buf, bufinfo.len, &num_bytes_written, portMAX_DELAY);
    switch (ret) {
        case ESP_OK:
            break;
        case ESP_ERR_INVALID_ARG:
            mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("I2S write: Parameter error"));
            break;
        default:
            // this error not documented in ESP-IDF
            mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("I2S write: Undocumented error")); 
            break;
    }
    
    return mp_obj_new_int(num_bytes_written);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(machine_i2s_write_obj, 2, machine_i2s_write);
#endif

STATIC mp_obj_t machine_i2s_start(mp_obj_t self_in) {  // TODO(?) self_in ---> self
    
    // TODO - add error checks ... for example, when this is called when no buffers exist in the queue
    // or:  has already been started and start() is called again
    // OR ... allow calling when no buffers exist and it's already running
    
    // TODO maybe do nothing is start() is called when it's already started?
    // - this could remove need to call isAvailable() in python code

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(machine_i2s_start_obj, machine_i2s_start);


STATIC mp_obj_t machine_i2s_deinit(mp_obj_t self_in) {
    machine_i2s_obj_t *self = self_in;
    i2s_driver_uninstall(self->id);
    self->used = false;
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(machine_i2s_deinit_obj, machine_i2s_deinit);

#if MEASURE_COPY_PERFORMANCE
STATIC mp_obj_t machine_i2s_copytest(mp_uint_t n_pos_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_bufsource, ARG_bufdest, ARG_option };
    STATIC const mp_arg_t allowed_args[] = {
        { MP_QSTR_bufsource, MP_ARG_KW_ONLY | MP_ARG_REQUIRED | MP_ARG_OBJ,  {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_bufdest,   MP_ARG_KW_ONLY | MP_ARG_REQUIRED | MP_ARG_OBJ,  {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_option,    MP_ARG_KW_ONLY | MP_ARG_REQUIRED | MP_ARG_INT,  {.u_int = 1} },
    };
    
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_pos_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(args), allowed_args, args);
    
    mp_buffer_info_t bufsource;
    mp_get_buffer_raise(args[ARG_bufsource].u_obj, &bufsource, MP_BUFFER_READ);
    
    mp_buffer_info_t bufdest;
    mp_get_buffer_raise(args[ARG_bufdest].u_obj, &bufdest, MP_BUFFER_WRITE);
    
    uint16_t option = args[ARG_option].u_int;
    uint32_t t0 = 0;
    uint32_t t1 = 0;
        
    if (option == 1) {
        t0 = mp_hal_ticks_us();
        memcpy(bufdest.buf,
               bufsource.buf, 
               bufsource.len);
        t1 = mp_hal_ticks_us();
    } else if (option == 2) {
        t0 = mp_hal_ticks_us();
        for (uint32_t i=0; i<bufsource.len; i++) {
            ((uint8_t *)bufdest.buf)[i] = ((uint8_t *)bufsource.buf)[i]; 
        }
        t1 = mp_hal_ticks_us();
    } else if (option == 3) {
        t0 = mp_hal_ticks_us();
        uint8_t *dest_ptr = (uint8_t *)bufdest.buf;
        uint8_t *source_ptr = (uint8_t *)bufsource.buf;
        for (uint32_t i=0; i<bufsource.len; i++) {
            *dest_ptr++ = *source_ptr++; 
        }
        t1 = mp_hal_ticks_us();
    } else {
        mp_raise_ValueError(MP_ERROR_TEXT("Invalid copy option"));
    }
    
    return mp_obj_new_int_from_uint(t1-t0);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(machine_i2s_copytest_obj, 1, machine_i2s_copytest);
#endif

STATIC const mp_rom_map_elem_t machine_i2s_locals_dict_table[] = {
    // Methods
    { MP_ROM_QSTR(MP_QSTR_init),            MP_ROM_PTR(&machine_i2s_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_getbuffer),       MP_ROM_PTR(&machine_i2s_getbuffer_obj) },
    { MP_ROM_QSTR(MP_QSTR_putbuffer),       MP_ROM_PTR(&machine_i2s_putbuffer_obj) },
    { MP_ROM_QSTR(MP_QSTR_start),           MP_ROM_PTR(&machine_i2s_start_obj) },
//    { MP_ROM_QSTR(MP_QSTR_readinto),        MP_ROM_PTR(&machine_i2s_readinto_obj) },
//    { MP_ROM_QSTR(MP_QSTR_write),           MP_ROM_PTR(&machine_i2s_write_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinit),          MP_ROM_PTR(&machine_i2s_deinit_obj) },
#if MEASURE_COPY_PERFORMANCE
    { MP_ROM_QSTR(MP_QSTR_copytest),       MP_ROM_PTR(&machine_i2s_copytest_obj) },
#endif     

    // Constants
    { MP_ROM_QSTR(MP_QSTR_NUM0),            MP_ROM_INT(I2S_NUM_0) },
    { MP_ROM_QSTR(MP_QSTR_NUM1),            MP_ROM_INT(I2S_NUM_1) },
    { MP_ROM_QSTR(MP_QSTR_RX),              MP_ROM_INT(I2S_MODE_MASTER | I2S_MODE_RX) },
    { MP_ROM_QSTR(MP_QSTR_TX),              MP_ROM_INT(I2S_MODE_MASTER | I2S_MODE_TX) },
    { MP_ROM_QSTR(MP_QSTR_STEREO),          MP_ROM_INT(I2S_CHANNEL_FMT_RIGHT_LEFT) },
    { MP_ROM_QSTR(MP_QSTR_MONO),            MP_ROM_INT(I2S_CHANNEL_FMT_ONLY_LEFT) },
};
MP_DEFINE_CONST_DICT(machine_i2s_locals_dict, machine_i2s_locals_dict_table);

const mp_obj_type_t machine_i2s_type = {
    { &mp_type_type },
    .name = MP_QSTR_I2S,
    .print = machine_i2s_print,
    .make_new = machine_i2s_make_new,
    .locals_dict = (mp_obj_dict_t *) &machine_i2s_locals_dict,
};