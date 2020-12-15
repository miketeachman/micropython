/*
 * This file is part of the Micro Python project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2013, 2014 Damien P. George
 * Copyright (c) 2015 Bryan Morrissey
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

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include "py/nlr.h"
#include "py/runtime.h"
#include "py/gc.h"
#include "py/objstr.h"
#include "py/objlist.h"
#include "py/mphal.h"
#include "irq.h"
#include "pin.h"
#include "genhdr/pins.h"
#include "dma.h"
#include "bufhelper.h"
#include "modmachine.h"
#include "led.h" // For debugging using led_toggle(n)

// Pyboard V1.0/V1.1:   Two standard I2S interfaces (multiplexed with SPI2 and SPI3) are available
// Pyboard D SF2W, SF3W:  Three standard I2S interfaces (multiplexed with SPI1, SPI2 and SPI3) are available

// add "how it works" section:  TODO
// describe: 
// - queues, active buffer
// - circular mode for continuous streaming
// - asynchronous design
// - call back
// - use of STM32 HAL API
// - ?


// STM32 HAL API does not implement a MONO channel format TODO  finish describing this ...

// TODO implement TVE suggestions

// 1. "id" versus "port"  TODO:  update following for STM platform
//    The MicroPython API identifies instances of a peripheral using "id", while the ESP-IDF uses "port".
//    - for example, the first I2S peripheral on the ESP32 would be indicated by id=0 in MicroPython
//      and port=0 in ESP-IDF
// 2. any C type, macro, or function prefaced by "i2s" is associated with an ESP-IDF I2S interface definition
// 3. any C type, macro, or function prefaced by "machine_i2s" is associated with the MicroPython implementation of I2S

// \moduleref pyb
// \class I2S - Inter-IC-Sound, a protocol to transfer isochronous audio data
//
// I2S is a serial protocol for sending and receiving audio. This implementation
// uses three physical lines: Bit Clock, Word Select, and Data

// Possible DMA configurations for I2S busses:

// TODO add F7 processor for SP1
// SPI2 RX:     DMA1_Stream3.CHANNEL_0
// SPI2 TX:     DMA1_Stream4.CHANNEL_0
// SPI3 RX:     DMA1_Stream0.CHANNEL_0 or DMA1_Stream2.CHANNEL_0
// SPI3 TX:     DMA1_Stream5.CHANNEL_0 or DMA1_Stream7.CHANNEL_0

#define MEASURE_COPY_PERFORMANCE 1

#define SIZEOF_DMA_BUFFER_IN_BYTES (256)  // TODO what is the minimal size for acceptable performance
#define QUEUE_CAPACITY (10)

typedef enum {
    TOP_HALF    = 0,
    BOTTOM_HALF = 1,
} machine_i2s_dma_buffer_ping_pong_t;  // TODO crazy long name

typedef enum {
    MONO   = 0,
    STEREO = 1,
} machine_i2s_format_t;

typedef struct _machine_i2s_queue_t {
    mp_obj_t                buffers[QUEUE_CAPACITY];
    int16_t                 head;
    int16_t                 tail;
    int16_t                 size; // TODO is this needed?
} machine_i2s_queue_t;

typedef struct _machine_i2s_obj_t {
    mp_obj_base_t           base;
    mp_int_t                i2s_id;
    I2S_HandleTypeDef       i2s;
    const dma_descr_t       *tx_dma_descr;
    const dma_descr_t       *rx_dma_descr; 
    DMA_HandleTypeDef       tx_dma;
    DMA_HandleTypeDef       rx_dma;
    mp_obj_t                callback;
    mp_obj_t                active_buffer;
    uint32_t                active_buffer_index;
    machine_i2s_queue_t     active_queue;
    machine_i2s_queue_t     idle_queue;
    uint8_t                 dma_buffer[SIZEOF_DMA_BUFFER_IN_BYTES];
    pin_obj_t               *sck;
    pin_obj_t               *ws;
    pin_obj_t               *sd;
    uint16_t                mode;
    int8_t                  bits;
    machine_i2s_format_t    format;
    int32_t                 rate;
    bool                    used;
} machine_i2s_obj_t;

// TODO:  is this RAM usage unacceptably massive in uPy???  check sizeof() machine_i2s_obj_t versus the norms ....
// Static object mapping to I2S peripherals
//   note:  I2S implementation makes use of the following mapping between I2S peripheral and I2S object
//      I2S peripheral 1:  machine_i2s_obj[0]
//      I2S peripheral 2:  machine_i2s_obj[1]
STATIC machine_i2s_obj_t machine_i2s_obj[2] = {  //  TODO fix magic number 2  pybv1.1 = 1, pybD = 2
        [0].used = false,
        [1].used = false 
};

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
    queue->tail = (queue->tail + 1) % QUEUE_CAPACITY;
    queue->buffers[queue->tail] = item;
    queue->size = queue->size + 1;
}

// note:  must check queue is not empty before calling TODO ...thread safe?
//  TODO return NULL when queue is empty
STATIC mp_obj_t dequeue(machine_i2s_queue_t *queue) {
    mp_obj_t tmp = queue->buffers[queue->head];
    queue->head = (queue->head + 1) % QUEUE_CAPACITY;
    queue->size = queue->size - 1;
    return tmp;
}

STATIC bool isEmpty(machine_i2s_queue_t *queue) {
    return (bool)(queue->size == 0);
}


STATIC bool isFull(machine_i2s_queue_t *queue) {
    return (bool)(queue->size == QUEUE_CAPACITY);
}

//  For 32-bit audio samples, the STM32 HAL API expects each 32-bit sample to be encoded 
//  in an unusual byte ordering:  Byte_2, Byte_3, Byte_0, Byte_1
//      where:  Byte_0 is the least significant byte of the 32-bit sample
//
//  The following function takes a buffer containing 32-bits sample values formatted as little endian 
//  and performs an in-place modification into the STM32 HAL API convention
//
//  Example:
//
//   little_endian[] = [L_0-7,   L_8-15,  L_16-23, L_24-31, R_0-7,   R_8-15,  R_16-23, R_24-31] =  [Left channel, Right channel]           
//   stm_api[] =       [L_16-23, L_24-31, L_0-7,   L_8-15,  R_16-23, R_24-31, R_0-7,   R_8-15] = [Left channel, Right channel]
//
//   where:
//      L_0-7 is the least significant byte of the 32 bit sample in the Left channel 
//      L_24-31 is the most significant byte of the 32 bit sample in the Left channel 
//
//   little_endian[] =  [0x99, 0xBB, 0x11, 0x22, 0x44, 0x55, 0xAB, 0x77] = [Left channel, Right channel]           
//   stm_api[] =        [0x11, 0x22, 0x99, 0xBB, 0xAB, 0x77, 0x44, 0x55] = [Left channel, Right channel]
//
//   where:
//      LEFT Channel =  0x99, 0xBB, 0x11, 0x22
//      RIGHT Channel = 0x44, 0x55, 0xAB, 0x77
STATIC void machine_i2s_reformat_32_bit_samples(int32_t *sample, uint32_t num_samples) {
    int16_t sample_ms;
    int16_t sample_ls;
    for (uint32_t i=0; i<num_samples; i++) {
        sample_ls = sample[i] & 0xFFFF;
        sample_ms = sample[i] >> 16;
        sample[i] = (sample_ls << 16) + sample_ms;
    }
}

// TODO investigate mp_sched_schedule(self->callback, self);
STATIC void i2s_handle_mp_callback(machine_i2s_obj_t *self) {
    if (self->callback != mp_const_none) {
        gc_lock();
        nlr_buf_t nlr;
        if (nlr_push(&nlr) == 0) {
            mp_call_function_1(self->callback, self);
            nlr_pop();
        } else {
            // Uncaught exception; disable the callback so it doesn't run again.
            self->callback = mp_const_none;
            // DMA_HandleTypeDef dma = self->tx_dma;
            // __HAL_DMA_DISABLE(&dma);
            printf("uncaught exception in I2S(%u) DMA interrupt handler\n", self->i2s_id);
            mp_obj_print_exception(&mp_plat_print, (mp_obj_t)nlr.ret_val);
        }
        gc_unlock();
    }
}


// Simplying assumptions:
//   - size of sample buffers is an integer multiple of dma buffer size
//   -  TODO  note:  size of 1/2 dma buffer needs to be a multiple of 8 bytes  (so 1/2 of buffer always
//      contains integer number of complete stereo frames @ 32 bits/sample)
STATIC void machine_i2s_empty_dma(machine_i2s_obj_t *self, machine_i2s_dma_buffer_ping_pong_t dma_ping_pong) {
    
    mp_buffer_info_t bufinfo;
    
    // is a sample buffer actively being filled?
    // if not try to pull one from the idle queue
    
    if (self->active_buffer == NULL) {  // TODO rename idle_buffer ?  
        // try to pull new sample buffer from the queue
        if (!isEmpty(&self->idle_queue)) {
            mp_obj_t sample_buffer = dequeue(&self->idle_queue);
            self->active_buffer = sample_buffer;
            self->active_buffer_index = 0;
        } else {
            // no samples to 
            // TODO (?) fill dma buffer with zeros as there was no sample data to fill it
            // this would produce silence audio when no more sample data is available
            return;
        }
    }
    
    uint16_t dma_buffer_index = 0;
    if (dma_ping_pong == TOP_HALF) {
        dma_buffer_index = 0;  
    } else { // BOTTOM_HALF
        dma_buffer_index = SIZEOF_DMA_BUFFER_IN_BYTES/2; 
    }
    
    // 32 bit samples need to be reformatted to match STM32 HAL API requirements for I2S
    // TODO  SIZEOF_DMA_BUFFER_IN_BYTES/2  <--- make a macro for this
    if (self->bits == I2S_DATAFORMAT_32B) {
        machine_i2s_reformat_32_bit_samples((int32_t *)&self->dma_buffer[dma_buffer_index], SIZEOF_DMA_BUFFER_IN_BYTES/2/4);
    }
    
    // copy a block of samples from the dma buffer to the active buffer.
    // mono format is implemented by duplicating each sample into both L and R channels.
    // (STM32 HAL API has a stereo I2S implementation, but not mono)
    
    mp_get_buffer(self->active_buffer, &bufinfo, MP_BUFFER_WRITE);

    if ((self->format == MONO) && (self->bits == I2S_DATAFORMAT_16B)) {
        uint32_t samples_to_copy = SIZEOF_DMA_BUFFER_IN_BYTES/2/4;  // TODO - really confusing -- fix
        uint16_t *dma_buffer_p = (uint16_t *)&self->dma_buffer[dma_buffer_index];

        uint8_t *temp = (uint8_t *)bufinfo.buf;
        uint8_t *temp2 = &temp[self->active_buffer_index];
        uint16_t *active_buffer_p = (uint16_t *)temp2;

        for (uint32_t i=0; i<samples_to_copy; i++) {
            active_buffer_p[i] = dma_buffer_p[i*2]; 
        }
        self->active_buffer_index += SIZEOF_DMA_BUFFER_IN_BYTES/2/2;  // TODO - really confusing -- fix
        
    } else if ((self->format == MONO) && (self->bits == I2S_DATAFORMAT_32B)) {
        uint32_t samples_to_copy = SIZEOF_DMA_BUFFER_IN_BYTES/2/8;
        uint32_t *dma_buffer_p = (uint32_t *)&self->dma_buffer[dma_buffer_index];
        
        uint8_t *temp = (uint8_t *)bufinfo.buf;
        uint8_t *temp2 = &temp[self->active_buffer_index];
        uint32_t *active_buffer_p = (uint32_t *)temp2;
        
        for (uint32_t i=0; i<samples_to_copy; i++) {
            active_buffer_p[i] = dma_buffer_p[i*2]; 
        }
        self->active_buffer_index += SIZEOF_DMA_BUFFER_IN_BYTES/2/2;
        
    } else { // STEREO, both 16-bit and 32-bit
        memcpy(&((uint8_t *)bufinfo.buf)[self->active_buffer_index],
               &self->dma_buffer[dma_buffer_index], 
               SIZEOF_DMA_BUFFER_IN_BYTES/2);
        
        self->active_buffer_index += SIZEOF_DMA_BUFFER_IN_BYTES/2;
    }
    
    // has active buffer been filled?
    if (self->active_buffer_index >= bufinfo.len) {
        // clear buffer and push to active queue

        // TODO clear a buffer somewhere (before being put back into idle queue?)
        // memset(bufinfo.buf, 0, bufinfo.len);
        enqueue(&self->active_queue, self->active_buffer);
        self->active_buffer = NULL;
        i2s_handle_mp_callback(self);  // TODO "handle" <-- get better word
    }
}

// Simplying assumptions:
//   - size of sample buffers is an integer multiple of dma buffer size
//   -  TODO  note:  size of 1/2 dma buffer needs to be a multiple of 8 bytes  (so 1/2 of buffer always
//      contains integer number of complete stereo frames @ 32 bits/sample)
STATIC void machine_i2s_feed_dma(machine_i2s_obj_t *self, machine_i2s_dma_buffer_ping_pong_t dma_ping_pong) {
    
    mp_buffer_info_t bufinfo;
    
    // is a sample buffer actively being emptied?
    // if not try to pull one from the active queue
    
    if (self->active_buffer == NULL) {  // TODO rename active_buffer ?  
        // try to pull new sample buffer from the queue
        if (!isEmpty(&self->active_queue)) {  // TODO better name for active_queue?  sample_queue?
            mp_obj_t sample_buffer = dequeue(&self->active_queue);
            self->active_buffer = sample_buffer;
            self->active_buffer_index = 0;
        } else {
            // no samples to 
            // TODO (?) fill dma buffer with zeros as there was no sample data to fill it
            // this would produce silence audio when no more sample data is available
            return;
        }
    }
    
    uint16_t dma_buffer_index = 0;
    if (dma_ping_pong == TOP_HALF) {
        dma_buffer_index = 0;  
    } else { // BOTTOM_HALF
        dma_buffer_index = SIZEOF_DMA_BUFFER_IN_BYTES/2; 
    }
    
    mp_get_buffer(self->active_buffer, &bufinfo, MP_BUFFER_WRITE);
    
    // copy a block of samples from the active buffer to the dma buffer.
    // mono format is implemented by duplicating each sample into both L and R channels.
    // (STM32 HAL API has a stereo I2S implementation, but not mono)
    
    if ((self->format == MONO) && (self->bits == I2S_DATAFORMAT_16B)) {
        uint32_t samples_to_copy = SIZEOF_DMA_BUFFER_IN_BYTES/2/4; // TODO - really confusing -- fix
        uint16_t *dma_buffer_p = (uint16_t *)&self->dma_buffer[dma_buffer_index];

        uint8_t *temp = (uint8_t *)bufinfo.buf;
        uint8_t *temp2 = &temp[self->active_buffer_index];
        uint16_t *active_buffer_p = (uint16_t *)temp2;

        for (uint32_t i=0; i<samples_to_copy; i++) {
            dma_buffer_p[i*2] = active_buffer_p[i]; 
            dma_buffer_p[i*2+1] = active_buffer_p[i]; 
        }
        self->active_buffer_index += SIZEOF_DMA_BUFFER_IN_BYTES/2/2;  // TODO - really confusing -- fix
        
    } else if ((self->format == MONO) && (self->bits == I2S_DATAFORMAT_32B)) {
        uint32_t samples_to_copy = SIZEOF_DMA_BUFFER_IN_BYTES/2/8;
        uint32_t *dma_buffer_p = (uint32_t *)&self->dma_buffer[dma_buffer_index];
        
        uint8_t *temp = (uint8_t *)bufinfo.buf;
        uint8_t *temp2 = &temp[self->active_buffer_index];
        uint32_t *active_buffer_p = (uint32_t *)temp2;
        
        for (uint32_t i=0; i<samples_to_copy; i++) {
            dma_buffer_p[i*2] = active_buffer_p[i]; 
            dma_buffer_p[i*2+1] = active_buffer_p[i]; 
        }
        self->active_buffer_index += SIZEOF_DMA_BUFFER_IN_BYTES/2/2;
        
    } else { // STEREO, both 16-bit and 32-bit
        memcpy(&self->dma_buffer[dma_buffer_index], 
               &((uint8_t *)bufinfo.buf)[self->active_buffer_index], 
               SIZEOF_DMA_BUFFER_IN_BYTES/2);
        
        self->active_buffer_index += SIZEOF_DMA_BUFFER_IN_BYTES/2;
    }
    
    // 32 bit samples need to be reformatted to match STM32 HAL API requirements for I2S
    // TODO  SIZEOF_DMA_BUFFER_IN_BYTES/2  <--- make a macro for this
    if (self->bits == I2S_DATAFORMAT_32B) {
        machine_i2s_reformat_32_bit_samples((int32_t *)&self->dma_buffer[dma_buffer_index], SIZEOF_DMA_BUFFER_IN_BYTES/2/4);
    }
    
    // has active buffer been emptied?
    if (self->active_buffer_index >= bufinfo.len) {
        // clear buffer and push to idle queue
        memset(bufinfo.buf, 0, bufinfo.len);
        enqueue(&self->idle_queue, self->active_buffer);
        self->active_buffer = NULL;
        i2s_handle_mp_callback(self);  // TODO "handle" <-- get better word
    }
}

#if 0
static void led_flash_error(int colour, int count) {
    for(;;) {
        for (int c=0;c<count;c++) {
            led_state(colour, 1);
            HAL_Delay(300);
            led_state(colour, 0);
            HAL_Delay(300);
        }
        HAL_Delay(2000);
    }
}

static void led_flash_info(int colour, int count) {
    for (int c=0;c<count;c++) {
        led_state(colour, 1);
        HAL_Delay(300);
        led_state(colour, 0);
        HAL_Delay(300);
    }
}
#endif

// assumes init parameters are set up correctly
STATIC bool i2s_init(machine_i2s_obj_t *i2s_obj) {
    // init the GPIO lines
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
    GPIO_InitStructure.Pull = GPIO_PULLUP;

    if (i2s_obj->i2s_id == 1) {  // TODO replace magic number with Macro ... or use spi_find_index()?
        i2s_obj->i2s.Instance = I2S1;
        __SPI1_CLK_ENABLE();
        // configure DMA streams - see RM0090 section 10.3.3, Tables 42 & 43
        // TODO what to do when application tries to configure both I2S and SPI features on SPI1?

        if (i2s_obj->mode == I2S_MODE_MASTER_RX) {
            i2s_obj->rx_dma_descr = &dma_I2S_1_RX;
        } else {
            i2s_obj->tx_dma_descr = &dma_I2S_1_TX;
        }
    } else if (i2s_obj->i2s_id == 2) {
        i2s_obj->i2s.Instance = I2S2;
        __SPI2_CLK_ENABLE();
        // configure DMA streams - see RM0090 section 10.3.3, Tables 42 & 43
        if (i2s_obj->mode == I2S_MODE_MASTER_RX) {
            i2s_obj->rx_dma_descr = &dma_I2S_2_RX;
        } else {
            i2s_obj->tx_dma_descr = &dma_I2S_2_TX;
        }
    } else {
        // invalid i2s_id number; shouldn't get here as i2s object should not
        // have been created without setting a valid i2s instance number
        return false;
    }

    // GPIO Pin initialization
    if (i2s_obj->sck != MP_OBJ_NULL) {
        GPIO_InitStructure.Pin = i2s_obj->sck->pin_mask;
        const pin_af_obj_t *af = pin_find_af(i2s_obj->sck, AF_FN_I2S, i2s_obj->i2s_id);
        assert(af != NULL);  // TODO  assert ???  maybe uPY friendly way to indicate unexpected error?
        // Alt function is set using af->idx instead of GPIO_AFx_I2Sx macros
        GPIO_InitStructure.Alternate = (uint8_t)af->idx;
        HAL_GPIO_Init(i2s_obj->sck->gpio, &GPIO_InitStructure);
    }
    
    if (i2s_obj->ws != MP_OBJ_NULL) {
        GPIO_InitStructure.Pin = i2s_obj->ws->pin_mask;
        const pin_af_obj_t *af = pin_find_af(i2s_obj->ws, AF_FN_I2S, i2s_obj->i2s_id);
        assert(af != NULL);  // TODO  assert ???  maybe uPY friendly way to indicate unexpected error?
        // Alt function is set using af->idx instead of GPIO_AFx_I2Sx macros
        GPIO_InitStructure.Alternate = (uint8_t)af->idx;
        HAL_GPIO_Init(i2s_obj->ws->gpio, &GPIO_InitStructure);
    }
    
    if (i2s_obj->sd != MP_OBJ_NULL) {
        GPIO_InitStructure.Pin = i2s_obj->sd->pin_mask;
        const pin_af_obj_t *af = pin_find_af(i2s_obj->sd, AF_FN_I2S, i2s_obj->i2s_id);
        assert(af != NULL);  // TODO  assert ???  maybe uPY friendly way to indicate unexpected error?
        // Alt function is set using af->idx instead of GPIO_AFx_I2Sx macros
        GPIO_InitStructure.Alternate = (uint8_t)af->idx;
        HAL_GPIO_Init(i2s_obj->sd->gpio, &GPIO_InitStructure);
    }

    // Configure and enable I2SPLL - I2S_MASTER modes only:
    // ====================================================
    // References for STM32F405 (pybv10 and pybv11):
    //    1) table 127 "Audio frequency precision" of RM0090 Reference manual
    //    2) lines 457-494 of STM32Cube_FW_F4_V1.5.0/Drivers/BSP/STM32F4-Discovery/stm32f4_discovery_audio.c
    //
    // References for STM32F722 (PYBD-SF2W) and STM32F723 (PYBD-SF3W)
    //    1) table 204 "Audio-frequency precision" of RM0385 Reference manual
    //
    // References for STM32F767 (PYBD-SF6W)
    //    1) table 229 "Audio-frequency precision" of RM0410 Reference manual

    //    48kHz family is accurate for 8, 16, 24, and 48kHz but not 32 or 96
    //    44.1kHz family is accurate for 11.025, 22.05 and 44.1kHz but not 88.2

    // TODO: support more of the commonly-used frequencies and account for 16/32 bit frames
    // TODO: Refactor to use macros as provided by stm32f4xx_hal_rcc_ex.h
    
#if defined (STM32F405xx)
    // __HAL_RCC_PLLI2S_CONFIG(__PLLI2SN__, __PLLI2SR__)
    __HAL_RCC_PLLI2S_DISABLE();
    if ((i2s_obj->i2s.Init.AudioFreq & 0x7) == 0) {
        __HAL_RCC_PLLI2S_CONFIG(384, 5); // TODO
    } else {
        __HAL_RCC_PLLI2S_CONFIG(429, 4); // TODO
    }
        __HAL_RCC_PLLI2S_ENABLE();
#elif defined (STM32F722xx) || defined (STM32F723xx)
    // __HAL_RCC_PLLI2S_CONFIG(__PLLI2SN__, __PLLI2SQ__, __PLLI2SR__)
    // TODO can PLL config be set "per channel"?
    __HAL_RCC_PLLI2S_DISABLE();
    if ((i2s_obj->i2s.Init.AudioFreq & 0x7) == 0) {
        __HAL_RCC_PLLI2S_CONFIG(384, 1, 5); // TODO
    } else {
        __HAL_RCC_PLLI2S_CONFIG(429, 1, 4); // TODO
    }
        __HAL_RCC_PLLI2S_ENABLE();
#elif defined (STM32F767xx)
#error I2S not yet supported on the STM32F767xx processor (future)
#else
#error I2S does not support this processor
#endif // STM32F405xx

    if (HAL_I2S_Init(&i2s_obj->i2s) == HAL_OK) {
        // Reset and initialize Tx and Rx DMA channels

        if (i2s_obj->mode == I2S_MODE_MASTER_RX) {
            // Reset and initialize rx DMA
            dma_invalidate_channel(i2s_obj->rx_dma_descr);

            dma_init(&i2s_obj->rx_dma, i2s_obj->rx_dma_descr, DMA_PERIPH_TO_MEMORY, &i2s_obj->i2s);
            i2s_obj->i2s.hdmarx = &i2s_obj->rx_dma;
#if 0
            // set I2S Rx DMA interrupt priority
            if (i2s_obj->i2s_id == 1) {
                NVIC_SetPriority(DMA1_Stream3_IRQn, (IRQ_PRI_DMA+1));
                HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
            } else if (i2s_obj->i2s_id == 2) {
                //NVIC_SetPriority(DMA2_Stream3_IRQn, (IRQ_PRI_DMA+1));
                //NVIC_SetPriority(DMA1_Stream3_IRQn, (IRQ_PRI_DMA+1)); // lower DMA priority
                HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
            } else {
                // invalid i2s_id number; shouldn't get here as i2s object should not
                // have been created without setting a valid i2s instance number
                return false;
            }
#endif
        } else {
            // Reset and initialize tx DMA
            dma_invalidate_channel(i2s_obj->tx_dma_descr);

            dma_init(&i2s_obj->tx_dma, i2s_obj->tx_dma_descr, DMA_MEMORY_TO_PERIPH, &i2s_obj->i2s);
            i2s_obj->i2s.hdmatx = &i2s_obj->tx_dma;

#if 0
            // Workaround fix for streaming methods...
            // For streaming methods with a file based stream, I2S DMA IRQ priority must be lower than the SDIO DMA IRQ priority.
            // Explanation:
            //      i2s_stream_handler() is called in the I2S DMA IRQ handler context.
            //      SD Card reads (SDIO peripheral) are called in i2s_stream_handler() using DMA.
            //      When a block of SD card data is finished being read the
            //      SDIO DMA IRQ handler is never run because the I2S DMA IRQ handler is currently running
            //      and both IRQs have the same NVIC priority - as setup in dma_init().
            //      The end result is that i2s_stream_handler() gets "stuck" in an endless loop, in sdcard_wait_finished().
            //      This happens on the 2nd call to i2s_stream_handler().
            // The following two lines are a workaround to correct this problem.
            // When the priority of SDIO DMA IRQ is greater than
            // I2S DMA IRQ the SDIO DMA IRQ handler will interrupt the I2S DMA IRQ handler, allowing sdcard_wait_finished()
            // to complete.  Lower number implies higher interrupt priority
            // TODO figure out a more elegant way to solve this problem.
            // Investigate:  use the 1/2 complete DMA callback to execute stream read()

            // set I2S Tx DMA interrupt priority
            if (i2s_obj->i2s_id == 1) {
                NVIC_SetPriority(DMA2_Stream5_IRQn, (IRQ_PRI_DMA+1));
                HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);
            } else if (i2s_obj->i2s_id == 2) {
                NVIC_SetPriority(DMA1_Stream4_IRQn, (IRQ_PRI_DMA+1));
                HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
            } else {
                // invalid i2s_id number; shouldn't get here as i2s object should not
                // have been created without setting a valid i2s instance number
                return false;
            }
#endif
        }
        return true;
    } else {
        return false;
    }
}
    
#if 0
STATIC mp_obj_t machine_i2s_deinit(mp_obj_t self_in);
void i2s_deinit(void) {
    for (int i = 0; i < MP_ARRAY_SIZE(MP_STATE_PORT(pyb_i2s_obj_all)); i++) {
        machine_i2s_obj_t *i2s_obj = MP_STATE_PORT(pyb_i2s_obj_all)[i];
        if (i2s_obj != NULL) {
            machine_i2s_deinit(i2s_obj);
        }
    }
}
#endif

void HAL_I2S_ErrorCallback(I2S_HandleTypeDef *hi2s) {
    uint32_t errorCode = HAL_I2S_GetError(hi2s);
    printf("I2S Error = %ld\n", errorCode);
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s) {
    machine_i2s_obj_t *self;
    if (hi2s->Instance == I2S1) {
        self = &(machine_i2s_obj)[0];
    } else {
        self = &(machine_i2s_obj)[1];
    }
    
    // bottom half of buffer now filled, 
    // safe to empty the bottom half while the top half of buffer is being filled
    machine_i2s_empty_dma(self, BOTTOM_HALF);  // TODO check with =S= vs uPy coding rules.  is machine_i2s prefix really needed for STATIC?
}     
    
void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
    machine_i2s_obj_t *self;
    if (hi2s->Instance == I2S1) {
        self = &(machine_i2s_obj)[0];
    } else {
        self = &(machine_i2s_obj)[1];
    }
    
    
    // top half of buffer now filled, 
    // safe to empty the top  half while the bottom half of buffer is being filled
    machine_i2s_empty_dma(self, TOP_HALF);  // TODO check with =S= vs uPy coding rules.  is machine_i2s prefix really needed for STATIC?
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s) {
    machine_i2s_obj_t *self;
    if (hi2s->Instance == I2S1) {
        self = &(machine_i2s_obj)[0];
    } else {
        self = &(machine_i2s_obj)[1];
    }
    
    // bottom half of buffer now emptied, 
    // safe to fill the bottom half while the top half of buffer is being emptied
    machine_i2s_feed_dma(self, BOTTOM_HALF);  // TODO check with =S= vs uPy coding rules.  is machine_i2s prefix really needed for STATIC?
}

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
    machine_i2s_obj_t *self;
    if (hi2s->Instance == I2S1) {
        self = &(machine_i2s_obj)[0];
    } else {
        self = &(machine_i2s_obj)[1];
    }
    
    // top half of buffer now emptied, 
    // safe to fill the top  half while the bottom half of buffer is being emptied
    machine_i2s_feed_dma(self, TOP_HALF);  // TODO check with =S= vs uPy coding rules.  is machine_i2s prefix really needed for STATIC?
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
    
    // set the I2S configuration values
    memset(&self->i2s, 0, sizeof(self->i2s));
    
    queueInit(&self->active_queue);
    queueInit(&self->idle_queue);

    //
    // ---- Check validity of arguments ----
    //

    // are I2S pin assignments valid?
    const pin_af_obj_t *pin_af;

    // is SCK valid?
    if (mp_obj_is_type(args[ARG_sck].u_obj, &pin_type)) {
        pin_af = pin_find_af(args[ARG_sck].u_obj, AF_FN_I2S, self->i2s_id);
        if (pin_af->type != AF_PIN_TYPE_I2S_CK) {
            mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("no valid SCK pin for I2S%d"), self->i2s_id);
        }
    } else {
        mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("SCK not a Pin type"));
    }
    
    // is WS valid?
    if (mp_obj_is_type(args[ARG_ws].u_obj, &pin_type)) {
        pin_af = pin_find_af(args[ARG_ws].u_obj, AF_FN_I2S, self->i2s_id);
        if (pin_af->type != AF_PIN_TYPE_I2S_WS) {
            mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("no valid WS pin for I2S%d"), self->i2s_id);
        }
    } else {
        mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("WS not a Pin type"));
    }
    
    // is SD valid?
    if (mp_obj_is_type(args[ARG_sd].u_obj, &pin_type)) {
        pin_af = pin_find_af(args[ARG_sd].u_obj, AF_FN_I2S, self->i2s_id);
        if (pin_af->type != AF_PIN_TYPE_I2S_SD) {
            mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("no valid SD pin for I2S%d"), self->i2s_id);
        }
    } else {
        mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("SD not a Pin type"));
    }

    // is Mode valid?
    uint16_t i2s_mode = args[ARG_mode].u_int;
    if ((i2s_mode != (I2S_MODE_MASTER_RX)) &&
        (i2s_mode != (I2S_MODE_MASTER_TX))) {
        mp_raise_ValueError(MP_ERROR_TEXT("Mode is not valid"));
    }
    
    // is Bits valid?
    int8_t i2s_bits_per_sample = -1;
    // TODO add 24 bits -- audio folks will expect to see this
    if (args[ARG_bits].u_int == 16) { i2s_bits_per_sample = I2S_DATAFORMAT_16B; }
    else if (args[ARG_bits].u_int == 32) { i2s_bits_per_sample = I2S_DATAFORMAT_32B; }
    else { 
        mp_raise_ValueError(MP_ERROR_TEXT("Bits is not valid"));
    }
    
    // is Format valid?
    machine_i2s_format_t i2s_format = args[ARG_format].u_int;
    if ((i2s_format != MONO) &&
        (i2s_format != STEREO)) {
        mp_raise_ValueError(MP_ERROR_TEXT("Format is not valid"));
    }

    // is Sample Rate valid?
    // No validation done:  TODO can it be validated?
    
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
            enqueue(&self->idle_queue, elem[i]);
        }
    } else {
        mp_raise_TypeError(MP_ERROR_TEXT("Buffers must be contained in a list or tuple"));
    }

    // is callback valid?
    //printf("callable = %d\n", mp_obj_is_callable(args[ARG_callback].u_obj));  // TODO test with no callback
    // TODO raise exception if callback is bogus ?
    
    self->sck = args[ARG_sck].u_obj;
    self->ws = args[ARG_ws].u_obj;
    self->sd = args[ARG_sd].u_obj;
    self->mode = i2s_mode;
    self->bits = i2s_bits_per_sample;
    self->format = i2s_format;
    self->rate = args[ARG_rate].u_int;
    self->callback = args[ARG_callback].u_obj;
    
    I2S_InitTypeDef *init = &self->i2s.Init;
    init->Mode = i2s_mode;
    init->Standard   = I2S_STANDARD_PHILIPS;
    init->DataFormat = i2s_bits_per_sample;
    init->MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
    init->AudioFreq  = args[ARG_rate].u_int;
    init->CPOL       = I2S_CPOL_LOW;
    init->ClockSource = I2S_CLOCK_PLL; 
    
    // init the I2S bus
    if (!i2s_init(self)) {
        mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("I2S port %d init failed"), self->i2s_id);
    }
    
    self->used = true;
}

/******************************************************************************/
// MicroPython bindings for I2S
STATIC void machine_i2s_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    machine_i2s_obj_t *self = MP_OBJ_TO_PTR(self_in);
    
    qstr mode = 0;
    if (self->mode == I2S_MODE_MASTER_RX) { mode = MP_QSTR_RX; }
    else if (self->mode == I2S_MODE_MASTER_TX) { mode = MP_QSTR_TX; }
    else { /* shouldn't get here */ }
    
    uint8_t bits = 0;
    if (self->bits == I2S_DATAFORMAT_16B) { bits = 16; }
    else if (self->bits == I2S_DATAFORMAT_24B) { mode = 24; }
    else if (self->bits == I2S_DATAFORMAT_32B) { mode = 32; }
    else { /* shouldn't get here */ }
    
    // TODO add self->format
    
    mp_printf(print, "I2S(id=%u, sck=%q, ws=%q, sd=%q\n"
            "mode=%q, bits=%u, rate=%d)\n",
            self->i2s_id, self->sck->name, self->ws->name, self->sd->name,
            mode, bits, self->rate
            );
}

// TODO move all the pin stuff to the start of the file
// Construct an I2S object on the given bus.  `bus` can be 1 or 2.
// I2S requires a clock pin (SCK), a word select pin (WS) and
// a data pin (SD).
//
// Alternate Function (AF) Pin Mappings for I2S on pyboards
//
// Valid pins for I2S on the pyboard, models v1.0, v1.1:
// see alternate function mapping in datasheet
// TODO check pin availability
//     SCK -   B13 / Y6,  PB10 / Y9,       (SPI2 SCK)
//     WS -    B12 / Y5,  PB9  / Y4,       (SPI2 NSS)
//     SD -    B15 / Y8,  PC3  / X22       (SPI2 MOSI)
//   - `SPI(1)` is on the X position: `(NSS, SCK, MISO, MOSI) = (X5, X6, X7, X8) = (PA4, PA5, PA6, PA7)`
//   - `SPI(2)` is on the Y position: `(NSS, SCK, MISO, MOSI) = (Y5, Y6, Y7, Y8) = (PB12, PB13, PB14, PB15)`


//
// Valid pins for I2S on the pyboard D, models SF2W, SF3W:
// see alternate function  mapping in datasheet
// TODO check pin availability on DIP28 and DIP68 boards
//     SCK -   A5  / W6 / X6,  PB3  / --     (SPI1 SCK)
//     WS -    A4  / --,  PA15 / --     (SPI1 NSS)
//     SD -    A7  / --,  PB5  / --     (SPI1 MOSI)
//
//     SCK -   A9  / --,  PB13 / --,  PB10 / --,  PD3 / --  (SPI2 SCK)
//     WS -    PB12 / Y5,  PB9  / --                        (SPI2 NSS)
//     SD -    PB15 / --,  PC3  / --,  PC1 / --,  PC3 / --   (SPI2 MOSI)
//
//     SCK -   PB3  / --,  PC10 / --,   / --,   / --  (SPI3 SCK)
//     WS -    PA4 / ,  PA15  / --                        (SPI3 NSS)
//     SD -    PB5 / --, PC12   / --,   PD6/ --,   / --   (SPI3 MOSI)


//
// The I2S3 port is disabled by default on the pyboard, as its pins would
// conflict with the SD Card and other pyboard functions.
//
STATIC mp_obj_t machine_i2s_make_new(const mp_obj_type_t *type, size_t n_pos_args, size_t n_kw_args, const mp_obj_t *args) {
    mp_arg_check_num(n_pos_args, n_kw_args, 1, MP_OBJ_FUN_ARGS_MAX, true);

    machine_i2s_obj_t *self;
    
    // note: it is safe to assume that the arg pointer below references a positional argument because the arg check above
    //       guarantees that at least one positional argument has been provided
    uint8_t i2s_id = mp_obj_get_int(args[0]);  // TODO i2s_id has values 1,2,3 for STM32 ...different than ESP32 e.g.  0,1
    if (i2s_id == 1) { // TODO fix magic number 
        self = &machine_i2s_obj[0];
    } else if (i2s_id == 2) { // TODO fix magic number 
        self = &machine_i2s_obj[1];
    } else {
        mp_raise_ValueError(MP_ERROR_TEXT("I2S ID is not valid"));
    }

    self->base.type = &machine_i2s_type;
    self->i2s_id = i2s_id;

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
    if (self->mode == I2S_MODE_MASTER_TX) {
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
    if (self->mode == I2S_MODE_MASTER_TX) {
        // add buffer to queue 
        // TODO change way of doing this ... try to add with enqueue(), then test for true/false ...false = queue is full... can then 
        // eliminate isFull() routine
        if (isFull(&self->active_queue)) {
            mp_raise_ValueError(MP_ERROR_TEXT("Nogo - active queue is friggen full - end of the road bud"));
        }
        
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

STATIC mp_obj_t machine_i2s_start(mp_obj_t self_in) {  // TODO(?) self_in ---> self
    machine_i2s_obj_t *self = self_in;
    
    // TODO - add error checks ... for example, when this is called when no buffers exist in the queue
    // or:  has already been started and start() is called again
    // OR ... allow calling when no buffers exist and it's already running
    
    mp_buffer_info_t bufinfo;
    
    // TODO maybe do nothing is start() is called when it's already started?
    // - this could remove need to call isAvailable() in python code

#if 0
    if (isEmpty(&self->active_queue) && self->mode == I2S_MODE_MASTER_TX) {
        mp_raise_ValueError(MP_ERROR_TEXT("Nothing to transmit"));
        // TODO - might be OK to start and just send empty dma_buffer (TODO:  clear it first)
        // and then later have putbuffer fill it
    }
    
    if (isEmpty(&self->idle_queue) && self->mode == I2S_MODE_MASTER_RX) {
        mp_raise_ValueError(MP_ERROR_TEXT("No buffer for read"));
        // TODO - might be OK to start and just receive into empty dma_buffer
        // and then later have readinto empty it
    }
#endif
    
    // TODO seems to be a huge amount of coupling between queues and machine_i2s_feed_dma()
    if (self->mode == I2S_MODE_MASTER_TX) {
        mp_obj_t sample_buffer = dequeue(&self->active_queue);
        self->active_buffer = sample_buffer;
        mp_get_buffer(self->active_buffer, &bufinfo, MP_BUFFER_WRITE);
    } else {  // RX
        mp_obj_t sample_buffer = dequeue(&self->idle_queue);
        self->active_buffer = sample_buffer;
        mp_get_buffer(self->active_buffer, &bufinfo, MP_BUFFER_WRITE);
    }


    // start DMA.  DMA is configured to run continuously, using a circular buffer configuration
    uint16_t number_of_samples = 0;
    if (self->bits == I2S_DATAFORMAT_16B) {
        number_of_samples = SIZEOF_DMA_BUFFER_IN_BYTES / 2;
    } else {  // 32 bits
        number_of_samples = SIZEOF_DMA_BUFFER_IN_BYTES / 4;
    }
    HAL_StatusTypeDef status;

    if (self->mode == I2S_MODE_MASTER_TX) {
        machine_i2s_feed_dma(self, TOP_HALF);  // TODO is machine_i2s prefix really desirable for STATIC?
        machine_i2s_feed_dma(self, BOTTOM_HALF);
        status = HAL_I2S_Transmit_DMA(&self->i2s, (void *)self->dma_buffer, number_of_samples);
    } else {  // RX
        status = HAL_I2S_Receive_DMA(&self->i2s, (void *)self->dma_buffer, number_of_samples);
    }

    if (status != HAL_OK) {
        mp_hal_raise(status);
    }
    
    return mp_const_none;
    
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(machine_i2s_start_obj, machine_i2s_start);


STATIC mp_obj_t machine_i2s_deinit(mp_obj_t self_in) {
    machine_i2s_obj_t *self = self_in;
    if (self->used) {
        dma_deinit(self->tx_dma_descr);
        dma_deinit(self->rx_dma_descr);
        HAL_I2S_DeInit(&self->i2s);
        self->used = false;
    }
    
    if (self->i2s.Instance == I2S1) {
        __SPI1_FORCE_RESET();
        __SPI1_RELEASE_RESET();
        __SPI1_CLK_DISABLE();
    } else if (self->i2s.Instance == I2S2) {
        __SPI2_FORCE_RESET();
        __SPI2_RELEASE_RESET();
        __SPI2_CLK_DISABLE();
    }

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
    { MP_ROM_QSTR(MP_QSTR_deinit),          MP_ROM_PTR(&machine_i2s_deinit_obj) },
#if MEASURE_COPY_PERFORMANCE
    { MP_ROM_QSTR(MP_QSTR_copytest),       MP_ROM_PTR(&machine_i2s_copytest_obj) },
#endif     
    
    // TODO add separate callback() method to configure a callback?  research uPy conventions
    
    // TODO when "getbuffer" is implemented be sure to initialize the buf with zeros, to eliminate
    // residual samples that might come into play for a TX situation where we are reading
    // a SD Card file and there is a partial fill at the end of the file.

    // Constants
    { MP_ROM_QSTR(MP_QSTR_RX),              MP_ROM_INT(I2S_MODE_MASTER_RX) },
    { MP_ROM_QSTR(MP_QSTR_TX),              MP_ROM_INT(I2S_MODE_MASTER_TX) },
    { MP_ROM_QSTR(MP_QSTR_STEREO),          MP_ROM_INT(STEREO) },
    { MP_ROM_QSTR(MP_QSTR_MONO),            MP_ROM_INT(MONO) },
};
MP_DEFINE_CONST_DICT(machine_i2s_locals_dict, machine_i2s_locals_dict_table);

const mp_obj_type_t machine_i2s_type = {
    { &mp_type_type },
    .name = MP_QSTR_I2S,
    .print = machine_i2s_print,
    .make_new = machine_i2s_make_new,
    .locals_dict = (mp_obj_dict_t *) &machine_i2s_locals_dict,
};
