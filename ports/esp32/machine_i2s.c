/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2021 Mike Teachman
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

// TODO review need for all headers
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_task.h"
#include "py/obj.h"
#include "py/runtime.h"
#include "py/misc.h"
#include "py/gc.h"
#include "py/stream.h"
#include "modmachine.h"
#include "mphalport.h"
#include "driver/i2s.h"

#include "soc/i2s_reg.h"

#define MEASURE_COPY_PERFORMANCE 1

#define I2S_TASK_PRIORITY        (ESP_TASK_PRIO_MIN + 1)
#define I2S_TASK_STACK_SIZE      (2048)

#define SIZEOF_DMA_BUFFER_IN_FRAMES (256)
#define NUM_BYTES_IN_SAMPLE (4)

#define NUM_FORMATS (6)  // TODO confusion with self->format ?
#define I2S_RX_FRAME_SIZE_IN_BYTES (8)

STATIC const int8_t i2s_frame_overlay[NUM_FORMATS][I2S_RX_FRAME_SIZE_IN_BYTES] = {
    { 6,  7, -1, -1, -1, -1, -1, -1 },  // Mono, 16-bits
    { 5,  6,  7, -1, -1, -1, -1, -1 },  // Mono, 24-bits
    { 4,  5,  6,  7, -1, -1, -1, -1 },  // Mono, 32-bits
    { 6,  7,  2,  3, -1, -1, -1, -1 },  // Stereo, 16-bits
    { 5,  6,  7,  1,  2,  3 - 1, -1 },  // Stereo, 24-bits
    { 4,  5,  6,  7,  0,  1,  2,  3 },  // Stereo, 32-bits
};

typedef enum {
    MONO = 0,
    STEREO = 1,
} machine_i2s_format_t;

// === HOW IT WORKS ===  TODO
//
// Blocking
//
// Non-Blocking
//
// Uasyncio
//
// Streaming
//


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

typedef struct _machine_i2s_obj_t {
    mp_obj_base_t base;
    i2s_port_t i2s_id;
    int8_t sck;
    int8_t ws;
    int8_t sd;
    uint8_t mode;
    i2s_bits_per_sample_t bits;
    machine_i2s_format_t format;
    int32_t rate;
    int32_t buffer;
    mp_obj_t callback;
    bool used;
    uint8_t i2s_read_buffer[240];                 // TODO fix magic.  Note:  240 works for all combos of 16/24/32 bit, stereo/mono
    QueueHandle_t i2s_buffer_transfer_queue;
    QueueHandle_t i2s_event_queue;
    volatile TaskHandle_t client_task_handle;
    uint32_t dma_buf_len_in_bytes;
    bool asyncio_detected;
} machine_i2s_obj_t;

typedef enum {
    I2S_TX_TRANSFER,
    I2S_RX_TRANSFER,
} machine_i2s_transfer_type_t;

typedef struct _machine_i2s_buffer_transfer_t {  // TODO make consistent with Pyboard
    mp_buffer_info_t bufinfo;
    mp_obj_t callback;
    machine_i2s_transfer_type_t type;
} machine_i2s_buffer_transfer_t;


// Static object mapping to I2S peripherals  TODO change to root pointer?
//   note:  I2S implementation makes use of the following mapping between I2S peripheral and I2S object
//      I2S peripheral 1:  machine_i2s_obj[0]
//      I2S peripheral 2:  machine_i2s_obj[1]
STATIC machine_i2s_obj_t machine_i2s_obj[I2S_NUM_MAX] = {
    [0].used = false,
    [1].used = false
};

// this will be called on MicroPython startup and every soft reset
void machine_i2s_init(void) {
    for (uint8_t i = 0; i < I2S_NUM_MAX; i++) {
        machine_i2s_obj[i].used = false;
    }
}

// TODO for each instance of I2S?

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
//   wav_samples[] =  [0x99, 0xBB, 0x11, 0x22, 0x44, 0x55, 0xAB, 0x77] = [Left channel, Right channel]
//   i2s_samples[] =  [0x44, 0x55, 0xAB, 0x77, 0x99, 0xBB, 0x11, 0x22] = [Right channel,  Left channel]
//   notes:
//       samples in wav_samples[] arranged in little endian format:
//           0x77 is the most significant byte of the 32-bit sample
//           0x44 is the least significant byte of the 32-bit sample
//   where:
//      LEFT Channel =  0x99, 0xBB, 0x11, 0x22
//      RIGHT Channel = 0x44, 0x55, 0xAB, 0x77
STATIC void machine_i2s_swap_32_bit_stereo_channels(mp_buffer_info_t *bufinfo) {
    int32_t swap_sample;
    int32_t *sample = bufinfo->buf;
    uint32_t num_samples = bufinfo->len / 4;
    for (uint32_t i = 0; i < num_samples; i += 2) {
        swap_sample = sample[i + 1];
        sample[i + 1] = sample[i];
        sample[i] = swap_sample;
    }
}

STATIC int8_t get_overlay_index(i2s_bits_per_sample_t bits, machine_i2s_format_t format) {
    if (format == MONO) {
        if (bits == I2S_BITS_PER_SAMPLE_16BIT) {
            return 0;
        } else if (bits == I2S_BITS_PER_SAMPLE_24BIT) {
            return 1;
        } else { // 32 bits
            return 2;
        }
    } else { // STEREO
        if (bits == I2S_BITS_PER_SAMPLE_16BIT) {
            return 3;
        } else if (bits == I2S_BITS_PER_SAMPLE_24BIT) {
            return 4;
        } else { // 32 bits
            return 5;
        }
    }
}

STATIC i2s_bits_per_sample_t machine_i2s_get_esp_bits(uint8_t mode, i2s_bits_per_sample_t bits) {
    if (mode == (I2S_MODE_MASTER | I2S_MODE_TX)) {
        return bits;
    } else { // Master Rx
        // read 32 bit words for microphones
        return I2S_BITS_PER_SAMPLE_32BIT;
    }
}

STATIC i2s_channel_fmt_t machine_i2s_get_esp_format(uint8_t mode, machine_i2s_format_t format) {
    if (mode == (I2S_MODE_MASTER | I2S_MODE_TX)) {
        if (format == MONO) {
            return I2S_CHANNEL_FMT_ONLY_LEFT;
        } else {  // STEREO
            return I2S_CHANNEL_FMT_RIGHT_LEFT;
        }
    } else { // Master Rx
        // read a full STEREO frame for microphones
        return I2S_CHANNEL_FMT_RIGHT_LEFT;
    }
}

uint32_t the_one_readinto_function_to_rule_them_all(machine_i2s_obj_t *self, mp_buffer_info_t *bufinfo, bool asyncio) {

    // uint32_t t0 = mp_hal_ticks_us();

    // TODO raise exception if buffer is not a multiple of sample size

    // TODO add "if no callback ..."  e.g. blocking readinto
    uint8_t target_frame_size_in_bytes = self->bits / 8;

    if (self->format == STEREO) {
        // printf("STEREO\n");
        target_frame_size_in_bytes *= 2;
    }
    // printf("target_frame_size_in_bytes: %d\n", target_frame_size_in_bytes);

    uint32_t buf_index = 0;
    bool partial_read = false;
    while ((buf_index < bufinfo->len) && (partial_read == false)) {
        uint32_t num_bytes_read_from_dma = 0;
        // printf("buf_index: %d\n", buf_index);
        // TODO give "bufinfo" a better name
        uint32_t sample_spaces_remaining_in_buffer = (bufinfo->len - buf_index) / target_frame_size_in_bytes;
        uint32_t num_bytes_to_read_from_dma = MIN(sizeof(self->i2s_read_buffer),
            sample_spaces_remaining_in_buffer * I2S_RX_FRAME_SIZE_IN_BYTES);
        TickType_t delay;
        if (asyncio) {
            delay = 0;
        } else {
            delay = portMAX_DELAY;
        }

        esp_err_t ret = i2s_read(
            self->i2s_id,
            self->i2s_read_buffer,
            num_bytes_to_read_from_dma,     // TODO confusing -- > to_read versus read
            &num_bytes_read_from_dma,
            delay);

        if (num_bytes_read_from_dma < num_bytes_to_read_from_dma) {
            partial_read = true;
            // printf("$$$$$$$$$$$$$$$$$$$$$$$$  stalled out, num_bytes_read_from_dma: %d\n", num_bytes_read_from_dma);
        }

        if (num_bytes_read_from_dma % 8 != 0) {
            printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!! num bytes read NOT a multiple of 8\n");
        }

        // printf("-- %d samples left: %d\n", num_bytes_read_from_dma, sample_spaces_remaining_in_buffer);

        switch (ret) {
            case ESP_OK:
                break;
            case ESP_ERR_INVALID_ARG:
                mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("I2S read: Parameter error"));
                break;
            default:
                // this error not documented in ESP-IDF
                mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("I2S read: Undocumented error"));
                break;
        }

        uint32_t dma_index = 0;
        while (dma_index < num_bytes_read_from_dma) {
            uint8_t *target_p = bufinfo->buf + buf_index;
            uint8_t *dma_p = self->i2s_read_buffer + dma_index;

            uint8_t overlay_index = get_overlay_index(self->bits, self->format);
            for (uint8_t i = 0; i < I2S_RX_FRAME_SIZE_IN_BYTES; i++) {
                int8_t dma_mapping = i2s_frame_overlay[overlay_index][i];
                if (dma_mapping != -1) {
                    *target_p++ = dma_p[dma_mapping];
                    buf_index++;
                }
                dma_index++;
            }
        }
    }

    // uint32_t t1 = mp_hal_ticks_us();
    // printf("readinto [us]: %d\n", t1-t0);

    if (asyncio && (buf_index < bufinfo->len)) {
        // Unable to empty the entire buffer.  This indicates DMA RX buffers are empty
        // clear the I2S event queue to indicate that all DMA RX buffers are empty
        // TODO indicate that race condition is possible in this design, but that is acceptable
        // printf("++++++++++++++ DMA full, Reset I2S event queue\n");
        xQueueReset(self->i2s_event_queue);
    }

    return buf_index;
}

uint32_t the_one_write_function_to_rule_them_all(machine_i2s_obj_t *self, mp_buffer_info_t *bufinfo, bool asyncio) {
    if ((self->bits == I2S_BITS_PER_SAMPLE_32BIT) && (self->format == STEREO)) {
        machine_i2s_swap_32_bit_stereo_channels(bufinfo);
    }

    uint32_t num_bytes_written = 0;
    esp_err_t ret;

    TickType_t delay;
    if (asyncio) {
        delay = 0;
    } else {
        delay = portMAX_DELAY;
    }

    ret = i2s_write(self->i2s_id, bufinfo->buf, bufinfo->len, &num_bytes_written, delay);

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

    if (asyncio && (num_bytes_written < bufinfo->len)) {
        // Unable to empty the entire buffer.  This indicates DMA TX buffers are full
        // clear the I2S event queue to indicate that all DMA TX buffers are full
        // TODO race condition may exist, might be acceptable, investigate
        xQueueReset(self->i2s_event_queue);

        // undo the swap transformation as the buffer has not been completely emptied
        if ((self->bits == I2S_BITS_PER_SAMPLE_32BIT) && (self->format == STEREO)) {
            machine_i2s_swap_32_bit_stereo_channels(bufinfo);
        }
    }
    return num_bytes_written;
}

static void i2s_client_task(void *self_in) {
    machine_i2s_obj_t *self = (machine_i2s_obj_t *)self_in;

    machine_i2s_buffer_transfer_t i2s_buffer_transfer;
    uint32_t max_full_dma_bufs = 0;

    for (;;) {
        if (xQueueReceive(self->i2s_buffer_transfer_queue, &i2s_buffer_transfer, portMAX_DELAY)) {
            if (i2s_buffer_transfer.type == I2S_TX_TRANSFER) {
                // mp_buffer_info_t bufinfo;
                // mp_get_buffer_raise(i2s_buffer_transfer.buffer, &bufinfo, MP_BUFFER_WRITE);  // TODO  MP_BUFFER_READ ?  raise?
                the_one_write_function_to_rule_them_all(self, &i2s_buffer_transfer.bufinfo, false);
                mp_sched_schedule(i2s_buffer_transfer.callback, self);
            } else { // I2S_RX_TRANSFER
                uint32_t num_full_dma_bufs = uxQueueMessagesWaiting(self->i2s_event_queue);
                if (num_full_dma_bufs > max_full_dma_bufs) {
                    max_full_dma_bufs = num_full_dma_bufs;
                }
                // printf("# full bufs:  %d, max full bufs:  %d\n", num_full_dma_bufs, max_full_dma_bufs);

                // mp_buffer_info_t bufinfo;
                // mp_get_buffer_raise(i2s_buffer_transfer.bufinfo, &bufinfo, MP_BUFFER_WRITE);  // TODO  MP_BUFFER_READ ?  raise?
                the_one_readinto_function_to_rule_them_all(self, &i2s_buffer_transfer.bufinfo, false);
                mp_sched_schedule(i2s_buffer_transfer.callback, self);
            }
        }
    }
}

// TODO n_pos_args is not an implementation pattern for MicroPython
STATIC void machine_i2s_init_helper(machine_i2s_obj_t *self, size_t n_pos_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {

    enum {
        ARG_sck,
        ARG_ws,
        ARG_sd,
        ARG_mode,
        ARG_bits,
        ARG_format,
        ARG_rate,
        ARG_bufferlen,  // bytes
    };

    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_sck,              MP_ARG_KW_ONLY | MP_ARG_REQUIRED | MP_ARG_OBJ,   {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_ws,               MP_ARG_KW_ONLY | MP_ARG_REQUIRED | MP_ARG_OBJ,   {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_sd,               MP_ARG_KW_ONLY | MP_ARG_REQUIRED | MP_ARG_OBJ,   {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_mode,             MP_ARG_KW_ONLY | MP_ARG_REQUIRED | MP_ARG_INT,   {.u_int = -1} },
        { MP_QSTR_bits,             MP_ARG_KW_ONLY | MP_ARG_REQUIRED | MP_ARG_INT,   {.u_int = -1} },
        { MP_QSTR_format,           MP_ARG_KW_ONLY | MP_ARG_REQUIRED | MP_ARG_INT,   {.u_int = -1} },
        { MP_QSTR_rate,             MP_ARG_KW_ONLY | MP_ARG_REQUIRED | MP_ARG_INT,   {.u_int = -1} },
        { MP_QSTR_bufferlen,        MP_ARG_KW_ONLY | MP_ARG_REQUIRED | MP_ARG_INT,   {.u_int = -1} },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_pos_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // TODO - in STM32 port the configuration settings data structure was zero'd ... same for ESP32?

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
    machine_i2s_format_t i2s_format = args[ARG_format].u_int;
    if ((i2s_format != STEREO) &&
        (i2s_format != MONO)) {
        mp_raise_ValueError(MP_ERROR_TEXT("Format is not valid"));
    }

    // is Sample Rate valid?
    // No validation done:  ESP-IDF API does not indicate a valid range for sample rate

    // is Buffer size valid?
    // TODO validate

    self->sck = sck;
    self->ws = ws;
    self->sd = sd;
    self->mode = args[ARG_mode].u_int;
    self->bits = args[ARG_bits].u_int;
    self->format = args[ARG_format].u_int;
    self->rate = args[ARG_rate].u_int;
    self->buffer = args[ARG_bufferlen].u_int;
    self->callback = MP_OBJ_NULL;

    self->client_task_handle = NULL;
    self->asyncio_detected = false;

    // TODO convert following into function.  convert from bytes to ESP32 DMA len
    uint32_t dma_buf_len_in_bytes = SIZEOF_DMA_BUFFER_IN_FRAMES;
    dma_buf_len_in_bytes *= machine_i2s_get_esp_bits(self->mode, self->bits) / 8;

    if (machine_i2s_get_esp_format(self->mode, self->format) == I2S_CHANNEL_FMT_RIGHT_LEFT) {
        dma_buf_len_in_bytes *= 2;
    }
    self->dma_buf_len_in_bytes = dma_buf_len_in_bytes;

    // calculate how many DMA buffers are needed
    uint16_t dma_buf_count = self->buffer / dma_buf_len_in_bytes;

    // TODO or maybe throw exception?

    printf("dma_buf_count:  %d\n", dma_buf_count);
    printf("self->dma_buf_len_in_bytes: %d\n", self->dma_buf_len_in_bytes);
    printf("sizeof machine_i2s_obj_t:  %d\n", sizeof(machine_i2s_obj_t));

    i2s_config_t i2s_config;
    i2s_config.communication_format = I2S_COMM_FORMAT_I2S;
    i2s_config.mode = self->mode;
    i2s_config.bits_per_sample = machine_i2s_get_esp_bits(self->mode, self->bits);
    i2s_config.channel_format = machine_i2s_get_esp_format(self->mode, self->format);
    i2s_config.sample_rate = self->rate;
    i2s_config.intr_alloc_flags = ESP_INTR_FLAG_LOWMED; // TODO - understand this -- allows simultaneous use of both I2S channels
    i2s_config.dma_buf_count = dma_buf_count;  // 2 <= count <= 128
    i2s_config.dma_buf_len = SIZEOF_DMA_BUFFER_IN_FRAMES;  // len <= 1024
    i2s_config.use_apll = false;

    // uninstall I2S driver when changes are being made to an active I2S peripheral
    if (self->used) {
        i2s_driver_uninstall(self->i2s_id);
    }

    self->i2s_buffer_transfer_queue = xQueueCreate(1, sizeof(machine_i2s_buffer_transfer_t));
    // esp_err_t ret = i2s_driver_install(self->i2s_id, &i2s_config, 0, NULL);

    esp_err_t ret = i2s_driver_install(self->i2s_id, &i2s_config, dma_buf_count, &self->i2s_event_queue);
    switch (ret) {
        case ESP_OK:
            break;
        case ESP_ERR_INVALID_ARG:
            mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("I2S driver install: Parameter error"));
            break;
        case ESP_ERR_NO_MEM:
            mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("I2S driver install: Out of memory"));
            break;
        default:
            // this error not documented in ESP-IDF
            mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("I2S driver install: Undocumented error"));
            break;
    }

    // apply workaround for bug in some ESP-IDF versions that swap
    // the left and right channels
    // https://github.com/espressif/esp-idf/issues/6625
    REG_SET_BIT(I2S_CONF_REG(self->i2s_id), I2S_TX_MSB_RIGHT);
    REG_SET_BIT(I2S_CONF_REG(self->i2s_id), I2S_RX_MSB_RIGHT);

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

    ret = i2s_set_pin(self->i2s_id, &pin_config);
    switch (ret) {
        case ESP_OK:
            break;
        case ESP_ERR_INVALID_ARG:
            mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("I2S set pin: Parameter error"));
            break;
        case ESP_FAIL:
            mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("I2S set pin: IO error"));
            break;
        default:
            // this error not documented in ESP-IDF
            mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("I2S set pin: Undocumented error"));
            break;
    }
    self->used = true;
}

/******************************************************************************/
// MicroPython bindings for I2S  TODO update with all arguments
STATIC void machine_i2s_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    machine_i2s_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_printf(print, "I2S(id=%u, sck=%d, ws=%d, sd=%d\n"
        "mode=%u,\n"
        "bits=%u, format=%u,\n"
        "rate=%d)",
        self->i2s_id, self->sck, self->ws, self->sd,
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
    if (i2s_id == 0) {
        self = &machine_i2s_obj[0];
    } else if (i2s_id == 1) {
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

STATIC mp_obj_t machine_i2s_obj_init(mp_uint_t n_pos_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    // note:  "pos_args + 1" below has the effect of skipping over "self"
    machine_i2s_init_helper(pos_args[0], n_pos_args - 1, pos_args + 1, kw_args);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(machine_i2s_init_obj, 1, machine_i2s_obj_init);

STATIC mp_uint_t machine_i2s_stream_read(mp_obj_t self_in, void *buf_in, mp_uint_t size, int *errcode) {
    machine_i2s_obj_t *self = MP_OBJ_TO_PTR(self_in);

    // printf("start stream_read\n");

    // make sure we want at least 1 char
    if (size == 0) {
        return 0;
    }

    if (!self->used) {  // TODO needed?  (obj seems to be used if we got this far)
        *errcode = MP_EPERM;
    }

    if (self->mode != (I2S_MODE_MASTER | I2S_MODE_RX)) {
        *errcode = MP_EPERM;
        // TODO throw exception?
    }

    // TODO should likely put asyncio first and throw an exception if callback is set (or just ignore the callback)
    if (self->callback != MP_OBJ_NULL) {
        // printf("got a callback\n");
        machine_i2s_buffer_transfer_t buffer_transfer;
        buffer_transfer.bufinfo.buf = (void *)buf_in;
        buffer_transfer.bufinfo.len = size;
        buffer_transfer.callback = self->callback;
        buffer_transfer.type = I2S_RX_TRANSFER;
        xQueueSend(self->i2s_buffer_transfer_queue, &buffer_transfer, portMAX_DELAY);  // TODO handle error return case
        return buffer_transfer.bufinfo.len;
    } else {
        // printf("no callback\n");
        mp_buffer_info_t bufinfo;
        bufinfo.buf = (void *)buf_in;
        bufinfo.len = size;
        uint32_t num_bytes_read;
        if (self->asyncio_detected == true) {  // TODO refactor ... just the true/false are different
            num_bytes_read = the_one_readinto_function_to_rule_them_all(self, &bufinfo, true);
            // printf("async, num_bytes_read: %d\n", num_bytes_read);
        } else {
            num_bytes_read = the_one_readinto_function_to_rule_them_all(self, &bufinfo, false);
            // printf("non-async, num_bytes_read: %d\n", num_bytes_read);
        }
        return num_bytes_read;
    }
}

STATIC mp_uint_t machine_i2s_stream_write(mp_obj_t self_in, const void *buf_in, mp_uint_t size, int *errcode) {
    machine_i2s_obj_t *self = MP_OBJ_TO_PTR(self_in);

    // printf("start stream_write\n");

    if (!self->used) {  // TODO see comment on stream_read()
        *errcode = MP_EPERM;
    }

    if (self->mode != (I2S_MODE_MASTER | I2S_MODE_TX)) {
        *errcode = MP_EPERM;
    }

    // TODO should likely put asyncio first and throw an exception if callback is set (or just ignore the callback)
    if (self->callback != MP_OBJ_NULL) {
        // printf("got a callback\n");
        machine_i2s_buffer_transfer_t buffer_transfer;
        buffer_transfer.bufinfo.buf = (void *)buf_in;
        buffer_transfer.bufinfo.len = size;
        buffer_transfer.callback = self->callback;
        buffer_transfer.type = I2S_TX_TRANSFER;
        xQueueSend(self->i2s_buffer_transfer_queue, &buffer_transfer, portMAX_DELAY);  // TODO handle error return case
        return buffer_transfer.bufinfo.len;  // TODO return "size" - consistent with pyboard
    } else {
        // printf("no callback\n");
        mp_buffer_info_t bufinfo;
        bufinfo.buf = (void *)buf_in;
        bufinfo.len = size;
        uint32_t num_bytes_written;
        if (self->asyncio_detected == true) {  // TODO refactor ... just the true/false are different
            num_bytes_written = the_one_write_function_to_rule_them_all(self, &bufinfo, true);
            // printf("async, num_bytes_written: %d\n", num_bytes_written);
        } else {
            num_bytes_written = the_one_write_function_to_rule_them_all(self, &bufinfo, false);
            // printf("non-async, num_bytes_written: %d\n", num_bytes_written);
        }
        return num_bytes_written;
    }
}

STATIC mp_obj_t machine_i2s_deinit(mp_obj_t self_in) {
    machine_i2s_obj_t *self = self_in;
    i2s_driver_uninstall(self->i2s_id);
    self->asyncio_detected = false;
    self->used = false;

    // TODO kill any FreeRTOS Task for this I2S instance
    // TODO delete any FreeRTOS Queues
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(machine_i2s_deinit_obj, machine_i2s_deinit);

STATIC mp_obj_t machine_i2s_callback(mp_obj_t self_in, mp_obj_t handler) {
    machine_i2s_obj_t *self = self_in;
    if (handler != mp_const_none && !mp_obj_is_callable(handler)) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid handler"));
    }
    self->callback = handler;

    // TODO add ability to disable non-blocking operation and kill the running task
    // self->client_task_handle = NULL;
    // vTaskDelete(NULL);

    // non-block operation requires a task to process audio data in the background
    if (xTaskCreatePinnedToCore(i2s_client_task, "i2s", I2S_TASK_STACK_SIZE, self, I2S_TASK_PRIORITY, (TaskHandle_t *)&self->client_task_handle, MP_TASK_COREID) != pdPASS) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("failed to create I2S task"));
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(machine_i2s_callback_obj, machine_i2s_callback);


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
        for (uint32_t i = 0; i < bufsource.len; i++) {
            ((uint8_t *)bufdest.buf)[i] = ((uint8_t *)bufsource.buf)[i];
        }
        t1 = mp_hal_ticks_us();
    } else if (option == 3) {
        t0 = mp_hal_ticks_us();
        uint8_t *dest_ptr = (uint8_t *)bufdest.buf;
        uint8_t *source_ptr = (uint8_t *)bufsource.buf;
        for (uint32_t i = 0; i < bufsource.len; i++) {
            *dest_ptr++ = *source_ptr++;
        }
        t1 = mp_hal_ticks_us();
    } else {
        mp_raise_ValueError(MP_ERROR_TEXT("Invalid copy option"));
    }

    return mp_obj_new_int_from_uint(t1 - t0);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(machine_i2s_copytest_obj, 1, machine_i2s_copytest);
#endif

STATIC const mp_rom_map_elem_t machine_i2s_locals_dict_table[] = {
    // Methods
    { MP_ROM_QSTR(MP_QSTR_init),            MP_ROM_PTR(&machine_i2s_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_readinto),        MP_ROM_PTR(&mp_stream_readinto_obj) },
    { MP_ROM_QSTR(MP_QSTR_write),           MP_ROM_PTR(&mp_stream_write_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinit),          MP_ROM_PTR(&machine_i2s_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR_callback),        MP_ROM_PTR(&machine_i2s_callback_obj) },

    #if MEASURE_COPY_PERFORMANCE  // temporary - to be removed before mainline commit
    { MP_ROM_QSTR(MP_QSTR_copytest),        MP_ROM_PTR(&machine_i2s_copytest_obj) },
    #endif

    // Constants
    { MP_ROM_QSTR(MP_QSTR_RX),              MP_ROM_INT(I2S_MODE_MASTER | I2S_MODE_RX) },
    { MP_ROM_QSTR(MP_QSTR_TX),              MP_ROM_INT(I2S_MODE_MASTER | I2S_MODE_TX) },
    { MP_ROM_QSTR(MP_QSTR_STEREO),          MP_ROM_INT(STEREO) },
    { MP_ROM_QSTR(MP_QSTR_MONO),            MP_ROM_INT(MONO) },
};
MP_DEFINE_CONST_DICT(machine_i2s_locals_dict, machine_i2s_locals_dict_table);

STATIC mp_uint_t machine_i2s_ioctl(mp_obj_t self_in, mp_uint_t request, mp_uint_t arg, int *errcode) {
    machine_i2s_obj_t *self = self_in;
    mp_uint_t ret;
    mp_uint_t flags = arg;

    // printf("ioctl() poll\n");
    // set a flag indicating that I2S is being used in uasyncio mode
    // note:  assumption that an IO poll from the uasyncio scheduler is the only means
    // to get here
    // TODO -- above is a bad assumption - what if I2S is being used for methods in the stream protocol write, read, ioctl
    self->asyncio_detected = true;

    if (request == MP_STREAM_POLL) {
        ret = 0;

        if (flags & MP_STREAM_POLL_RD) {

            i2s_event_t i2s_event;

            // check event queue to determine if a DMA buffer has been filled
            // (which is an indication that at least one DMA buffer is available to be read)
            // note:  timeout = 0 so the call is non-blocking
            if (xQueueReceive(self->i2s_event_queue, &i2s_event, 0)) {
                if (i2s_event.type == I2S_EVENT_RX_DONE) {
                    // printf("ioctl RX EVENT\n");
                    // getting here means that at least one DMA buffer is now full
                    // indicate that data can be read from the stream
                    ret |= MP_STREAM_POLL_RD;
                }
            }
        }

        if (flags & MP_STREAM_POLL_WR) {

            i2s_event_t i2s_event;

            // check event queue to determine if a DMA buffer has been emptied
            // (which is an indication that at least one DMA buffer is available to be written)
            // note:  timeout = 0 so the call is non-blocking
            if (xQueueReceive(self->i2s_event_queue, &i2s_event, 0)) {
                if (i2s_event.type == I2S_EVENT_TX_DONE) {
                    // printf("ioctl TX EVENT\n");
                    // getting here means that at least one DMA buffer is now empty
                    // indicate that data can be written to the stream
                    ret |= MP_STREAM_POLL_WR;
                }
            }
        }
    } else {
        *errcode = MP_EINVAL;
        ret = MP_STREAM_ERROR;
    }

    return ret;
}

STATIC const mp_stream_p_t i2s_stream_p = {
    .read = machine_i2s_stream_read,
    .write = machine_i2s_stream_write,
    .ioctl = machine_i2s_ioctl,
    .is_text = false,
};

const mp_obj_type_t machine_i2s_type = {
    { &mp_type_type },
    .name = MP_QSTR_I2S,
    .print = machine_i2s_print,
    .getiter = mp_identity_getiter,
    .iternext = mp_stream_unbuffered_iter,
    .protocol = &i2s_stream_p,
    .make_new = machine_i2s_make_new,
    .locals_dict = (mp_obj_dict_t *)&machine_i2s_locals_dict,
};
