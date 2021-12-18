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

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#include "py/obj.h"
#include "py/runtime.h"
#include "py/mphal.h"
#include "py/misc.h"
#include "py/stream.h"
#include "py/objstr.h"
#include "modmachine.h"
#include "dma_channel.h"

#include "clock_config.h"
#include "fsl_dmamux.h"
#include "fsl_iomuxc.h"
#include "fsl_sai_edma.h"

#if MICROPY_PY_MACHINE_I2S
// The I2S module has 3 modes of operation:
//
// Mode1:  Blocking
// - readinto() and write() methods block until the supplied buffer is filled (read) or emptied (write)
// - this is the default mode of operation
//
// Mode2:  Non-Blocking
// - readinto() and write() methods return immediately
// - buffer filling and emptying happens asynchronously to the main MicroPython task
// - a callback function is called when the supplied buffer has been filled (read) or emptied (write)
// - non-blocking mode is enabled when a callback is set with the irq() method
// - the DMA callbacks (1/2 complete and complete) are used to implement the asynchronous background operations
//
// Mode3: Uasyncio
// - implements the stream protocol
// - uasyncio mode is enabled when the ioctl() function is called
// - the state of the internal ring buffer is used to detect that I2S samples can be read or written
//
// The samples contained in the app buffer supplied for the readinto() and write() methods have the following convention:
//   Mono:  little endian format
//   Stereo:  little endian format, left channel first
//
// I2S terms:
//   "frame":  consists of two audio samples (Left audio sample + Right audio sample)
//
// Misc:
// - for Mono configuration:
//   - readinto method: samples are gathered from the L channel only
//   - write method: every sample is output to both the L and R channels
// - for readinto method the I2S hardware is read using 8-byte frames
//   (this is standard for almost all I2S hardware, such as MEMS microphones)
// - all 3 Modes of operation are implemented using the HAL I2S Generic Driver
// - all sample data transfers use DMA
// - the DMA controller is configured in Circular mode to fulfil continuous and gapless sample flows
// - the DMA ping-pong buffer needs to be aligned to a cache line size of 32 bytes.  32 byte
//   alignment is needed to use the routines that clean and invalidate D-Cache which work on a
//   32 byte address boundary.  Not all STM32 devices have a D-Cache.  Buffer alignment
//   will still happen on these devices to keep this code simple.

// DMA ping-pong buffer size was empirically determined.  It is a tradeoff between:
// 1. memory use (smaller buffer size desirable to reduce memory footprint)
// 2. interrupt frequency (larger buffer size desirable to reduce interrupt frequency)
// The sizeof 1/2 of the DMA buffer must be evenly divisible by the cache line size of 32 bytes.
#define SIZEOF_DMA_BUFFER_IN_BYTES (256)
#define SIZEOF_HALF_DMA_BUFFER_IN_BYTES (SIZEOF_DMA_BUFFER_IN_BYTES / 2)

// For non-blocking mode, to avoid underflow/overflow, sample data is written/read to/from the ring buffer at a rate faster
// than the DMA transfer rate
#define NON_BLOCKING_RATE_MULTIPLIER (4)
#define SIZEOF_NON_BLOCKING_COPY_IN_BYTES (SIZEOF_HALF_DMA_BUFFER_IN_BYTES * NON_BLOCKING_RATE_MULTIPLIER)

#define NUM_I2S_USER_FORMATS (4)
#define I2S_RX_FRAME_SIZE_IN_BYTES (8)

typedef enum {
    RX,
    TX
} i2s_mode_t;

typedef enum {
    MONO,
    STEREO
} format_t;

typedef enum {
    BLOCKING,
    NON_BLOCKING,
    UASYNCIO
} io_mode_t;

typedef enum {
    TOP_HALF,
    BOTTOM_HALF
} ping_pong_t;

typedef struct _ring_buf_t {
    uint8_t *buffer;
    size_t head;
    size_t tail;
    size_t size;
} ring_buf_t;

typedef struct _non_blocking_descriptor_t {
    mp_buffer_info_t appbuf;
    uint32_t index;
    bool copy_in_progress;
} non_blocking_descriptor_t;

typedef struct _machine_i2s_obj_t {
    mp_obj_base_t base;
    uint8_t i2s_id;
    mp_hal_pin_obj_t sck;
    mp_hal_pin_obj_t ws;
    mp_hal_pin_obj_t sd;
    uint16_t mode;
    int8_t bits;
    format_t format;
    int32_t rate;
    int32_t ibuf;
    mp_obj_t callback_for_non_blocking;
    uint8_t dma_buffer[SIZEOF_DMA_BUFFER_IN_BYTES + 0x1f]; // 0x1f related to D-Cache alignment
    uint8_t *dma_buffer_dcache_aligned;
    ring_buf_t ring_buffer;
    uint8_t *ring_buffer_storage;
    non_blocking_descriptor_t non_blocking_descriptor;
    io_mode_t io_mode;
#if 0    
    I2S_HandleTypeDef hi2s;
    DMA_HandleTypeDef hdma_tx;
    DMA_HandleTypeDef hdma_rx;
    const dma_descr_t *dma_descr_tx;
    const dma_descr_t *dma_descr_rx;
#endif    
} machine_i2s_obj_t;

STATIC mp_obj_t machine_i2s_deinit(mp_obj_t self_in);

// The frame map is used with the readinto() method to transform the audio sample data coming
// from DMA memory (32-bit stereo) to the format specified
// in the I2S constructor.  e.g.  16-bit mono
STATIC const int8_t i2s_frame_map[NUM_I2S_USER_FORMATS][I2S_RX_FRAME_SIZE_IN_BYTES] = {
    { 0,  1, -1, -1, -1, -1, -1, -1 },  // Mono, 16-bits
    { 2,  3,  0,  1, -1, -1, -1, -1 },  // Mono, 32-bits
    { 0,  1,  4,  5, -1, -1, -1, -1 },  // Stereo, 16-bits
    { 2,  3,  0,  1,  6,  7,  4,  5 },  // Stereo, 32-bits
};



const clock_audio_pll_config_t audioPllConfig = {
    .loopDivider = 32,  /* PLL loop divider. Valid range for DIV_SELECT divider value: 27~54. */
    .postDivider = 1,   /* Divider after the PLL, should only be 1, 2, 4, 8, 16. */
    .numerator   = 77,  /* 30 bit numerator of fractional loop divider. */
    .denominator = 100, /* 30 bit denominator of fractional loop divider */
};

static edma_handle_t g_dmaHandle = {0};
AT_NONCACHEABLE_SECTION_ALIGN(static edma_tcd_t s_emdaTcd, 32);

static volatile bool s_Transfer_Done        = false;
static volatile uint32_t s_playIndex        = 0U;
static volatile uint32_t s_playCount        = 0U;
static volatile bool s_transferComplete     = false;
static volatile bool s_transferHalfComplete = false;


/* Select Audio/Video PLL (786.48 MHz) as sai1 clock source */
#define DEMO_SAI1_CLOCK_SOURCE_SELECT (2U)
/* Clock pre divider for sai1 clock source */
#define DEMO_SAI1_CLOCK_SOURCE_PRE_DIVIDER (1U)
/* Clock divider for sai1 clock source */
#define DEMO_SAI1_CLOCK_SOURCE_DIVIDER (63U)

/* DMA */
#define DMAMUX0            DMAMUX
#define DEMO_DMA           DMA0
#define DEMO_EDMA_CHANNEL  (0U)
#define DEMO_SAI_TX_SOURCE kDmaRequestMuxSai1Tx

#define DEMO_SAI_RX_SYNC_MODE kSAI_ModeSync


#define BOARD_SAI_RXCONFIG(config, mode)


#define DEMO_SAI_CHANNEL      (0)

#define DEMO_AUDIO_DATA_CHANNEL (2U)  // this is the number of channels Stereo = 2
#define DEMO_AUDIO_BIT_WIDTH    kSAI_WordWidth16bits
#define DEMO_AUDIO_SAMPLE_RATE  (kSAI_SampleRate16KHz)
#define DEMO_AUDIO_MASTER_CLOCK DEMO_SAI_CLK_FREQ

/* Get frequency of sai1 clock */
#define DEMO_SAI_CLK_FREQ                                                        \
    (CLOCK_GetFreq(kCLOCK_AudioPllClk) / (DEMO_SAI1_CLOCK_SOURCE_DIVIDER + 1U) / \
     (DEMO_SAI1_CLOCK_SOURCE_PRE_DIVIDER + 1U))

// TODO this struct is repeated in other peripherals... put into common H file?
typedef struct _iomux_table_t {
    uint32_t muxRegister;
    uint32_t muxMode;
    uint32_t inputRegister;
    uint32_t inputDaisy;
    uint32_t configRegister;
} iomux_table_t;

// TODO is it possible to configure SAI TX such that SAI1_RX_BLK can produce SCK signal?
#define IOMUX_TABLE_I2S \
    { IOMUXC_GPIO_AD_B1_14_SAI1_TX_BCLK }, { IOMUXC_GPIO_AD_B1_15_SAI1_TX_SYNC },  {IOMUXC_GPIO_AD_B1_13_SAI1_TX_DATA00 }

/* from BOARD_InitPins() in pin_mux.c
IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_13_SAI1_TX_DATA00, 1U); 
IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_14_SAI1_TX_BCLK, 1U); 
IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_15_SAI1_TX_SYNC, 1U); 

IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_13_SAI1_TX_DATA00, 0x10B0U); 
IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_14_SAI1_TX_BCLK, 0x10B0U); 
IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_15_SAI1_TX_SYNC, 0x10B0U); 
*/

STATIC const iomux_table_t iomux_table_i2s[] = {
    IOMUX_TABLE_I2S
};

#define SCK (iomux_table_i2s[index])
#define WS (iomux_table_i2s[index + 1])
#define SD (iomux_table_i2s[index + 2])

STATIC bool lpi2s_set_iomux(uint8_t i2s_id) {
    uint8_t index = (i2s_id - 1) * 3;  // TODO fix this ...
    
    // TODO fix this --> hiding "index" inside macro should be improved 
    if (SCK.muxRegister != 0) {
        IOMUXC_SetPinMux(SCK.muxRegister, SCK.muxMode, SCK.inputRegister, SCK.inputDaisy, SCK.configRegister, 1U);
        IOMUXC_SetPinConfig(SCK.muxRegister, SCK.muxMode, SCK.inputRegister, SCK.inputDaisy, SCK.configRegister, 0x10B0u);

        IOMUXC_SetPinMux(WS.muxRegister, WS.muxMode, WS.inputRegister, WS.inputDaisy, WS.configRegister, 1U);
        IOMUXC_SetPinConfig(WS.muxRegister, WS.muxMode, WS.inputRegister, WS.inputDaisy, WS.configRegister, 0x10B0u);
        
        IOMUXC_SetPinMux(SD.muxRegister, SD.muxMode, SD.inputRegister, SD.inputDaisy, SD.configRegister, 1U);
        IOMUXC_SetPinConfig(SD.muxRegister, SD.muxMode, SD.inputRegister, SD.inputDaisy, SD.configRegister, 0x10B0u);
        return true;
    } else {
        return false;
    }
}

void machine_i2s_init0() {
    for (uint8_t i = 0; i < MICROPY_HW_MAX_I2S; i++) {
        MP_STATE_PORT(machine_i2s_obj)[i] = NULL;
    }
}

// Ring Buffer
// Thread safe when used with these constraints:
// - Single Producer, Single Consumer
// - Sequential atomic operations
// One byte of capacity is used to detect buffer empty/full

STATIC void ringbuf_init(ring_buf_t *rbuf, uint8_t *buffer, size_t size) {
    rbuf->buffer = buffer;
    rbuf->size = size;
    rbuf->head = 0;
    rbuf->tail = 0;
}

STATIC bool ringbuf_push(ring_buf_t *rbuf, uint8_t data) {
    size_t next_tail = (rbuf->tail + 1) % rbuf->size;

    if (next_tail != rbuf->head) {
        rbuf->buffer[rbuf->tail] = data;
        rbuf->tail = next_tail;
        return true;
    }

    // full
    return false;
}

STATIC bool ringbuf_pop(ring_buf_t *rbuf, uint8_t *data) {
    if (rbuf->head == rbuf->tail) {
        // empty
        return false;
    }

    *data = rbuf->buffer[rbuf->head];
    rbuf->head = (rbuf->head + 1) % rbuf->size;
    return true;
}

STATIC bool ringbuf_is_empty(ring_buf_t *rbuf) {
    return rbuf->head == rbuf->tail;
}

STATIC bool ringbuf_is_full(ring_buf_t *rbuf) {
    return ((rbuf->tail + 1) % rbuf->size) == rbuf->head;
}

STATIC size_t ringbuf_available_data(ring_buf_t *rbuf) {
    return (rbuf->tail - rbuf->head + rbuf->size) % rbuf->size;
}

STATIC size_t ringbuf_available_space(ring_buf_t *rbuf) {
    return rbuf->size - ringbuf_available_data(rbuf) - 1;
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
//   wav_samples[] = [L_0-7,   L_8-15,  L_16-23, L_24-31, R_0-7,   R_8-15,  R_16-23, R_24-31] =  [Left channel, Right channel]
//   stm_api[] =     [L_16-23, L_24-31, L_0-7,   L_8-15,  R_16-23, R_24-31, R_0-7,   R_8-15] = [Left channel, Right channel]
//
//   where:
//      L_0-7 is the least significant byte of the 32 bit sample in the Left channel
//      L_24-31 is the most significant byte of the 32 bit sample in the Left channel
//
//   wav_samples[] =  [0x99, 0xBB, 0x11, 0x22, 0x44, 0x55, 0xAB, 0x77] = [Left channel, Right channel]
//   stm_api[] =      [0x11, 0x22, 0x99, 0xBB, 0xAB, 0x77, 0x44, 0x55] = [Left channel, Right channel]
//
//   where:
//      LEFT Channel =  0x99, 0xBB, 0x11, 0x22
//      RIGHT Channel = 0x44, 0x55, 0xAB, 0x77
STATIC void reformat_32_bit_samples(int32_t *sample, uint32_t num_samples) {
    int16_t sample_ms;
    int16_t sample_ls;
    for (uint32_t i = 0; i < num_samples; i++) {
        sample_ls = sample[i] & 0xFFFF;
        sample_ms = sample[i] >> 16;
        sample[i] = (sample_ls << 16) + sample_ms;
    }
}

STATIC int8_t get_frame_mapping_index(int8_t bits, format_t format) {
    if (format == MONO) {
        if (bits == 16) {
            return 0;
        } else { // 32 bits
            return 1;
        }
    } else { // STEREO
        if (bits == 16) {
            return 2;
        } else { // 32 bits
            return 3;
        }
    }
}

#if 0
STATIC int8_t get_dma_bits(uint16_t mode, int8_t bits) {
    if (mode == TX) {
        if (bits == 16) {
            return kSAI_WordWidth16bits;
        } else {
            return kSAI_WordWidth32bits;
        }
        return bits;
    } else { // Master Rx
        // always read 32 bit words for I2S e.g.  I2S MEMS microphones
        return kSAI_WordWidth32bits;
    }
}
#endif

STATIC uint32_t fill_appbuf_from_ringbuf(machine_i2s_obj_t *self, mp_buffer_info_t *appbuf) {

    // copy audio samples from the ring buffer to the app buffer
    // loop, copying samples until the app buffer is filled
    // For uasyncio mode, the loop will make an early exit if the ring buffer becomes empty
    // Example:
    //   a MicroPython I2S object is configured for 16-bit mono (2 bytes per audio sample).
    //   For every frame coming from the ring buffer (8 bytes), 2 bytes are "cherry picked" and
    //   copied to the supplied app buffer.
    //   Thus, for every 1 byte copied to the app buffer, 4 bytes are read from the ring buffer.
    //   If a 8kB app buffer is supplied, 32kB of audio samples is read from the ring buffer.

    uint32_t num_bytes_copied_to_appbuf = 0;
    uint8_t *app_p = (uint8_t *)appbuf->buf;
    uint8_t appbuf_sample_size_in_bytes = (self->bits == 16? 2 : 4) * (self->format == STEREO ? 2: 1);
    uint32_t num_bytes_needed_from_ringbuf = appbuf->len * (I2S_RX_FRAME_SIZE_IN_BYTES / appbuf_sample_size_in_bytes);
    uint8_t discard_byte;
    while (num_bytes_needed_from_ringbuf) {

        uint8_t f_index = get_frame_mapping_index(self->bits, self->format);

        for (uint8_t i = 0; i < I2S_RX_FRAME_SIZE_IN_BYTES; i++) {
            int8_t r_to_a_mapping = i2s_frame_map[f_index][i];
            if (r_to_a_mapping != -1) {
                if (self->io_mode == BLOCKING) {
                    // poll the ringbuf until a sample becomes available,  copy into appbuf using the mapping transform
                    while (ringbuf_pop(&self->ring_buffer, app_p + r_to_a_mapping) == false) {
                        ;
                    }
                    num_bytes_copied_to_appbuf++;
                } else if (self->io_mode == UASYNCIO) {
                    if (ringbuf_pop(&self->ring_buffer, app_p + r_to_a_mapping) == false) {
                        // ring buffer is empty, exit
                        goto exit;
                    } else {
                        num_bytes_copied_to_appbuf++;
                    }
                } else {
                    return 0;  // should never get here (non-blocking mode does not use this function)
                }
            } else { // r_a_mapping == -1
                // discard unused byte from ring buffer
                if (self->io_mode == BLOCKING) {
                    // poll the ringbuf until a sample becomes available
                    while (ringbuf_pop(&self->ring_buffer, &discard_byte) == false) {
                        ;
                    }
                } else if (self->io_mode == UASYNCIO) {
                    if (ringbuf_pop(&self->ring_buffer, &discard_byte) == false) {
                        // ring buffer is empty, exit
                        goto exit;
                    }
                } else {
                    return 0;  // should never get here (non-blocking mode does not use this function)
                }
            }
            num_bytes_needed_from_ringbuf--;
        }
        app_p += appbuf_sample_size_in_bytes;
    }
exit:
    return num_bytes_copied_to_appbuf;
}

// function is used in IRQ context
STATIC void fill_appbuf_from_ringbuf_non_blocking(machine_i2s_obj_t *self) {

    // attempt to copy a block of audio samples from the ring buffer to the supplied app buffer.
    // audio samples will be formatted as part of the copy operation

    uint32_t num_bytes_copied_to_appbuf = 0;
    uint8_t *app_p = &(((uint8_t *)self->non_blocking_descriptor.appbuf.buf)[self->non_blocking_descriptor.index]);

    uint8_t appbuf_sample_size_in_bytes = (self->bits == 16? 2 : 4) * (self->format == STEREO ? 2: 1);
    uint32_t num_bytes_remaining_to_copy_to_appbuf = self->non_blocking_descriptor.appbuf.len - self->non_blocking_descriptor.index;
    uint32_t num_bytes_remaining_to_copy_from_ring_buffer = num_bytes_remaining_to_copy_to_appbuf *
        (I2S_RX_FRAME_SIZE_IN_BYTES / appbuf_sample_size_in_bytes);
    uint32_t num_bytes_needed_from_ringbuf = MIN(SIZEOF_NON_BLOCKING_COPY_IN_BYTES, num_bytes_remaining_to_copy_from_ring_buffer);
    uint8_t discard_byte;
    if (ringbuf_available_data(&self->ring_buffer) >= num_bytes_needed_from_ringbuf) {
        while (num_bytes_needed_from_ringbuf) {

            uint8_t f_index = get_frame_mapping_index(self->bits, self->format);

            for (uint8_t i = 0; i < I2S_RX_FRAME_SIZE_IN_BYTES; i++) {
                int8_t r_to_a_mapping = i2s_frame_map[f_index][i];
                if (r_to_a_mapping != -1) {
                    ringbuf_pop(&self->ring_buffer, app_p + r_to_a_mapping);
                    num_bytes_copied_to_appbuf++;
                } else { // r_a_mapping == -1
                    // discard unused byte from ring buffer
                    ringbuf_pop(&self->ring_buffer, &discard_byte);
                }
                num_bytes_needed_from_ringbuf--;
            }
            app_p += appbuf_sample_size_in_bytes;
        }
        self->non_blocking_descriptor.index += num_bytes_copied_to_appbuf;

        if (self->non_blocking_descriptor.index >= self->non_blocking_descriptor.appbuf.len) {
            self->non_blocking_descriptor.copy_in_progress = false;
            mp_sched_schedule(self->callback_for_non_blocking, MP_OBJ_FROM_PTR(self));
        }
    }
}

STATIC uint32_t copy_appbuf_to_ringbuf(machine_i2s_obj_t *self, mp_buffer_info_t *appbuf) {

    // copy audio samples from the app buffer to the ring buffer
    // loop, reading samples until the app buffer is emptied
    // for uasyncio mode, the loop will make an early exit if the ring buffer becomes full

    uint32_t a_index = 0;

    while (a_index < appbuf->len) {
        if (self->io_mode == BLOCKING) {
            // copy a byte to the ringbuf when space becomes available
            while (ringbuf_push(&self->ring_buffer, ((uint8_t *)appbuf->buf)[a_index]) == false) {
                ;
            }
            a_index++;
        } else if (self->io_mode == UASYNCIO) {
            if (ringbuf_push(&self->ring_buffer, ((uint8_t *)appbuf->buf)[a_index]) == false) {
                // ring buffer is full, exit
                break;
            } else {
                a_index++;
            }
        } else {
            return 0;  // should never get here (non-blocking mode does not use this function)
        }
    }

    return a_index;
}

// function is used in IRQ context
STATIC void copy_appbuf_to_ringbuf_non_blocking(machine_i2s_obj_t *self) {

    // copy audio samples from app buffer into ring buffer
    uint32_t num_bytes_remaining_to_copy = self->non_blocking_descriptor.appbuf.len - self->non_blocking_descriptor.index;
    uint32_t num_bytes_to_copy = MIN(SIZEOF_NON_BLOCKING_COPY_IN_BYTES, num_bytes_remaining_to_copy);

    if (ringbuf_available_space(&self->ring_buffer) >= num_bytes_to_copy) {
        for (uint32_t i = 0; i < num_bytes_to_copy; i++) {
            ringbuf_push(&self->ring_buffer,
                ((uint8_t *)self->non_blocking_descriptor.appbuf.buf)[self->non_blocking_descriptor.index + i]);
        }

        self->non_blocking_descriptor.index += num_bytes_to_copy;
        if (self->non_blocking_descriptor.index >= self->non_blocking_descriptor.appbuf.len) {
            self->non_blocking_descriptor.copy_in_progress = false;
            mp_sched_schedule(self->callback_for_non_blocking, MP_OBJ_FROM_PTR(self));
        }
    }
}

// function is used in IRQ context
STATIC void empty_dma(machine_i2s_obj_t *self, ping_pong_t dma_ping_pong) {
    uint16_t dma_buffer_offset = 0;

    if (dma_ping_pong == TOP_HALF) {
        dma_buffer_offset = 0;
    } else { // BOTTOM_HALF
        dma_buffer_offset = SIZEOF_HALF_DMA_BUFFER_IN_BYTES;
    }

    uint8_t *dma_buffer_p = &self->dma_buffer_dcache_aligned[dma_buffer_offset];

#if 0  // TODO
    // flush and invalidate cache so the CPU reads data placed into RAM by DMA
    MP_HAL_CLEANINVALIDATE_DCACHE(dma_buffer_p, SIZEOF_HALF_DMA_BUFFER_IN_BYTES);
#endif

    // when space exists, copy samples into ring buffer
    if (ringbuf_available_space(&self->ring_buffer) >= SIZEOF_HALF_DMA_BUFFER_IN_BYTES) {
        for (uint32_t i = 0; i < SIZEOF_HALF_DMA_BUFFER_IN_BYTES; i++) {
            ringbuf_push(&self->ring_buffer, dma_buffer_p[i]);
        }
    }
}

// function is used in IRQ context
STATIC void feed_dma(machine_i2s_obj_t *self, ping_pong_t dma_ping_pong) {
    uint16_t dma_buffer_offset = 0;

    if (dma_ping_pong == TOP_HALF) {
        dma_buffer_offset = 0;
    } else { // BOTTOM_HALF
        dma_buffer_offset = SIZEOF_HALF_DMA_BUFFER_IN_BYTES;
    }

    uint8_t *dma_buffer_p = &self->dma_buffer_dcache_aligned[dma_buffer_offset];

    // when data exists, copy samples from ring buffer
    if (ringbuf_available_data(&self->ring_buffer) >= SIZEOF_HALF_DMA_BUFFER_IN_BYTES) {

        // copy a block of samples from the ring buffer to the dma buffer.
        // STM32 HAL API has a stereo I2S implementation, but not mono
        // mono format is implemented by duplicating each sample into both L and R channels.
        if ((self->format == MONO) && (self->bits == 16)) {
            for (uint32_t i = 0; i < SIZEOF_HALF_DMA_BUFFER_IN_BYTES / 4; i++) {
                for (uint8_t b = 0; b < sizeof(uint16_t); b++) {
                    ringbuf_pop(&self->ring_buffer, &dma_buffer_p[i * 4 + b]);
                    dma_buffer_p[i * 4 + b + 2] = dma_buffer_p[i * 4 + b]; // duplicated mono sample
                }
            }
        } else if ((self->format == MONO) && (self->bits == 32)) {
            for (uint32_t i = 0; i < SIZEOF_HALF_DMA_BUFFER_IN_BYTES / 8; i++) {
                for (uint8_t b = 0; b < sizeof(uint32_t); b++) {
                    ringbuf_pop(&self->ring_buffer, &dma_buffer_p[i * 8 + b]);
                    dma_buffer_p[i * 8 + b + 4] = dma_buffer_p[i * 8 + b]; // duplicated mono sample
                }
            }
        } else { // STEREO, both 16-bit and 32-bit
            for (uint32_t i = 0; i < SIZEOF_HALF_DMA_BUFFER_IN_BYTES; i++) {
                ringbuf_pop(&self->ring_buffer, &dma_buffer_p[i]);
            }
        }

        // reformat 32 bit samples to match STM32 HAL API format
        if (self->bits == 32) {
            reformat_32_bit_samples((int32_t *)dma_buffer_p, SIZEOF_HALF_DMA_BUFFER_IN_BYTES / (sizeof(uint32_t)));
        }
    } else {
        // underflow.  clear buffer to transmit "silence" on the I2S bus
        memset(dma_buffer_p, 0, SIZEOF_HALF_DMA_BUFFER_IN_BYTES);
    }

    // flush cache to RAM so DMA can read the sample data
    MP_HAL_CLEAN_DCACHE(dma_buffer_p, SIZEOF_HALF_DMA_BUFFER_IN_BYTES);
}

#if 0
void HAL_I2S_ErrorCallback(I2S_HandleTypeDef *hi2s) {
    uint32_t errorCode = HAL_I2S_GetError(hi2s);
    printf("I2S Error = %ld\n", errorCode);
}
#endif

#if 0
void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s) {
#else
void HAL_I2S_RxCpltCallback(void) {
#endif
    machine_i2s_obj_t *self;
#if 0
    if (hi2s->Instance == I2S1) {
#else
    if(1) {
#endif
        self = MP_STATE_PORT(machine_i2s_obj)[0];
    } else {
        self = MP_STATE_PORT(machine_i2s_obj)[1];
    }
    // bottom half of buffer now filled,
    // safe to empty the bottom half while the top half of buffer is being filled
    empty_dma(self, BOTTOM_HALF);

    // for non-blocking operation, this IRQ-based callback handles
    // the readinto() method requests.
    if ((self->io_mode == NON_BLOCKING) && (self->non_blocking_descriptor.copy_in_progress)) {
        fill_appbuf_from_ringbuf_non_blocking(self);
    }
}

#if 0
void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
#else
void HAL_I2S_RxHalfCpltCallback(void) {
#endif
    machine_i2s_obj_t *self;
#if 0
    if (hi2s->Instance == I2S1) {
#else
    if(1) {
#endif
        self = MP_STATE_PORT(machine_i2s_obj)[0];
    } else {
        self = MP_STATE_PORT(machine_i2s_obj)[1];
    }
    // top half of buffer now filled,
    // safe to empty the top half while the bottom half of buffer is being filled
    empty_dma(self, TOP_HALF);

    // for non-blocking operation, this IRQ-based callback handles
    // the readinto() method requests.
    if ((self->io_mode == NON_BLOCKING) && (self->non_blocking_descriptor.copy_in_progress)) {
        fill_appbuf_from_ringbuf_non_blocking(self);
    }
}

void HAL_I2S_TxCpltCallback(machine_i2s_obj_t *self) {
    // for non-blocking operation, this IRQ-based callback handles
    // the write() method requests.
    if ((self->io_mode == NON_BLOCKING) && (self->non_blocking_descriptor.copy_in_progress)) {
        copy_appbuf_to_ringbuf_non_blocking(self);
    }

    // bottom half of buffer now emptied,
    // safe to fill the bottom half while the top half of buffer is being emptied
    feed_dma(self, BOTTOM_HALF);
}

void HAL_I2S_TxHalfCpltCallback(machine_i2s_obj_t *self) {
    // for non-blocking operation, this IRQ-based callback handles
    // the write() method requests.
    if ((self->io_mode == NON_BLOCKING) && (self->non_blocking_descriptor.copy_in_progress)) {
        copy_appbuf_to_ringbuf_non_blocking(self);
    }

    // top half of buffer now emptied,
    // safe to fill the top half while the bottom half of buffer is being emptied
    feed_dma(self, TOP_HALF);
}


// output ~200us pulse
void diag_pulse_gpio(uint16_t pin_num) {
    const machine_pin_obj_t *pin = machine_pin_board_pins[pin_num];

    for (uint32_t i=0; i<10000; i++) {
        mp_hal_pin_high(pin);
    }

    mp_hal_pin_low(pin);
}



    /*! @brief Define callback function for eDMA.
     *
     * This callback function is called in the EDMA interrupt handle.
     * In normal mode, run into callback function means the transfer users need is done.
     * In scatter gather mode, run into callback function means a transfer control block (tcd) is finished. Not
     * all transfer finished, users can get the finished tcd numbers using interface EDMA_GetUnusedTCDNumber.
     *
     * @param handle EDMA handle pointer, users shall not touch the values inside.
     * @param userData The callback user parameter pointer. Users can use this parameter to involve things users need to
     *                 change in EDMA callback function.
     * @param transferDone If the current loaded transfer done. In normal mode it means if all transfer done. In scatter
     *                     gather mode, this parameter shows is the current transfer block in EDMA register is done. As the
     *                     load of core is different, it will be different if the new tcd loaded into EDMA registers while
     *                     this callback called. If true, it always means new tcd still not loaded into registers, while
     *                     false means new tcd already loaded into registers.
     * @param tcds How many tcds are done from the last callback. This parameter only used in scatter gather mode. It
     *             tells user how many tcds are finished between the last callback and this.
     */
    //typedef void (*edma_callback)(struct _edma_handle *handle, void *userData, bool transferDone, uint32_t tcds);


/* User callback function for EDMA transfer. */
void EDMA_TX_Callback(edma_handle_t *handle, void *userData, bool transferDone, uint32_t tcds)
{
    machine_i2s_obj_t *self = userData;
    //mp_printf(MP_PYTHON_PRINTER, "i2s_id = %d, transferDone = %d\n", self->i2s_id, transferDone);

    // pulse GPIO pin
    //diag_pulse_gpio(32);

    if (transferDone)
    {
        HAL_I2S_TxCpltCallback(self);
    }
    else
    {
        HAL_I2S_TxHalfCpltCallback(self);
    }
}


STATIC bool i2s_init(machine_i2s_obj_t *self) {
    
    //mp_printf(MP_PYTHON_PRINTER, "start i2s_init()\n");

    CLOCK_InitAudioPll(&audioPllConfig);
    
    /*Clock setting for SAI1*/
    CLOCK_SetMux(kCLOCK_Sai1Mux, DEMO_SAI1_CLOCK_SOURCE_SELECT);
    CLOCK_SetDiv(kCLOCK_Sai1PreDiv, DEMO_SAI1_CLOCK_SOURCE_PRE_DIVIDER);
    CLOCK_SetDiv(kCLOCK_Sai1Div, DEMO_SAI1_CLOCK_SOURCE_DIVIDER);

    // Configure board-specific pin MUX based on the hardware device number.
    lpi2s_set_iomux(self->i2s_id);
    
    /* Init DMAMUX */
    int chan = allocate_dma_channel();
    DMAMUX_Init(DMAMUX0);
    DMAMUX_SetSource(DMAMUX0, chan, DEMO_SAI_TX_SOURCE);
    DMAMUX_EnableChannel(DMAMUX0, chan);
    
    /* Create EDMA handle */
    /*
     * dmaConfig.enableRoundRobinArbitration = false;
     * dmaConfig.enableHaltOnError = true;
     * dmaConfig.enableContinuousLinkMode = false;
     * dmaConfig.enableDebugMode = false;
     */
    edma_config_t dmaConfig;
    memset(&dmaConfig, 0, sizeof(dmaConfig));
    EDMA_GetDefaultConfig(&dmaConfig);
    EDMA_Init(DMA0, &dmaConfig);  // TODO SPI has this init as well; will this conflict with SPI operation?

    EDMA_CreateHandle(&g_dmaHandle, DMA0, chan);
    EDMA_SetCallback(&g_dmaHandle, EDMA_TX_Callback, self);
    EDMA_ResetChannel(DMA0, chan);
    
    /* SAI init */
    SAI_Init(SAI1);
    
    /* I2S mode configurations */
    sai_transceiver_t saiConfig;
    memset(&saiConfig, 0, sizeof(saiConfig));
    SAI_GetClassicI2SConfig(&saiConfig, kSAI_WordWidth16bits, kSAI_Stereo, 1U << DEMO_SAI_CHANNEL);
    saiConfig.syncMode    = kSAI_ModeAsync;
    saiConfig.masterSlave = kSAI_Master;
    SAI_TxSetConfig(SAI1, &saiConfig);
    /* set bit clock divider */
    SAI_TxSetBitClockRate(SAI1, DEMO_AUDIO_MASTER_CLOCK, DEMO_AUDIO_SAMPLE_RATE, DEMO_AUDIO_BIT_WIDTH,
                          DEMO_AUDIO_DATA_CHANNEL);
    /* sai rx configurations */
    //BOARD_SAI_RXCONFIG(&saiConfig, DEMO_SAI_RX_SYNC_MODE);  // TODO likely not needed
    
    MP_HAL_CLEAN_DCACHE(self->dma_buffer_dcache_aligned, SIZEOF_DMA_BUFFER_IN_BYTES);

    /* Configure and submit transfer structure 1 */

    // TODO see if the combined SAI/EDMA API would simplify the implementation:  e.g.  SAI_TransferSendEDMA()
    uint32_t destAddr = SAI_TxGetDataRegisterAddress(SAI1, DEMO_SAI_CHANNEL);

    edma_transfer_config_t transferConfig;
    memset(&transferConfig, 0, sizeof(transferConfig));
    EDMA_PrepareTransfer(&transferConfig,
            self->dma_buffer_dcache_aligned, kSAI_WordWidth16bits / 8U,
            (void *)destAddr, kSAI_WordWidth16bits / 8U,
            (FSL_FEATURE_SAI_FIFO_COUNT - saiConfig.fifo.fifoWatermark) * (kSAI_WordWidth16bits / 8U),
            SIZEOF_DMA_BUFFER_IN_BYTES, kEDMA_MemoryToPeripheral);
    memset(&s_emdaTcd, 0, sizeof(s_emdaTcd));
    EDMA_TcdSetTransferConfig(&s_emdaTcd, &transferConfig, &s_emdaTcd);
    EDMA_TcdEnableInterrupts(&s_emdaTcd, kEDMA_MajorInterruptEnable | kEDMA_HalfInterruptEnable);
    EDMA_InstallTCD(DMA0, chan, &s_emdaTcd);
    EDMA_StartTransfer(&g_dmaHandle);

    /* Enable DMA enable bit */
    SAI_TxEnableDMA(SAI1, kSAI_FIFORequestDMAEnable, true);
    /* Enable SAI Tx clock */
    SAI_TxEnable(SAI1, true);  // <--- SD Card failure and .Format failure with this line
    /* Enable the channel FIFO */
    SAI_TxSetChannelFIFOMask(SAI1, 1U << DEMO_SAI_CHANNEL);

#if 0  // TODO this goes into deinit method
    /* Disable the channel FIFO */
    SAI_TxSetChannelFIFOMask(SAI1, 0U << DEMO_SAI_CHANNEL);

    /* Disable SAI Tx clock */
    SAI_TxEnable(SAI1, false);

    /* Disable DMA enable bit */
    SAI_TxEnableDMA(SAI1, kSAI_FIFORequestDMAEnable, false);

    /* Disable SAI instance. */
    SAI_Deinit(SAI1);
#endif

    return true;  // TODO to get it compiling
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
        ARG_ibuf,
    };

    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_sck,      MP_ARG_KW_ONLY | MP_ARG_REQUIRED | MP_ARG_OBJ,   {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_ws,       MP_ARG_KW_ONLY | MP_ARG_REQUIRED | MP_ARG_OBJ,   {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_sd,       MP_ARG_KW_ONLY | MP_ARG_REQUIRED | MP_ARG_OBJ,   {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_mode,     MP_ARG_KW_ONLY | MP_ARG_REQUIRED | MP_ARG_INT,   {.u_int = -1} },
        { MP_QSTR_bits,     MP_ARG_KW_ONLY | MP_ARG_REQUIRED | MP_ARG_INT,   {.u_int = -1} },
        { MP_QSTR_format,   MP_ARG_KW_ONLY | MP_ARG_REQUIRED | MP_ARG_INT,   {.u_int = -1} },
        { MP_QSTR_rate,     MP_ARG_KW_ONLY | MP_ARG_REQUIRED | MP_ARG_INT,   {.u_int = -1} },
        { MP_QSTR_ibuf,     MP_ARG_KW_ONLY | MP_ARG_REQUIRED | MP_ARG_INT,   {.u_int = -1} },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_pos_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

#if 0    
    memset(&self->hi2s, 0, sizeof(self->hi2s));
#endif
    //
    // ---- Check validity of arguments ----
    //

    // are I2S pin assignments valid?
#if 0
    const machine_pin_obj_t *pin = pin_find(args[ARG_sck].u_obj);
    
    typedef struct {
        mp_obj_base_t base;
        qstr name;  // pad name
        GPIO_Type *gpio;  // gpio instance for pin
        uint32_t pin;  // pin number
        uint32_t muxRegister;
        uint32_t configRegister;
        uint8_t af_list_len;  // length of available alternate functions list
        uint8_t adc_list_len; // length of available ADC options list
        const machine_pin_af_obj_t *af_list;  // pointer to list with alternate functions
        const machine_pin_adc_obj_t *adc_list; // pointer to list with ADC options
    } machine_pin_obj_t;
#endif
    
    
#if 0    
    const pin_af_obj_t *pin_af;
    
    // is SCK valid?
    if (mp_obj_is_type(args[ARG_sck].u_obj, &pin_type)) {
        pin_af = pin_find_af(MP_OBJ_TO_PTR(args[ARG_sck].u_obj), AF_FN_I2S, self->i2s_id);
        if (pin_af->type != AF_PIN_TYPE_I2S_CK) {
            mp_raise_ValueError(MP_ERROR_TEXT("invalid SCK pin"));
        }
    } else {
        mp_raise_ValueError(MP_ERROR_TEXT("SCK not a Pin type"));
    }

    // is WS valid?
    if (mp_obj_is_type(args[ARG_ws].u_obj, &pin_type)) {
        pin_af = pin_find_af(MP_OBJ_TO_PTR(args[ARG_ws].u_obj), AF_FN_I2S, self->i2s_id);
        if (pin_af->type != AF_PIN_TYPE_I2S_WS) {
            mp_raise_ValueError(MP_ERROR_TEXT("invalid WS pin"));
        }
    } else {
        mp_raise_ValueError(MP_ERROR_TEXT("WS not a Pin type"));
    }

    // is SD valid?
    if (mp_obj_is_type(args[ARG_sd].u_obj, &pin_type)) {
        pin_af = pin_find_af(MP_OBJ_TO_PTR(args[ARG_sd].u_obj), AF_FN_I2S, self->i2s_id);
        if (pin_af->type != AF_PIN_TYPE_I2S_SD) {
            mp_raise_ValueError(MP_ERROR_TEXT("invalid SD pin"));
        }
    } else {
        mp_raise_ValueError(MP_ERROR_TEXT("SD not a Pin type"));
    }
#endif
    
    // is Mode valid?
    uint16_t i2s_mode = args[ARG_mode].u_int;
    if ((i2s_mode != (RX)) &&
        (i2s_mode != (TX))) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid mode"));
    }

    // is Bits valid?
    int8_t i2s_bits = args[ARG_bits].u_int;
    if ((i2s_bits != 16) &&
        (i2s_bits != 32)) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid bits"));
    }

    // is Format valid?
    format_t i2s_format = args[ARG_format].u_int;
    if ((i2s_format != MONO) &&
        (i2s_format != STEREO)) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid format"));
    }

    // is Rate valid?
    // Not checked

    // is Ibuf valid?
    int32_t ring_buffer_len = args[ARG_ibuf].u_int;
    if (ring_buffer_len > 0) {
        uint8_t *buffer = m_new(uint8_t, ring_buffer_len);
        self->ring_buffer_storage = buffer;
        ringbuf_init(&self->ring_buffer, buffer, ring_buffer_len);
    } else {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid ibuf"));
    }

    self->sck = MP_OBJ_TO_PTR(args[ARG_sck].u_obj);
    self->ws = MP_OBJ_TO_PTR(args[ARG_ws].u_obj);
    self->sd = MP_OBJ_TO_PTR(args[ARG_sd].u_obj);
    self->mode = i2s_mode;
    self->bits = i2s_bits;
    self->format = i2s_format;
    self->rate = args[ARG_rate].u_int;
    self->ibuf = ring_buffer_len;
    self->callback_for_non_blocking = MP_OBJ_NULL;
    self->non_blocking_descriptor.copy_in_progress = false;
    self->io_mode = BLOCKING;

#if 0    
    I2S_InitTypeDef *init = &self->hi2s.Init;
    init->Mode = i2s_mode;
    init->Standard = I2S_STANDARD_PHILIPS;
    init->DataFormat = get_dma_bits(self->mode, self->bits);
    init->MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
    init->AudioFreq = args[ARG_rate].u_int;
    init->CPOL = I2S_CPOL_LOW;
    init->ClockSource = I2S_CLOCK_PLL;
    
#endif
    // init the I2S bus
    if (!i2s_init(self)) {
        mp_raise_msg_varg(&mp_type_OSError, MP_ERROR_TEXT("I2S init failed"));
    }
    
#if 0
    
    // start DMA.  DMA is configured to run continuously, using a circular buffer configuration
    uint32_t number_of_samples = 0;
    if (init->DataFormat == kSAI_WordWidth32bits) {
        number_of_samples = SIZEOF_DMA_BUFFER_IN_BYTES / sizeof(uint16_t);
    } else {  // 32 bits
        number_of_samples = SIZEOF_DMA_BUFFER_IN_BYTES / sizeof(uint32_t);
    }

    HAL_StatusTypeDef status;
    if (self->mode == TX) {
        status = HAL_I2S_Transmit_DMA(&self->hi2s, (void *)self->dma_buffer_dcache_aligned, number_of_samples);
    } else {  // RX
        status = HAL_I2S_Receive_DMA(&self->hi2s, (void *)self->dma_buffer_dcache_aligned, number_of_samples);
    }
    if (status != HAL_OK) {
        mp_raise_msg_varg(&mp_type_OSError, MP_ERROR_TEXT("DMA init failed"));
    }
#endif
    
    
    
}

STATIC void machine_i2s_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    machine_i2s_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_printf(print, "I2S(id=%u,\n"
#if 0            
        "sck="MP_HAL_PIN_FMT ",\n"
        "ws="MP_HAL_PIN_FMT ",\n"
        "sd="MP_HAL_PIN_FMT ",\n"
#endif        
        "mode=%u,\n"
        "bits=%u, format=%u,\n"
        "rate=%d, ibuf=%d)",
        self->i2s_id,
#if 0        
        mp_hal_pin_name(self->sck),
        mp_hal_pin_name(self->ws),
        mp_hal_pin_name(self->sd),
#endif        
        self->mode,
        self->bits, self->format,
        self->rate, self->ibuf
        );
}

STATIC mp_obj_t machine_i2s_make_new(const mp_obj_type_t *type, size_t n_pos_args, size_t n_kw_args, const mp_obj_t *args) {
    mp_arg_check_num(n_pos_args, n_kw_args, 1, MP_OBJ_FUN_ARGS_MAX, true);

    uint8_t i2s_id = mp_obj_get_int(args[0]);
    if (i2s_id < 1 || i2s_id > MICROPY_HW_MAX_I2S) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid id"));
    }

    uint8_t i2s_id_zero_base = i2s_id - 1;
    machine_i2s_obj_t *self;
    if (MP_STATE_PORT(machine_i2s_obj)[i2s_id_zero_base] == NULL) {
        self = m_new_obj(machine_i2s_obj_t);
        MP_STATE_PORT(machine_i2s_obj)[i2s_id_zero_base] = self;
        self->base.type = &machine_i2s_type;
        self->i2s_id = i2s_id;
    } else {
        self = MP_STATE_PORT(machine_i2s_obj)[i2s_id_zero_base];
        machine_i2s_deinit(MP_OBJ_FROM_PTR(self));
    }

    // align DMA buffer start to the cache line size (32 bytes)
    self->dma_buffer_dcache_aligned = (uint8_t *)((uint32_t)(self->dma_buffer + 0x1f) & ~0x1f);

    // TODO fill the DMA buffer with NULLs to start?
    memset(self->dma_buffer_dcache_aligned, 0, SIZEOF_DMA_BUFFER_IN_BYTES);

    mp_map_t kw_args;
    mp_map_init_fixed_table(&kw_args, n_kw_args, args + n_pos_args);
    machine_i2s_init_helper(self, n_pos_args - 1, args + 1, &kw_args);
    return MP_OBJ_FROM_PTR(self);
}

STATIC mp_obj_t machine_i2s_init(size_t n_pos_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    machine_i2s_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    machine_i2s_deinit(MP_OBJ_FROM_PTR(self));
    machine_i2s_init_helper(self, n_pos_args - 1, pos_args + 1, kw_args);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(machine_i2s_init_obj, 1, machine_i2s_init);

STATIC mp_obj_t machine_i2s_deinit(mp_obj_t self_in) {

    machine_i2s_obj_t *self = MP_OBJ_TO_PTR(self_in);

#if 0
    dma_deinit(self->dma_descr_tx);
    dma_deinit(self->dma_descr_rx);
    HAL_I2S_DeInit(&self->hi2s);

    if (self->hi2s.Instance == I2S1) {
        __SPI1_FORCE_RESET();
        __SPI1_RELEASE_RESET();
        __SPI1_CLK_DISABLE();
    } else if (self->hi2s.Instance == I2S2) {
        __SPI2_FORCE_RESET();
        __SPI2_RELEASE_RESET();
        __SPI2_CLK_DISABLE();
    }
#endif
    m_free(self->ring_buffer_storage);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(machine_i2s_deinit_obj, machine_i2s_deinit);

STATIC mp_obj_t machine_i2s_irq(mp_obj_t self_in, mp_obj_t handler) {
    machine_i2s_obj_t *self = MP_OBJ_TO_PTR(self_in);
    if (handler != mp_const_none && !mp_obj_is_callable(handler)) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid callback"));
    }

    if (handler != mp_const_none) {
        self->io_mode = NON_BLOCKING;
    } else {
        self->io_mode = BLOCKING;
    }

    self->callback_for_non_blocking = handler;
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(machine_i2s_irq_obj, machine_i2s_irq);

// Shift() is typically used as a volume control.
// shift=1 increases volume by 6dB, shift=-1 decreases volume by 6dB
STATIC mp_obj_t machine_i2s_shift(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_buf, ARG_bits, ARG_shift};
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_buf,    MP_ARG_REQUIRED | MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_bits,   MP_ARG_REQUIRED | MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = -1} },
        { MP_QSTR_shift, MP_ARG_REQUIRED | MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = -1} },
    };

    // parse args
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(args[ARG_buf].u_obj, &bufinfo, MP_BUFFER_RW);

    int16_t *buf_16 = bufinfo.buf;
    int32_t *buf_32 = bufinfo.buf;

    uint8_t bits = args[ARG_bits].u_int;
    int8_t shift = args[ARG_shift].u_int;

    uint32_t num_audio_samples;
    switch (bits) {
        case 16:
            num_audio_samples = bufinfo.len / sizeof(uint16_t);
            break;

        case 32:
            num_audio_samples = bufinfo.len / sizeof(uint32_t);
            break;

        default:
            mp_raise_ValueError(MP_ERROR_TEXT("invalid bits"));
            break;
    }

    for (uint32_t i = 0; i < num_audio_samples; i++) {
        switch (bits) {
            case 16:
                if (shift >= 0) {
                    buf_16[i] = buf_16[i] << shift;
                } else {
                    buf_16[i] = buf_16[i] >> abs(shift);
                }
                break;
            case 32:
                if (shift >= 0) {
                    buf_32[i] = buf_32[i] << shift;
                } else {
                    buf_32[i] = buf_32[i] >> abs(shift);
                }
                break;
        }
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(machine_i2s_shift_fun_obj, 0, machine_i2s_shift);
STATIC MP_DEFINE_CONST_STATICMETHOD_OBJ(machine_i2s_shift_obj, MP_ROM_PTR(&machine_i2s_shift_fun_obj));

STATIC const mp_rom_map_elem_t machine_i2s_locals_dict_table[] = {
    // Methods
    { MP_ROM_QSTR(MP_QSTR_init),            MP_ROM_PTR(&machine_i2s_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_readinto),        MP_ROM_PTR(&mp_stream_readinto_obj) },
    { MP_ROM_QSTR(MP_QSTR_write),           MP_ROM_PTR(&mp_stream_write_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinit),          MP_ROM_PTR(&machine_i2s_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR_irq),             MP_ROM_PTR(&machine_i2s_irq_obj) },

    // Static method
    { MP_ROM_QSTR(MP_QSTR_shift),           MP_ROM_PTR(&machine_i2s_shift_obj) },

    // Constants
    { MP_ROM_QSTR(MP_QSTR_RX),              MP_ROM_INT(RX) },
    { MP_ROM_QSTR(MP_QSTR_TX),              MP_ROM_INT(TX) },
    { MP_ROM_QSTR(MP_QSTR_STEREO),          MP_ROM_INT(STEREO) },
    { MP_ROM_QSTR(MP_QSTR_MONO),            MP_ROM_INT(MONO) },
};
MP_DEFINE_CONST_DICT(machine_i2s_locals_dict, machine_i2s_locals_dict_table);

STATIC mp_uint_t machine_i2s_stream_read(mp_obj_t self_in, void *buf_in, mp_uint_t size, int *errcode) {
    machine_i2s_obj_t *self = MP_OBJ_TO_PTR(self_in);

    if (self->mode != RX) {
        *errcode = MP_EPERM;
        return MP_STREAM_ERROR;
    }

    uint8_t appbuf_sample_size_in_bytes = (self->bits / 8) * (self->format == STEREO ? 2: 1);
    if (size % appbuf_sample_size_in_bytes != 0) {
        *errcode = MP_EINVAL;
        return MP_STREAM_ERROR;
    }

    if (size == 0) {
        return 0;
    }

    if (self->io_mode == NON_BLOCKING) {
        self->non_blocking_descriptor.appbuf.buf = (void *)buf_in;
        self->non_blocking_descriptor.appbuf.len = size;
        self->non_blocking_descriptor.index = 0;
        self->non_blocking_descriptor.copy_in_progress = true;
        return size;
    } else { // blocking or uasyncio mode
        mp_buffer_info_t appbuf;
        appbuf.buf = (void *)buf_in;
        appbuf.len = size;
        uint32_t num_bytes_read = fill_appbuf_from_ringbuf(self, &appbuf);
        return num_bytes_read;
    }
}

STATIC mp_uint_t machine_i2s_stream_write(mp_obj_t self_in, const void *buf_in, mp_uint_t size, int *errcode) {
    machine_i2s_obj_t *self = MP_OBJ_TO_PTR(self_in);

    if (self->mode != TX) {
        *errcode = MP_EPERM;
        return MP_STREAM_ERROR;
    }

    if (size == 0) {
        return 0;
    }

    if (self->io_mode == NON_BLOCKING) {
        self->non_blocking_descriptor.appbuf.buf = (void *)buf_in;
        self->non_blocking_descriptor.appbuf.len = size;
        self->non_blocking_descriptor.index = 0;
        self->non_blocking_descriptor.copy_in_progress = true;
        return size;
    } else { // blocking or uasyncio mode
        mp_buffer_info_t appbuf;
        appbuf.buf = (void *)buf_in;
        appbuf.len = size;
        uint32_t num_bytes_written = copy_appbuf_to_ringbuf(self, &appbuf);
        return num_bytes_written;
    }
}

STATIC mp_uint_t machine_i2s_ioctl(mp_obj_t self_in, mp_uint_t request, uintptr_t arg, int *errcode) {
    machine_i2s_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_uint_t ret;
    uintptr_t flags = arg;
    self->io_mode = UASYNCIO; // a call to ioctl() is an indication that uasyncio is being used

    if (request == MP_STREAM_POLL) {
        ret = 0;

        if (flags & MP_STREAM_POLL_RD) {
            if (self->mode != RX) {
                *errcode = MP_EPERM;
                return MP_STREAM_ERROR;
            }

            if (!ringbuf_is_empty(&self->ring_buffer)) {
                ret |= MP_STREAM_POLL_RD;
            }
        }

        if (flags & MP_STREAM_POLL_WR) {
            if (self->mode != TX) {
                *errcode = MP_EPERM;
                return MP_STREAM_ERROR;
            }

            if (!ringbuf_is_full(&self->ring_buffer)) {
                ret |= MP_STREAM_POLL_WR;
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

#endif // MICROPY_PY_MACHINE_I2S
