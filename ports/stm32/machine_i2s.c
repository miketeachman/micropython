/*
 * This file is part of the Micro Python project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2013, 2014 Damien P. George
 * Copyright (c) 2015 Bryan Morrissey
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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include "py/nlr.h"
#include "py/runtime.h"
#include "py/gc.h"
#include "py/objstr.h"
#include "py/objlist.h"
#include "py/mphal.h"
#include "py/stream.h"
#include "irq.h"
#include "pin.h"
#include "genhdr/pins.h"
#include "dma.h"
#include "bufhelper.h"
#include "modmachine.h"
#include "led.h" // For debugging using led_toggle(n)

// Emperical testing show that PyBoard D boards produce distorted audio for both write and read modes
// of operation. A workaround exists: DMA buffer memory is defined in file scope rather than 
// on the MicroPython Heap.
// The PyBoard V1.1 does not have this problem.
// TODO determine the root cause of the audio problems in the PyBoard D boards
#if defined (STM32F722xx) || defined (STM32F723xx)
#define STATIC_DMA_BUFFER 1  // buffer allocated from static memory
#else
#define STATIC_DMA_BUFFER 0  // buffer allocated from MicroPython heap
#endif

// Pyboard V1.0/V1.1:   Two standard I2S interfaces (multiplexed with SPI2 and SPI3) are available
// Pyboard D SF2W, SF3W:  Three standard I2S interfaces (multiplexed with SPI1, SPI2 and SPI3) are available

// add "how it works" section:  TODO
// describe: 
// - queues, active buffer
// - circular mode for continuous streaming
// - asynchronous design
// - call back
// - use of STM32 HAL API
// - IRQ callbacks are the engines that drive the whole thing

//  STM32 buffer formats for write() and readinto() methods:
// ****** TODO **** update for STM32
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

// === HOW IT WORKS ===  TODO
//
// Blocking
//
// Non-Blocking
//
// Uasyncio
//
// Streaming


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

#define SIZEOF_DMA_BUFFER_IN_BYTES (256)  // TODO what is the minimal size for acceptable performance?
#define SIZEOF_HALF_DMA_BUFFER_IN_BYTES (SIZEOF_DMA_BUFFER_IN_BYTES / 2)
#define SIZEOF_NON_BLOCKING_COPY_IN_BYTES (SIZEOF_HALF_DMA_BUFFER_IN_BYTES * 4)  // TODO what is this for ?

typedef enum {
    TOP_HALF    = 0,
    BOTTOM_HALF = 1,
} machine_i2s_dma_buffer_ping_pong_t;  // TODO crazy long name

typedef enum {
    MONO   = 0,
    STEREO = 1,
} machine_i2s_format_t;

typedef struct _circular_buf_t {
    uint8_t *buffer;
    size_t head;  // ISR can modify
    size_t tail;  // ISR can modify
    size_t max;   // of the buffer
    bool full;
} circular_buf_t;

typedef struct _non_blocking_info_t {
    mp_buffer_info_t            bufinfo;
    uint32_t                    bufindex;
} non_blocking_info_t;

typedef struct _machine_i2s_obj_t {
    mp_obj_base_t           base;
    uint8_t                 i2s_id;
    I2S_HandleTypeDef       i2s;
    const dma_descr_t       *tx_dma_descr;
    const dma_descr_t       *rx_dma_descr; 
    DMA_HandleTypeDef       tx_dma;
    DMA_HandleTypeDef       rx_dma;
    mp_obj_t                callback;
#if STATIC_DMA_BUFFER
    uint8_t                 *dma_buffer;
#else
    uint8_t                 dma_buffer[SIZEOF_DMA_BUFFER_IN_BYTES];
#endif
    pin_obj_t               *sck;
    pin_obj_t               *ws;
    pin_obj_t               *sd;
    uint16_t                mode;
    int8_t                  bits;
    machine_i2s_format_t    format;
    int32_t                 rate;
    int32_t                 bufferlen;
    circular_buf_t          internal_buffer;  // TODO change to ring buffer
    uint8_t                 *ring_buffer_storage;  // allocated from MicroPython heap
    bool                    used;
    bool                    uasyncio_detected;
    bool                    non_blocking;
    non_blocking_info_t     non_blocking_info;

} machine_i2s_obj_t;

//  I2S implementation mapping:
//      I2S peripheral 1:  machine_i2s_obj[0]
//      I2S peripheral 2:  machine_i2s_obj[1]
STATIC machine_i2s_obj_t *machine_i2s_obj[2];

#if STATIC_DMA_BUFFER
STATIC uint8_t dma_buffer[2][SIZEOF_DMA_BUFFER_IN_BYTES];
#endif

// ring buffer implemention
// Source:  https://github.com/embeddedartistry/embedded-resources/tree/master/examples/c/circular_buffer
// License:  CC0 1.0 Universal:  https://github.com/embeddedartistry/embedded-resources/blob/master/LICENSE

void circular_buf_init(circular_buf_t *cbuf, uint8_t *buffer, size_t size);
void circular_buf_reset(circular_buf_t *cbuf);
void circular_buf_put(circular_buf_t *cbuf, uint8_t data);
int circular_buf_put2(circular_buf_t *cbuf, uint8_t data);
int circular_buf_get(circular_buf_t *cbuf, uint8_t *data);
int circular_buf_get16(circular_buf_t *cbuf, uint16_t *data);
int circular_buf_get32(circular_buf_t *cbuf, uint32_t *data);
bool circular_buf_empty(circular_buf_t *cbuf);
bool circular_buf_full(circular_buf_t *cbuf);
size_t circular_buf_capacity(circular_buf_t *cbuf);
size_t circular_buf_size(circular_buf_t *cbuf);

static void advance_pointer(circular_buf_t *cbuf)
{
    assert(cbuf);

    if(cbuf->full)
    {
        cbuf->tail = (cbuf->tail + 1) % cbuf->max;
    }

    cbuf->head = (cbuf->head + 1) % cbuf->max;

    // mark full because tail is advanced on the next time around
    cbuf->full = (cbuf->head == cbuf->tail);
}

static void retreat_pointer(circular_buf_t *cbuf)
{
    assert(cbuf);
    cbuf->full = false;
    cbuf->tail = (cbuf->tail + 1) % cbuf->max;
}

void circular_buf_init(circular_buf_t *cbuf, uint8_t* buffer, size_t size)
{
    assert(buffer && size); // TODO replace all asserts with MicroPython exceptions
    cbuf->buffer = buffer;
    cbuf->max = size;
    circular_buf_reset(cbuf);
    assert(circular_buf_empty(cbuf));
}

void circular_buf_reset(circular_buf_t *cbuf)
{
    assert(cbuf);
    cbuf->head = 0;
    cbuf->tail = 0;
    cbuf->full = false;
}

size_t circular_buf_size(circular_buf_t *cbuf)  // TODO improve function name
{
    assert(cbuf);
    size_t size = cbuf->max;
    if(!cbuf->full)
    {
        if(cbuf->head >= cbuf->tail)
        {
            size = (cbuf->head - cbuf->tail);
        }
        else
        {
            size = (cbuf->max + cbuf->head - cbuf->tail);
        }
    }

    return size;
}

size_t circular_buf_capacity(circular_buf_t *cbuf)
{
    assert(cbuf);
    return cbuf->max;
}

void circular_buf_put(circular_buf_t *cbuf, uint8_t data)
{
    assert(cbuf && cbuf->buffer);
    cbuf->buffer[cbuf->head] = data;
    advance_pointer(cbuf);
}

int circular_buf_put2(circular_buf_t *cbuf, uint8_t data)
{
    int r = -1;
    assert(cbuf && cbuf->buffer);
    if(!circular_buf_full(cbuf))
    {
        cbuf->buffer[cbuf->head] = data;
        advance_pointer(cbuf);
        r = 0;
    }

    return r;
}

int circular_buf_get(circular_buf_t *cbuf, uint8_t *data)
{
    assert(cbuf && data && cbuf->buffer);

    int r = -1;
    if(!circular_buf_empty(cbuf))
    {
        *data = cbuf->buffer[cbuf->tail];
        retreat_pointer(cbuf);
        r = 0;
    }
    return r;
}

// TODO eliminate this function
int circular_buf_get16(circular_buf_t *cbuf, uint16_t *data)
{
    int r = -1;
    uint8_t data8;
    r = circular_buf_get(cbuf, &data8);
    *data = data8;    
    r = circular_buf_get(cbuf, &data8);
    *data += data8 << 8;
    return r;
}

// TODO eliminate this function
int circular_buf_get32(circular_buf_t *cbuf, uint32_t *data)
{
    int r = -1;
    uint8_t data8;
    r = circular_buf_get(cbuf, &data8);
    *data = data8;
    r = circular_buf_get(cbuf, &data8);
    *data += data8 << 8;
    r = circular_buf_get(cbuf, &data8);
    *data += data8 << 16;
    r = circular_buf_get(cbuf, &data8);
    *data += data8 << 24;
    return r;
}

bool circular_buf_empty(circular_buf_t *cbuf)
{
    assert(cbuf);
    return (!cbuf->full && (cbuf->head == cbuf->tail));
}

bool circular_buf_full(circular_buf_t *cbuf)
{
    assert(cbuf);
    return cbuf->full;
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
STATIC void machine_i2s_reformat_32_bit_samples(int32_t *sample, uint32_t num_samples) {
    int16_t sample_ms;
    int16_t sample_ls;
    for (uint32_t i=0; i<num_samples; i++) {
        sample_ls = sample[i] & 0xFFFF;
        sample_ms = sample[i] >> 16;
        sample[i] = (sample_ls << 16) + sample_ms;
    }
}

#define NUM_FORMATS (4)  // TODO confusion with self->format ?
#define I2S_RX_FRAME_SIZE_IN_BYTES (8)

// TODO add 24-bits
STATIC const int8_t i2s_frame_overlay[NUM_FORMATS][I2S_RX_FRAME_SIZE_IN_BYTES] = {
    { 0,  1, -1, -1, -1, -1, -1, -1 },  // Mono, 16-bits
    { 2,  3,  0,  1, -1, -1, -1, -1 },  // Mono, 32-bits
    { 0,  1,  4,  5, -1, -1, -1, -1 },  // Stereo, 16-bits
    { 2,  3,  0,  1,  6,  7,  4,  5 },  // Stereo, 32-bits
};

STATIC int8_t get_overlay_index(int8_t bits, machine_i2s_format_t format) {
    if (format == MONO) {
        if (bits == I2S_DATAFORMAT_16B) {
            return 0;
        } else { // 32 bits
            return 1;
        }
    } else { // STEREO
        if (bits == I2S_DATAFORMAT_16B) {
            return 2;
        } else { // 32 bits
            return 3;
        }
    }
}

STATIC int8_t machine_i2s_get_stm_bits(uint16_t mode, int8_t bits) {
    if (mode == I2S_MODE_MASTER_TX) {
        return bits;
    } else { // Master Rx
        // always read 32 bit words for microphones
        return I2S_DATAFORMAT_32B;
    }
}

uint32_t the_one_readinto_function_to_rule_them_all(machine_i2s_obj_t *self, mp_buffer_info_t *bufinfo , bool asyncio) {
    // block until the entire sample buffer can be filled from the internal buffer. Space becomes
    // available in the internal buffer when sample data is copied from DMA.
    
    // TODO add "if no callback ..."  e.g. blocking readinto
    
    uint8_t target_frame_size_in_bytes;
    
    switch (self->bits) {
        case I2S_DATAFORMAT_16B:
            target_frame_size_in_bytes = 2;
            break;
        case I2S_DATAFORMAT_24B:
            target_frame_size_in_bytes = 3;
            break;
        case I2S_DATAFORMAT_32B:
        default:
            target_frame_size_in_bytes = 4;
            break;
    }
    
    if (self->format == STEREO) {
        target_frame_size_in_bytes *= 2;
    }
    
    //printf("target_frame_size_in_bytes: %d\n", target_frame_size_in_bytes);

    // TODO horrible naming
    uint32_t num_bytes_to_copy_from_ring_buffer_max = bufinfo->len * I2S_RX_FRAME_SIZE_IN_BYTES / target_frame_size_in_bytes;
    uint32_t num_bytes_to_copy_from_ring_buffer;
    
    // TODO consider using a State variable to determine mode.  e..g  asyncio, blocking, non_blocking
    if (!asyncio) {
        // TODO consider that some bytes in circular buf may be skipped over (so #bytes in circular
        // buf would be more than #bytes in target buffer)
        // TODO bug: if size provided buf > size circular bug the readinto() call hangs below
        // SOLN:  loop, reading from circular buf until provided buf is filled
        while (num_bytes_to_copy_from_ring_buffer_max > circular_buf_size(&self->internal_buffer)) {
            mp_hal_delay_us(1);  // this is needed to avoid the compiler optimizing out the buffer_space_available calc (?)
        }
        num_bytes_to_copy_from_ring_buffer = num_bytes_to_copy_from_ring_buffer_max;
        //printf("num_bytes_to_copy_from_ring_buffer: %ld\n", num_bytes_to_copy_from_ring_buffer);
    } else {  // asyncio
        num_bytes_to_copy_from_ring_buffer = MIN(num_bytes_to_copy_from_ring_buffer_max, circular_buf_size(&self->internal_buffer));
        //printf("num_bytes_to_copy_from_ring_buffer: %ld\n", num_bytes_to_copy_from_ring_buffer);
    }

    if (num_bytes_to_copy_from_ring_buffer) {
        uint8_t *target_p = bufinfo->buf;
        uint8_t overlay_index = get_overlay_index(self->bits, self->format);
        for (uint32_t i=0; i<num_bytes_to_copy_from_ring_buffer / I2S_RX_FRAME_SIZE_IN_BYTES; i++) {
            
            for (uint32_t j=0; j<I2S_RX_FRAME_SIZE_IN_BYTES; j++) {

                uint8_t sample;
                circular_buf_get(&self->internal_buffer, &sample);  // TODO eliminate this --> copy directly to &sample ?
                int8_t dma_mapping = i2s_frame_overlay[overlay_index][j];
                if (dma_mapping != -1) {
                    *(target_p + dma_mapping) = sample;
                }
            }
            target_p += target_frame_size_in_bytes;
        }
    }
    
    return num_bytes_to_copy_from_ring_buffer / I2S_RX_FRAME_SIZE_IN_BYTES * target_frame_size_in_bytes;
}    


uint32_t the_one_write_function_to_rule_them_all(machine_i2s_obj_t *self, mp_buffer_info_t *bufinfo, bool asyncio) {
    // block until the entire sample buffer can be copied to the internal buffer. Space becomes
    // available in the internal buffer when sample data is moved to DMA.
    
    uint32_t num_bytes_to_copy;
    //printf("bufinfo->len: %d\n", bufinfo->len);

    // TODO add function to circular buffer to get space available
    uint32_t buffer_space_available = circular_buf_capacity(&self->internal_buffer) - circular_buf_size(&self->internal_buffer);

    if (!asyncio) {
        while (bufinfo->len > buffer_space_available) {
            buffer_space_available = circular_buf_capacity(&self->internal_buffer) - circular_buf_size(&self->internal_buffer);
            mp_hal_delay_us(1);  // this is needed to avoid the compiler optimizing out the buffer_space_available calc (?)
        }
        num_bytes_to_copy = bufinfo->len;
    } else {  // asyncio

        //printf("TOWFTRTA buffer_space_available:  %ld\n", buffer_space_available);
        //printf("TOWFTRTA bufinfo->len:  %d\n", bufinfo->len);
        num_bytes_to_copy = MIN(buffer_space_available, bufinfo->len);
    }

    if (num_bytes_to_copy) {
        for (uint32_t i=0; i<num_bytes_to_copy; i++) {
            circular_buf_put(&self->internal_buffer, ((uint8_t *)bufinfo->buf)[i]);
        }
    }
    return num_bytes_to_copy;
}

// Simplying assumptions:
//   - size of sample buffers is an integer multiple of dma buffer size
//   -  TODO  note:  size of 1/2 dma buffer needs to be a multiple of 8 bytes  (so 1/2 of buffer always
//      contains integer number of complete stereo frames @ 32 bits/sample)
// note:  this is called by an ISR.  keep it as short as possible
STATIC void machine_i2s_empty_dma(machine_i2s_obj_t *self, machine_i2s_dma_buffer_ping_pong_t dma_ping_pong) {
    uint16_t dma_buffer_index = 0;
    if (dma_ping_pong == TOP_HALF) {
        dma_buffer_index = 0;  
    } else { // BOTTOM_HALF
        dma_buffer_index = SIZEOF_DMA_BUFFER_IN_BYTES/2; 
    }
    
    // when room exists, copy samples into circular buffer 
    uint32_t buffer_space_available = circular_buf_capacity(&self->internal_buffer) - circular_buf_size(&self->internal_buffer);
    //printf("buffer_space_available: %ld\n", buffer_space_available);
    
    if (buffer_space_available >= SIZEOF_DMA_BUFFER_IN_BYTES/2) {
        for (uint32_t i=0; i<SIZEOF_DMA_BUFFER_IN_BYTES/2; i++) {
            circular_buf_put(&self->internal_buffer, self->dma_buffer[dma_buffer_index++]);
        }
    } else {
        //printf("no space\n");
    }
}

// Simplying assumptions:
//   - size of sample buffers is an integer multiple of dma buffer size
//   -  TODO  note:  size of 1/2 dma buffer needs to be a multiple of 8 bytes  (so 1/2 of buffer always
//      contains integer number of complete stereo frames @ 32 bits/sample)
// note:  this is called by an ISR.  keep it as short as possible
// TODO  -- move all the time intensive copies into the write function.  just copy bytes, that's all
STATIC void machine_i2s_feed_dma(machine_i2s_obj_t *self, machine_i2s_dma_buffer_ping_pong_t dma_ping_pong) {
    uint16_t dma_buffer_index = 0;
    if (dma_ping_pong == TOP_HALF) {
        dma_buffer_index = 0;  
    } else { // BOTTOM_HALF
        dma_buffer_index = SIZEOF_DMA_BUFFER_IN_BYTES/2; 
    }
    
    // TODO check that internal buffer has enough samples before doing the copy
    
    // copy a block of samples from the internal buffer to the dma buffer.
    // mono format is implemented by duplicating each sample into both L and R channels.
    // (STM32 HAL API has a stereo I2S implementation, but not mono)
    
    if ((self->format == MONO) && (self->bits == I2S_DATAFORMAT_16B)) {
        //printf("MONO 16B\n");
        uint32_t samples_to_copy = SIZEOF_DMA_BUFFER_IN_BYTES/2/4; // TODO - really confusing -- fix
        uint16_t *dma_buffer_p = (uint16_t *)&self->dma_buffer[dma_buffer_index];
        for (uint32_t i=0; i<samples_to_copy; i++) {
            uint16_t sample;
            circular_buf_get16(&self->internal_buffer, &sample);  // TODO check ret code 
            dma_buffer_p[i*2] = sample; 
            dma_buffer_p[i*2+1] = sample; 
        }
    } else if ((self->format == MONO) && (self->bits == I2S_DATAFORMAT_32B)) {
        uint32_t samples_to_copy = SIZEOF_DMA_BUFFER_IN_BYTES/2/8;  // TODO - really confusing -- fix
        uint32_t *dma_buffer_p = (uint32_t *)&self->dma_buffer[dma_buffer_index];
        for (uint32_t i=0; i<samples_to_copy; i++) {
            uint32_t sample;
            circular_buf_get32(&self->internal_buffer, &sample);  // TODO check ret code
            dma_buffer_p[i*2] = sample;
            dma_buffer_p[i*2+1] = sample;
        }
    } else { // STEREO, both 16-bit and 32-bit 
        uint32_t samples_to_copy = SIZEOF_DMA_BUFFER_IN_BYTES/2/4;  // ... TODO:  isn't 16 bit twice this number?
        uint32_t *dma_buffer_p = (uint32_t *)&self->dma_buffer[dma_buffer_index];
        for (uint32_t i=0; i<samples_to_copy; i++) {
            uint32_t sample=0;
            circular_buf_get32(&self->internal_buffer, &sample);  // TODO check ret code
            dma_buffer_p[i] = sample;
        }
    }
    
    // 32 bit samples need to be reformatted to match STM32 HAL API requirements for I2S
    // TODO  SIZEOF_DMA_BUFFER_IN_BYTES/2  <--- make a macro for this
    if (self->bits == I2S_DATAFORMAT_32B) {
        machine_i2s_reformat_32_bit_samples((int32_t *)&self->dma_buffer[dma_buffer_index], SIZEOF_DMA_BUFFER_IN_BYTES/2/4);
    }
}

// TODO assumption that supplied buffer is a multiple of 8 bytes.. check this somewhere
void non_blocking_copy_to_buffer (machine_i2s_obj_t *self) {
    //printf("non_blocking_copy_to_buffer\n");

    // attempt to copy a block of audio samples from the ring buffer to the supplied buffer.
    // audio samples will be formatted as part of the copy operation

    // determine the audio sample frame size for the destination buffer
    uint8_t target_frame_size_in_bytes;

    switch (self->bits) {
        case I2S_DATAFORMAT_16B:
            target_frame_size_in_bytes = 2;
            break;
        case I2S_DATAFORMAT_24B:
            target_frame_size_in_bytes = 3;
            break;
        case I2S_DATAFORMAT_32B:
        default:
            target_frame_size_in_bytes = 4;
            break;
    }

    if (self->format == STEREO) {
        target_frame_size_in_bytes *= 2;
    }

    uint32_t num_bytes_remaining_to_copy_to_buffer = self->non_blocking_info.bufinfo.len - self->non_blocking_info.bufindex;
    //printf("num_bytes_remaining_to_copy_to_buffer: %ld\n", num_bytes_remaining_to_copy_to_buffer);


    uint32_t num_bytes_remaining_to_copy_from_ring_buffer = num_bytes_remaining_to_copy_to_buffer *
                                                  (I2S_RX_FRAME_SIZE_IN_BYTES / target_frame_size_in_bytes);
    //printf("num_bytes_remaining_to_copy_from_ring_buffer: %ld\n", num_bytes_remaining_to_copy_from_ring_buffer);


    uint32_t num_bytes_to_copy_from_ring_buffer = MIN(SIZEOF_NON_BLOCKING_COPY_IN_BYTES, num_bytes_remaining_to_copy_from_ring_buffer);
    //printf("num_bytes_to_copy_from_ring_buffer:  %ld\n", num_bytes_to_copy_from_ring_buffer);

    //printf("circular_buf_size(&self->internal_buffer):  %d\n", circular_buf_size(&self->internal_buffer));

    if (circular_buf_size(&self->internal_buffer) >= num_bytes_to_copy_from_ring_buffer) {
        //printf("Copy 'em\n");

        uint8_t *target_p = (uint8_t *)self->non_blocking_info.bufinfo.buf + self->non_blocking_info.bufindex;
        uint8_t overlay_index = get_overlay_index(self->bits, self->format);
        for (uint32_t i=0; i<num_bytes_to_copy_from_ring_buffer / I2S_RX_FRAME_SIZE_IN_BYTES; i++) {

            for (uint32_t j=0; j<I2S_RX_FRAME_SIZE_IN_BYTES; j++) {

                uint8_t sample;
                circular_buf_get(&self->internal_buffer, &sample);  // TODO eliminate this copy to &sample ?
                int8_t dma_mapping = i2s_frame_overlay[overlay_index][j];
                if (dma_mapping != -1) {
                    *(target_p + dma_mapping) = sample;
                }
            }
            target_p += target_frame_size_in_bytes;
        }
        self->non_blocking_info.bufindex += num_bytes_to_copy_from_ring_buffer /
                        (I2S_RX_FRAME_SIZE_IN_BYTES / target_frame_size_in_bytes);  // TODO confusing


        if (self->non_blocking_info.bufindex >= self->non_blocking_info.bufinfo.len) {
            //printf("===========  Done copy to supplied buffer ======= \n");
            self->non_blocking = false;
            mp_sched_schedule(self->callback, self);
        }
    }
}

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

    // GPIO Pin initialization  // TODO refactor to call a single function for each pin
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

    // TODO: support more of the commonly-used frequencies and account for 16/24/32 bit frames
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
#error I2S does not supported on this processor
#endif // STM32F405xx

    if (HAL_I2S_Init(&i2s_obj->i2s) == HAL_OK) {
        // Reset and initialize Tx and Rx DMA channels
        if (i2s_obj->mode == I2S_MODE_MASTER_RX) {
            // Reset and initialize RX DMA
            dma_invalidate_channel(i2s_obj->rx_dma_descr);
            dma_init(&i2s_obj->rx_dma, i2s_obj->rx_dma_descr, DMA_PERIPH_TO_MEMORY, &i2s_obj->i2s);
            i2s_obj->i2s.hdmarx = &i2s_obj->rx_dma;
        } else {  // I2S_MODE_MASTER_TX
            // Reset and initialize TX DMA
            dma_invalidate_channel(i2s_obj->tx_dma_descr);
            dma_init(&i2s_obj->tx_dma, i2s_obj->tx_dma_descr, DMA_MEMORY_TO_PERIPH, &i2s_obj->i2s);
            i2s_obj->i2s.hdmatx = &i2s_obj->tx_dma;
        }
        return true;
    } else {
        return false;
    }
}
    
void HAL_I2S_ErrorCallback(I2S_HandleTypeDef *hi2s) {
    uint32_t errorCode = HAL_I2S_GetError(hi2s);
    printf("I2S Error = %ld\n", errorCode);
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s) {

    // TODO refactor 1/2 and complete callback functions to call a single function
    machine_i2s_obj_t *self;
    if (hi2s->Instance == I2S1) {
        self = machine_i2s_obj[0];
        //printf("RxCplt obj 0:  self 0x%lx\n", (uint32_t)self);
    } else {
        self = machine_i2s_obj[1];
        //printf("RxCplt obj 1:  self 0x%lx\n", (uint32_t)self);
    }
    
    // bottom half of buffer now filled, 
    // safe to empty the bottom half while the top half of buffer is being filled
    machine_i2s_empty_dma(self, BOTTOM_HALF);  // TODO check with =S= vs uPy coding rules.  is machine_i2s prefix really needed for STATIC?

    // for non-blocking operation, this IRQ-based callback needs to service
    // and read() method requests for copying audio samples from the internal ring buffer
    // to the buffer supplied in the read() method call
    if (self->non_blocking) {
        non_blocking_copy_to_buffer(self);
    }
}     
    
void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
    machine_i2s_obj_t *self;
    if (hi2s->Instance == I2S1) {
        self = machine_i2s_obj[0];
        //printf("RxHalfCplt obj 0:  self 0x%lx\n", (uint32_t)self);
    } else {
        self = machine_i2s_obj[1];
        //printf("RxHalfCplt obj 1:  self 0x%lx\n", (uint32_t)self);
    }
    
    // top half of buffer now filled,
    // safe to empty the top half while the bottom half of buffer is being filled
    machine_i2s_empty_dma(self, TOP_HALF);  // TODO check with =S= vs uPy coding rules.  is machine_i2s prefix really needed for STATIC?

    // for non-blocking operation, this IRQ-based callback needs to service
    // and read() method requests for copying audio samples from the internal ring buffer
    // to the buffer supplied in the read() method call
    if (self->non_blocking) {
        non_blocking_copy_to_buffer(self);
    }
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s) {
    machine_i2s_obj_t *self;
    
    if (hi2s->Instance == I2S1) {
        self = machine_i2s_obj[0];
    } else {
        self = machine_i2s_obj[1];
    }
    
    //uint32_t ticks_now_us = mp_hal_ticks_us();
    //uint32_t delta = ticks_now_us - self->junk_complete_prev;
    //if (delta > 2010) printf("C delta [us]:  %ld\n", delta);
    //if (delta < 1990) printf("C delta [us]:  %ld\n", delta);
    //printf("C delta [us]:  %ld\n", delta);

    // TODO refactor in both Tx callbacks, single function
    if (self->non_blocking) {
        // copy audio samples from supplied buffer into internal buffer
        // TODO add function to circular buffer to get space available
        // TODO buffer_space_available is a bad variable name as it's related to circular buffer
        uint32_t buffer_space_available = circular_buf_capacity(&self->internal_buffer) - circular_buf_size(&self->internal_buffer);
        //printf("buffer_space_available: %ld\n", buffer_space_available);

        uint32_t num_bytes_remaining_to_copy = self->non_blocking_info.bufinfo.len - self->non_blocking_info.bufindex;
        uint32_t num_bytes_to_copy = MIN(SIZEOF_NON_BLOCKING_COPY_IN_BYTES, num_bytes_remaining_to_copy);

        //printf("num_bytes_to_copy:  %ld\n", num_bytes_to_copy);

        if (buffer_space_available >= num_bytes_to_copy) {
            //printf("copying to circular buffer\n");
            // enough space available in internal buffer - do the copy

            //uint32_t t0 = mp_hal_ticks_us();

            for (uint32_t i=0; i<num_bytes_to_copy; i++) {
                circular_buf_put(&self->internal_buffer,
                                 ((uint8_t *)self->non_blocking_info.bufinfo.buf)[self->non_blocking_info.bufindex + i]);
            }

            self->non_blocking_info.bufindex += num_bytes_to_copy;
            //uint32_t t1 = mp_hal_ticks_us();

            //printf("cp time [us]: %ld\n", t1-t0);

            if (self->non_blocking_info.bufindex >= self->non_blocking_info.bufinfo.len) {
                //printf("uPy CB\n");
                self->non_blocking = false;
                mp_sched_schedule(self->callback, self);
            }
        }
    }

    // bottom half of buffer now emptied, 
    // safe to fill the bottom half while the top half of buffer is being emptied
    machine_i2s_feed_dma(self, BOTTOM_HALF);  // TODO check with =S= vs uPy coding rules.  is machine_i2s prefix really needed for STATIC?

    //self->junk_complete_prev = ticks_now_us;
}

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
    machine_i2s_obj_t *self;
    if (hi2s->Instance == I2S1) {
        self = machine_i2s_obj[0];
    } else {
        self = machine_i2s_obj[1];
    }
    
    //uint32_t ticks_now_us = mp_hal_ticks_us();
    //uint32_t delta = ticks_now_us - self->junk_half_prev;
    //if (delta > 2010) printf("H delta [us]:  %ld\n", delta);
    //if (delta < 1990) printf("H delta [us]:  %ld\n", delta);
    //printf("H delta [us]:  %ld\n", delta);

    if (self->non_blocking) {
        // copy audio samples from supplied buffer into internal buffer
        // TODO add function to circular buffer to get space available
        uint32_t buffer_space_available = circular_buf_capacity(&self->internal_buffer) - circular_buf_size(&self->internal_buffer);
        //printf("buffer_space_available: %ld\n", buffer_space_available);

        uint32_t num_bytes_remaining_to_copy = self->non_blocking_info.bufinfo.len - self->non_blocking_info.bufindex;
        uint32_t num_bytes_to_copy = MIN(SIZEOF_NON_BLOCKING_COPY_IN_BYTES, num_bytes_remaining_to_copy);

        //printf("num_bytes_to_copy:  %ld\n", num_bytes_to_copy);

        if (buffer_space_available >= num_bytes_to_copy) {
            //printf("copying to circular buffer\n");
            // enough space available in internal buffer - do the copy

            //uint32_t t0 = mp_hal_ticks_us();

            for (uint32_t i=0; i<num_bytes_to_copy; i++) {
                circular_buf_put(&self->internal_buffer,
                                 ((uint8_t *)self->non_blocking_info.bufinfo.buf)[self->non_blocking_info.bufindex + i]);
            }

            self->non_blocking_info.bufindex += num_bytes_to_copy;
            //uint32_t t1 = mp_hal_ticks_us();

            //printf("cp time [us]: %ld\n", t1-t0);

            if (self->non_blocking_info.bufindex >= self->non_blocking_info.bufinfo.len) {
                //printf("uPy CB\n");
                self->non_blocking = false;
                mp_sched_schedule(self->callback, self);
            }
        }
    }

    // top half of buffer now emptied, 
    // safe to fill the top half while the bottom half of buffer is being emptied
    machine_i2s_feed_dma(self, TOP_HALF);  // TODO check with =S= vs uPy coding rules.  is machine_i2s prefix really needed for STATIC?

    //self->junk_half_prev = ticks_now_us;
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
        ARG_bufferlen,
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
    
    memset(&self->i2s, 0, sizeof(self->i2s));
    
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
    // TODO add 24 bit support -- diehard audio folks will expect to see this
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
    // No validation done:  TODO can it be validated?  multiple of 8 bytes?
    // TODO  round stack size down to a multiple of the I2S frame size
    // allocate an internal sample buffer
    int32_t internal_buffer_len = args[ARG_bufferlen].u_int;
    if (internal_buffer_len > 0) {
        uint8_t *buffer = m_new(uint8_t, internal_buffer_len);
        self->ring_buffer_storage = buffer;
        circular_buf_init(&self->internal_buffer, buffer, internal_buffer_len);
    } else {
        mp_raise_ValueError(MP_ERROR_TEXT("Buffer length is not valid"));
    }
    
    self->sck = args[ARG_sck].u_obj;
    self->ws = args[ARG_ws].u_obj;
    self->sd = args[ARG_sd].u_obj;
    self->mode = i2s_mode;
    self->bits = i2s_bits_per_sample;
    self->format = i2s_format;
    self->rate = args[ARG_rate].u_int;
    self->callback = MP_OBJ_NULL;
    self->uasyncio_detected = false;
    self->non_blocking = false;
    
    I2S_InitTypeDef *init = &self->i2s.Init;
    init->Mode = i2s_mode;
    init->Standard   = I2S_STANDARD_PHILIPS;
    init->DataFormat = machine_i2s_get_stm_bits(self->mode, self->bits);
    init->MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
    init->AudioFreq  = args[ARG_rate].u_int;
    init->CPOL       = I2S_CPOL_LOW;
    init->ClockSource = I2S_CLOCK_PLL; 
    
    // init the I2S bus
    if (!i2s_init(self)) {
        mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("I2S port %d init failed"), self->i2s_id);
    }
    
    self->used = true;
    
    // start DMA.  DMA is configured to run continuously, using a circular buffer configuration
    // see "How it works" at the top of file
    uint16_t number_of_samples = 0;
    if (init->DataFormat == I2S_DATAFORMAT_16B) {
        number_of_samples = SIZEOF_DMA_BUFFER_IN_BYTES / 2;
    } else {  // 32 bits
        number_of_samples = SIZEOF_DMA_BUFFER_IN_BYTES / 4;
    }
    HAL_StatusTypeDef status;

    if (self->mode == I2S_MODE_MASTER_TX) {
        status = HAL_I2S_Transmit_DMA(&self->i2s, (void *)self->dma_buffer, number_of_samples);
    } else {  // RX
        status = HAL_I2S_Receive_DMA(&self->i2s, (void *)self->dma_buffer, number_of_samples);
    }

    if (status != HAL_OK) {
        mp_hal_raise(status);
    }
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
    else if (self->bits == I2S_DATAFORMAT_24B) { bits = 24; }
    else if (self->bits == I2S_DATAFORMAT_32B) { bits = 32; }
    else { /* shouldn't get here */ }
    
    // TODO update with all arguments
    
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

STATIC mp_obj_t machine_i2s_make_new(const mp_obj_type_t *type, size_t n_pos_args, size_t n_kw_args, const mp_obj_t *args) {
    mp_arg_check_num(n_pos_args, n_kw_args, 1, MP_OBJ_FUN_ARGS_MAX, true);
    uint8_t i2s_id = mp_obj_get_int(args[0]);
    machine_i2s_obj_t *self = m_new_obj(machine_i2s_obj_t);

    if(0) {
#if MICROPY_HW_ENABLE_I2S1
    } else if (i2s_id == 1) {
        machine_i2s_obj[0] = self;
#endif
#if MICROPY_HW_ENABLE_I2S2
    } else if (i2s_id == 2) {
        machine_i2s_obj[1] = self;
#endif
    } else {
        mp_raise_ValueError(MP_ERROR_TEXT("I2S ID is not valid"));
    }
    self->base.type = &machine_i2s_type;
    self->i2s_id = i2s_id;

    // is I2S peripheral already in use?
    if (self->used) {
        mp_raise_ValueError(MP_ERROR_TEXT("I2S port is already in use"));
    }
    
#if STATIC_DMA_BUFFER
    self->dma_buffer = &(dma_buffer[i2s_id - 1][0]);
#endif

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

STATIC mp_uint_t machine_i2s_stream_read(mp_obj_t self_in, void *buf_in, mp_uint_t size, int *errcode) {
    machine_i2s_obj_t *self = MP_OBJ_TO_PTR(self_in);
    
    // make sure we want at least 1 char
    if (size == 0) {
        return 0;
    }

    if (!self->used) {
        *errcode = MP_EPERM;
    }

    if (self->mode != I2S_MODE_MASTER_RX) {
        *errcode = MP_EPERM;
    }

    // TODO consider more intuitive flow
    //  if non-blocking
    //  else if blocking
    //  else if asyncio
    //  else error

    if (self->callback != MP_OBJ_NULL) {
        // callback set indicates non-blocking read
        //printf("=========== non-blocking read ==========\n");
        self->non_blocking_info.bufinfo.buf = (void *)buf_in;
        self->non_blocking_info.bufinfo.len = size;
        self->non_blocking_info.bufindex = 0;
        self->non_blocking = true;
        return size;
    } else {
        mp_buffer_info_t bufinfo;
        bufinfo.buf = (void *)buf_in;
        bufinfo.len = size;
        uint32_t num_bytes_read;
        if (self->uasyncio_detected == true) {  // TODO refactor ... just the true/false are different
            num_bytes_read = the_one_readinto_function_to_rule_them_all(self, &bufinfo, true);
            //printf("async, num_bytes_read: %ld\n", num_bytes_read);
        } else {
            num_bytes_read = the_one_readinto_function_to_rule_them_all(self, &bufinfo, false);
            //printf("non-async, num_bytes_read: %ld\n", num_bytes_read);
        }
        return num_bytes_read;
    }
}

STATIC mp_uint_t machine_i2s_stream_write(mp_obj_t self_in, const void *buf_in, mp_uint_t size, int *errcode) {
    machine_i2s_obj_t *self = MP_OBJ_TO_PTR(self_in);

    if (!self->used) {
        *errcode = MP_EPERM;
    }

    if (self->mode != I2S_MODE_MASTER_TX) {
        *errcode = MP_EPERM;
    }

    // TODO consider more intuitive flow
    //  if non-blocking
    //  else if blocking
    //  else if asyncio
    //  else error

    if (self->callback != MP_OBJ_NULL) {
        // callback set indicates non-blocking write
        //printf("=========== non-blocking write ==========\n");
        self->non_blocking_info.bufinfo.buf = (void *)buf_in;
        self->non_blocking_info.bufinfo.len = size;
        self->non_blocking_info.bufindex = 0;
        self->non_blocking = true;
        return size;
    } else {
        mp_buffer_info_t bufinfo;
        bufinfo.buf = (void *)buf_in;
        bufinfo.len = size;
        uint32_t num_bytes_written;
        if (self->uasyncio_detected == true) {  // TODO refactor ... just the true/false are different
            num_bytes_written = the_one_write_function_to_rule_them_all(self, &bufinfo, true);
            //printf("async, num_bytes_written: %ld\n", num_bytes_written);
        } else {
            num_bytes_written = the_one_write_function_to_rule_them_all(self, &bufinfo, false);
            //printf("non-async, num_bytes_written: %d\n", num_bytes_written);
        }
        return num_bytes_written;
    }
}


STATIC mp_obj_t machine_i2s_deinit(mp_obj_t self_in) {
    
    machine_i2s_obj_t *self = self_in;
    
    dma_deinit(self->tx_dma_descr);
    dma_deinit(self->rx_dma_descr);
    HAL_I2S_DeInit(&self->i2s);
    
    if (self->i2s.Instance == I2S1) {
        __SPI1_FORCE_RESET();
        __SPI1_RELEASE_RESET();
        __SPI1_CLK_DISABLE();
    } else if (self->i2s.Instance == I2S2) {
        __SPI2_FORCE_RESET();
        __SPI2_RELEASE_RESET();
        __SPI2_CLK_DISABLE();
    }
    
    if (self->ring_buffer_storage) {
        m_free(self->ring_buffer_storage);
    }
    
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(machine_i2s_deinit_obj, machine_i2s_deinit);

STATIC mp_obj_t machine_i2s_callback(mp_obj_t self_in, mp_obj_t handler) {
    machine_i2s_obj_t *self = self_in;
    if (handler != mp_const_none && !mp_obj_is_callable(handler)) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid handler"));
    }

    self->callback = handler;

    // TODO add ability to disable non-blocking operation 
    // self->client_task_handle = NULL;

    // TODO set some sort of flag to indicate NON-BLOCKING operation

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
    { MP_ROM_QSTR(MP_QSTR_readinto),        MP_ROM_PTR(&mp_stream_readinto_obj) },
    { MP_ROM_QSTR(MP_QSTR_write),           MP_ROM_PTR(&mp_stream_write_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinit),          MP_ROM_PTR(&machine_i2s_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR_callback),        MP_ROM_PTR(&machine_i2s_callback_obj) },
    
#if MEASURE_COPY_PERFORMANCE  // temporary - to be removed before mainline commit
    { MP_ROM_QSTR(MP_QSTR_copytest),       MP_ROM_PTR(&machine_i2s_copytest_obj) },
#endif     
    
    // Constants
    { MP_ROM_QSTR(MP_QSTR_RX),              MP_ROM_INT(I2S_MODE_MASTER_RX) },
    { MP_ROM_QSTR(MP_QSTR_TX),              MP_ROM_INT(I2S_MODE_MASTER_TX) },
    { MP_ROM_QSTR(MP_QSTR_STEREO),          MP_ROM_INT(STEREO) },
    { MP_ROM_QSTR(MP_QSTR_MONO),            MP_ROM_INT(MONO) },
};
MP_DEFINE_CONST_DICT(machine_i2s_locals_dict, machine_i2s_locals_dict_table);

STATIC mp_uint_t machine_i2s_ioctl(mp_obj_t self_in, mp_uint_t request, mp_uint_t arg, int *errcode) {
    machine_i2s_obj_t *self = self_in;
    mp_uint_t ret;
    mp_uint_t flags = arg;
    //printf("ioctl() poll\n");
    // set a flag indicating that I2S is being used in uasyncio mode
    // note:  assumption that an IO poll from the uasyncio scheduler is the only means
    // to get here
    self->uasyncio_detected = true;  // TODO find a better mechanism to detect asyncio

    //printf("ioctl delta last run [us]:  %ld\n", ticks_now_us - self->junk_t0);

    //printf("ioctl\n");

    if (request == MP_STREAM_POLL) {
        ret = 0;

        if (flags & MP_STREAM_POLL_RD) {
            // check ring buffer to determine if there is any sample data to read
            if (circular_buf_size(&self->internal_buffer) > 5000) {
                //printf("ioctl RD circular_buf_size(&self->internal_buffer): %d\n", circular_buf_size(&self->internal_buffer));
                ret |= MP_STREAM_POLL_RD;
            }
        }

        if (flags & MP_STREAM_POLL_WR) {
            // check ring buffer to determine if there is any space to write sample data
            int32_t buffer_space_available = circular_buf_capacity(&self->internal_buffer) - circular_buf_size(&self->internal_buffer);
            //printf("ioctl buffer_space_available: %ld\n", buffer_space_available);
            //if(!circular_buf_full(&self->internal_buffer)) {
            //printf("ioctl WR buffer_space_available: %ld\n", buffer_space_available);
            if (buffer_space_available) {
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
    .locals_dict = (mp_obj_dict_t *) &machine_i2s_locals_dict,
};
