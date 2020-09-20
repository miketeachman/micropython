/*
 * This file is part of the MicroPython ESP32 project
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

/*
 * Module:  i2stools
 *          
 * Purpose: routines to support the I2S feature
 * 
 * Functions:
 * 
 *      copy()  
 * 
 *          Processes a buffer of audio samples coming from an I2S Microphone:
 *          - sequentially extracts each 8 byte stereo frame
 *          - extracts either the Left, Right, or both channel(s) of sample data from the frame
 *          - optionally converts the 4 byte sample data to 2 byte sample data (taking the MS two bytes)
 *          - copies the samples to an output buffer
 *          - returns number of bytes written to the output buffer
 *           
 *          Performance Tests:  256 stereo frames (2048 bytes) processed in 125uS.  ESP32 @ 240MHz.
 *                              Equivalent MicroPython code runs in approximately 3500us
 *                              Using this compiled C function is about 28x faster
 *           
 *          Typical use cases.  Create a buffer of audio sample data to be:
 *              1. written to a file on a SD Card
 *              2. used in a decibel calculation routine. e.g. to calculate dB(A)
 *           
 *          MicroPython Example:
 *              import i2stools
 *              SAMPLE_BUFFER_SIZE = 2048
 *              raw_samples = bytearray(SAMPLE_BUFFER_SIZE)  # bytearray holding 32-bit audio data, both L+R channels
 *              copied_samples = bytearray(SAMPLE_BUFFER_SIZE // 4) # bytearray to hold 16-bit audio data, one channel
 *              i2stools.copy(bufin=raw_samples, bufout=copied_samples, channel=i2stools.LEFT, format=i2stools.B16)
 *              ... do something with copied_samples ...
 *              
 *      shift()
 *
 *          Performs an arithmetic shift of each audio sample in a buffer
 *          - a single bit shift changes the gain by 6dB.  e.g.  1-bit left shift increases gain by 6dB
 *          - a negative shift value is a right shift.  e.g. shift=-2 shifts right by 2-bits
 *
 *          Typical use cases.
 *              1. volume control.  Left shift = increase volume.  Right shift = reduce volume
 *
 *          MicroPython Example:  right shift 16-bit audio samples by 1-bit
 *              import i2stools
 *              SAMPLE_BUFFER_SIZE = 2048
 *              raw_samples = bytearray(SAMPLE_BUFFER_SIZE)  # bytearray holding 16-bit audio data
 *              shifted_samples = bytearray(SAMPLE_BUFFER_SIZE) # bytearray to hold 16-bit audio data
 *              i2stools.shift(bufin=raw_samples, bufout=shifted_samples, shift=-1, format=i2stools.B16)
 *              ... do something with shifted_samples ...
 *
 *          Caution:  arguments have no validity checks
 */

#include "py/nlr.h"
#include "py/obj.h"
#include "py/runtime.h"
#include "py/binary.h"

#define NUM_BYTES_IN_SAMPLE (4)
//#define NUM_BYTES_IN_STEREO_FRAME (NUM_BYTES_IN_SAMPLE * 2)

typedef enum {
    CHANNEL_LEFT = 0,
    CHANNEL_RIGHT = 1,
    CHANNEL_LEFT_RIGHT = 2,
} channel_t;

typedef enum {
    FORMAT_16_BIT,
    FORMAT_32_BIT,
} format_t;

STATIC mp_obj_t i2stools_copy(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_bufin, ARG_bufout, ARG_channel, ARG_format};
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_bufin,    MP_ARG_REQUIRED | MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_bufout,   MP_ARG_REQUIRED | MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_channel,                    MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = CHANNEL_LEFT} },
        { MP_QSTR_format,                     MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = FORMAT_16_BIT} },
    };

    // parse args
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    mp_buffer_info_t bufinfo_in;
    mp_get_buffer_raise(args[ARG_bufin].u_obj, &bufinfo_in, MP_BUFFER_READ);
    int32_t *buf_in = bufinfo_in.buf;

    mp_buffer_info_t bufinfo_out;
    mp_get_buffer_raise(args[ARG_bufout].u_obj, &bufinfo_out, MP_BUFFER_WRITE);
    int16_t *buf_out_16 = bufinfo_out.buf;
    int32_t *buf_out_32 = bufinfo_out.buf;
    
    channel_t channel = args[ARG_channel].u_int;
    format_t format = args[ARG_format].u_int;
    
    uint16_t skip = 0;
    uint16_t offset = 0;
    switch (channel) {
        case CHANNEL_LEFT:
            skip = 2;
            offset = 0;
            break;
        case CHANNEL_RIGHT:
            skip = 2;
            offset = 1;
            break;
        default:
        case CHANNEL_LEFT_RIGHT:
            skip = 1;
            offset = 0;
            break;
    }
    
    // note:  a "stereo frame" consists of a Left channel sample (4 bytes)
    // followed by a Right channel sample (4 bytes)
    uint32_t num_samples = bufinfo_in.len / NUM_BYTES_IN_SAMPLE;
    uint32_t num_bytes_copied = 0;
    uint32_t buf_in_index = 0;
    uint32_t buf_out_index = 0;    
    while (num_samples > buf_in_index) {
        // copy the specified channel sample from the input buffer
        int32_t sample = buf_in[buf_in_index + offset];

        switch (format) {
            case FORMAT_16_BIT:
                sample = sample >> 16;
                buf_out_16[buf_out_index] = (int16_t)sample;
                num_bytes_copied += sizeof(int16_t);
                break;
            case FORMAT_32_BIT:
            default:
                buf_out_32[buf_out_index] = sample;
                num_bytes_copied += sizeof(int32_t);
                break;
        }
        buf_in_index += skip;
        buf_out_index++;
    }
    return mp_obj_new_int(num_bytes_copied);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(i2stools_copy_obj, 0, i2stools_copy);

STATIC mp_obj_t i2stools_shift(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_bufin, ARG_bufout, ARG_shift, ARG_format};
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_bufin,    MP_ARG_REQUIRED | MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_bufout,   MP_ARG_REQUIRED | MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_shift ,                     MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_format,                     MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = FORMAT_16_BIT} },
    };

    // parse args
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    mp_buffer_info_t bufinfo_in;
    mp_get_buffer_raise(args[ARG_bufin].u_obj, &bufinfo_in, MP_BUFFER_READ);
    int16_t *buf_in_16 = bufinfo_in.buf;
    int32_t *buf_in_32 = bufinfo_in.buf;

    mp_buffer_info_t bufinfo_out;
    mp_get_buffer_raise(args[ARG_bufout].u_obj, &bufinfo_out, MP_BUFFER_WRITE);
    int16_t *buf_out_16 = bufinfo_out.buf;
    int32_t *buf_out_32 = bufinfo_out.buf;

    int8_t shift = args[ARG_shift].u_int;
    format_t format = args[ARG_format].u_int;

    uint32_t num_audio_samples;
    switch (format) {
        case FORMAT_16_BIT:
            num_audio_samples = bufinfo_in.len / 2;
            break;
        case FORMAT_32_BIT:
        default:
            num_audio_samples = bufinfo_in.len / 4;
            break;
    }

    for (uint32_t i = 0; i < num_audio_samples; i++) {
        switch (format) {
            case FORMAT_16_BIT:
                if (shift >= 0) {
                    buf_out_16[i] = buf_in_16[i] << shift;
                } else {
                    buf_out_16[i] = buf_in_16[i] >> abs(shift);
                }
                break;
            case FORMAT_32_BIT:
            default:
                if (shift >= 0) {
                    buf_out_32[i] = buf_in_32[i] << shift;
                } else {
                    buf_out_32[i] = buf_in_32[i] >> abs(shift);
                }
                break;
        }
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(i2stools_shift_obj, 0, i2stools_shift);

STATIC const mp_rom_map_elem_t i2stools_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),    MP_ROM_QSTR(MP_QSTR_i2stools) },
    { MP_ROM_QSTR(MP_QSTR_copy),        MP_ROM_PTR(&i2stools_copy_obj) },
    { MP_ROM_QSTR(MP_QSTR_shift),       MP_ROM_PTR(&i2stools_shift_obj) },
    { MP_ROM_QSTR(MP_QSTR_LEFT),        MP_ROM_INT(CHANNEL_LEFT) },
    { MP_ROM_QSTR(MP_QSTR_RIGHT),       MP_ROM_INT(CHANNEL_RIGHT) },
    { MP_ROM_QSTR(MP_QSTR_LEFT_RIGHT),  MP_ROM_INT(CHANNEL_LEFT_RIGHT) },
    { MP_ROM_QSTR(MP_QSTR_B16),         MP_ROM_INT(FORMAT_16_BIT) },
    { MP_ROM_QSTR(MP_QSTR_B32),         MP_ROM_INT(FORMAT_32_BIT) },
};

STATIC MP_DEFINE_CONST_DICT(mp_module_i2stools_globals, i2stools_globals_table);

const mp_obj_module_t mp_module_i2stools = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&mp_module_i2stools_globals,
};
