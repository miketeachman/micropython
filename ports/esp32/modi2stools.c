/*
 * This file is part of the MicroPython ESP32 project
 * 
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Mike Teachman
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
 *          - extracts either the Left or Right channel sample data (4 bytes) from the frame
 *          - optionally converts the 4 byte sample to a 2 byte sample (taking the MS two bytes)
 *          - copies the sample to an output buffer
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
 *          Caution:  arguments have no validity checks
 */

#include "py/nlr.h"
#include "py/obj.h"
#include "py/runtime.h"
#include "py/binary.h"

#define NUM_BYTES_IN_STEREO_FRAME (8)

typedef enum {
    CHANNEL_LEFT = 0,
    CHANNEL_RIGHT = 1,
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
    int32_t *buf_out_32 = bufinfo_out.buf;
    int16_t *buf_out_16 = bufinfo_out.buf; 
    
    channel_t channel = args[ARG_channel].u_int;
    format_t format = args[ARG_format].u_int;
    
    // note:  a "stereo frame" consists of a Left channel sample (4 bytes)
    // followed by a Right channel sample (4 bytes)
    uint32_t num_bytes_copied = 0;
    uint32_t num_stereo_frames = bufinfo_in.len / NUM_BYTES_IN_STEREO_FRAME;
    for (uint32_t i = 0; i < num_stereo_frames; i++) {
        // copy the specified channel sample from the input buffer
        int32_t sample = buf_in[2 * i + channel];

        switch (format) {
            case FORMAT_16_BIT:
                sample = sample >> 16;
                buf_out_16[i] = (int16_t)sample;
                num_bytes_copied += sizeof(int16_t);
                break;
            case FORMAT_32_BIT:
            default:
                buf_out_32[i] = sample;
                num_bytes_copied += sizeof(int32_t);
                break;
        }
    }
    return mp_obj_new_int(num_bytes_copied);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(i2stools_copy_obj, 0, i2stools_copy);

STATIC const mp_rom_map_elem_t i2stools_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),    MP_ROM_QSTR(MP_QSTR_i2stools) },
    { MP_ROM_QSTR(MP_QSTR_copy),        MP_ROM_PTR(&i2stools_copy_obj) },
    { MP_ROM_QSTR(MP_QSTR_LEFT),        MP_ROM_INT(CHANNEL_LEFT) },
    { MP_ROM_QSTR(MP_QSTR_RIGHT),       MP_ROM_INT(CHANNEL_RIGHT) },
    { MP_ROM_QSTR(MP_QSTR_B16),         MP_ROM_INT(FORMAT_16_BIT) },
    { MP_ROM_QSTR(MP_QSTR_B32),         MP_ROM_INT(FORMAT_32_BIT) },
};

STATIC MP_DEFINE_CONST_DICT(mp_module_i2stools_globals, i2stools_globals_table);

const mp_obj_module_t mp_module_i2stools = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&mp_module_i2stools_globals,
};
