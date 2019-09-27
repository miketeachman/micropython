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

#include "py/nlr.h"
#include "py/obj.h"
#include "py/runtime.h"
#include "py/binary.h"
#include <string.h>
#include <stdio.h>

STATIC mp_obj_t function_bitcrusher(mp_obj_t samples_in, mp_obj_t samples_out) {

    mp_buffer_info_t bufinfo_in;
    mp_get_buffer_raise(samples_in, &bufinfo_in, MP_BUFFER_READ);
    uint8_t *buf_in = bufinfo_in.buf;

    mp_buffer_info_t bufinfo_out;
    mp_get_buffer_raise(samples_out, &bufinfo_out, MP_BUFFER_WRITE);
    uint8_t *buf_out = bufinfo_out.buf;

    uint16_t num_samples = bufinfo_in.len / 8;

    for (uint16_t i=0; i<num_samples; i++) {
        buf_out[2*i] = buf_in[8*i + 2];
        buf_out[2*i + 1] = buf_in[8*i + 3];
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(function_bitcrusher_obj, function_bitcrusher);

const mp_obj_type_t bitcrusher_type = {
    { &mp_type_type },
    .name = MP_QSTR_bitcrusher,
};

STATIC const mp_rom_map_elem_t bitcrusher_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),            MP_ROM_QSTR(MP_QSTR_bitcrusher) },
    { MP_ROM_QSTR(MP_QSTR_bitcrusher),          MP_ROM_PTR(&function_bitcrusher_obj) },
};

STATIC MP_DEFINE_CONST_DICT (
    mp_module_bitcrusher_globals,
    bitcrusher_globals_table
);

const mp_obj_module_t mp_module_bitcrusher = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&mp_module_bitcrusher_globals,
};

