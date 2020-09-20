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

#include <string.h>
#include "py/obj.h"
#include "py/runtime.h"
#include "modmachine.h"
#include "mphalport.h"
#include "driver/i2s.h"

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
    mp_obj_base_t          base;
    i2s_port_t             id;
    int8_t                 sck;
    int8_t                 ws;
    int8_t                 sd;
    uint8_t                mode;
    i2s_bits_per_sample_t  bits;
    i2s_channel_fmt_t      format;
    int32_t                rate;
    bool                   used;
} machine_i2s_obj_t;

// Static object mapping to I2S peripherals  TODO change to root pointer? 
//   note:  I2S implementation makes use of the following mapping between I2S peripheral and I2S object
//      I2S peripheral 1:  machine_i2s_obj[0]
//      I2S peripheral 2:  machine_i2s_obj[1]
STATIC machine_i2s_obj_t machine_i2s_obj[I2S_NUM_MAX] = {
        [0].used = false,
        [1].used = false };

//  For 32-bit stereo, the ESP-IDF API has a channel convention of R, L channel ordering
//  The following function takes a buffer having L,R channel ordering and swaps channels   TODO rewrite
//  to work with the ESP-IDF ordering R, L
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

STATIC void machine_i2s_init_helper(machine_i2s_obj_t *self, size_t n_pos_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {

    enum {
        ARG_sck,
        ARG_ws,
        ARG_sd,
        ARG_mode,
        ARG_bits,
        ARG_format,
        ARG_rate,
    };

    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_sck,              MP_ARG_KW_ONLY | MP_ARG_REQUIRED | MP_ARG_OBJ,   {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_ws,               MP_ARG_KW_ONLY | MP_ARG_REQUIRED | MP_ARG_OBJ,   {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_sd,               MP_ARG_KW_ONLY | MP_ARG_REQUIRED | MP_ARG_OBJ,   {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_mode,             MP_ARG_KW_ONLY | MP_ARG_REQUIRED | MP_ARG_INT,   {.u_int = -1} },
        { MP_QSTR_bits,             MP_ARG_KW_ONLY | MP_ARG_REQUIRED | MP_ARG_INT,   {.u_int = -1} },
        { MP_QSTR_format,           MP_ARG_KW_ONLY | MP_ARG_REQUIRED | MP_ARG_INT,   {.u_int = -1} },
        { MP_QSTR_rate,             MP_ARG_KW_ONLY | MP_ARG_REQUIRED | MP_ARG_INT,   {.u_int = -1} },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_pos_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);
    
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
        (i2s_format != I2S_CHANNEL_FMT_ONLY_RIGHT) &&
        (i2s_format != I2S_CHANNEL_FMT_ONLY_LEFT)) {
        mp_raise_ValueError(MP_ERROR_TEXT("Format is not valid"));
    }

    // is Sample Rate valid?
    // No validation done:  ESP-IDF API does not indicate a valid range for sample rate

    // is APLL Rate valid?
    // No validation done:  ESP-IDF API does not indicate a valid range for APLL rate
    
    self->sck = sck;
    self->ws = ws;
    self->sd = sd;
    self->mode = args[ARG_mode].u_int;
    self->bits = args[ARG_bits].u_int;
    self->format = args[ARG_format].u_int;
    self->rate = args[ARG_rate].u_int;

    i2s_config_t i2s_config;
    i2s_config.communication_format = I2S_COMM_FORMAT_I2S;
    i2s_config.mode = self->mode;
    i2s_config.bits_per_sample = self->bits;
    i2s_config.channel_format = self->format;
    i2s_config.sample_rate = self->rate;
    i2s_config.intr_alloc_flags = ESP_INTR_FLAG_LOWMED; // allows simultaneous use of both I2S channels
    i2s_config.dma_buf_count = 10;  // TODO
    i2s_config.dma_buf_len = 256;  // TODO
    i2s_config.use_apll = false;

    // uninstall I2S driver when changes are being made to an active I2S peripheral
    if (self->used) {
        i2s_driver_uninstall(self->id);
    }

    esp_err_t ret = i2s_driver_install(self->id, &i2s_config, 0, NULL);
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
    
    if ((self->bits == 32) && (self->format == I2S_CHANNEL_FMT_RIGHT_LEFT)) {
        machine_i2s_swap_32_bit_stereo_channels(&bufinfo);
    }

    return mp_obj_new_int(num_bytes_read);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(machine_i2s_readinto_obj, 2, machine_i2s_readinto);

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
    
    if ((self->bits == 32) && (self->format == I2S_CHANNEL_FMT_RIGHT_LEFT)) {
        machine_i2s_swap_32_bit_stereo_channels(&bufinfo);
    }

    uint32_t num_bytes_written = 0;
    esp_err_t ret = i2s_write(self->id, bufinfo.buf, bufinfo.len, &num_bytes_written, portMAX_DELAY);
    switch (ret) {
        case ESP_OK:
            break;
        case ESP_ERR_INVALID_ARG:
            mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("I2S write:  Parameter error"));
            break;
        default:
            // this error not documented in ESP-IDF
            mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("I2S write:  Undocumented error")); 
            break;
    }
    
    return mp_obj_new_int(num_bytes_written);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(machine_i2s_write_obj, 2, machine_i2s_write);

STATIC mp_obj_t machine_i2s_deinit(mp_obj_t self_in) {
    machine_i2s_obj_t *self = self_in;
    i2s_driver_uninstall(self->id);
    self->used = false;
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(machine_i2s_deinit_obj, machine_i2s_deinit);

STATIC const mp_rom_map_elem_t machine_i2s_locals_dict_table[] = {
    // Methods
    { MP_ROM_QSTR(MP_QSTR_init),            MP_ROM_PTR(&machine_i2s_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_readinto),        MP_ROM_PTR(&machine_i2s_readinto_obj) },
    { MP_ROM_QSTR(MP_QSTR_write),           MP_ROM_PTR(&machine_i2s_write_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinit),          MP_ROM_PTR(&machine_i2s_deinit_obj) },

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