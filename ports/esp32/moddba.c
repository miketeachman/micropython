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
 *
 */

/* Module:  dba
 *
 * Purpose: calculate dBA (A-weighting decibels)
 *          from an input stream of digital audio samples
 *
 * Class DBA:
 *      Constructor: DBA(samples, resolution, coeffb, coeffb)
 *          samples: number of samples to process before calc() returns a dBA result
 *          resolution: bit resolution of samples being processed, either B16 or B24 (recommend B16)
 *                      for B16, calc() needs a buffer containing 16-bit samples
 *                      for B24, calc() needs a buffer containing 32-bit samples (highest order byte = 0x00)
 *
 *          coeffa: "a" weighting coefficients for IIR filter
 *          coeffb: "b" weighting coefficients for IIR filter
 *                   Notes:
 *                   - weighting coefficients need to match the sampling frequency of the audio samples
 *                   - supports up to 6th order IIR filter
 *
 *      Function:  calc(sample_data)
 *
 *                 sample_data:  bytearray of samples to process
 *
 *                 samples in array need to be ordered from the LS byte to MS byte:
 *                 example for 16-bits samples: 0x3456 0xABCD 0x9900
 *                 bytearray ordering, from 0: 0x56 0x34 0xCD 0xAB 0x00 0x99
 *
 *          Options to use calc()
 *          1. Returns a dBA result each time calc() is called. For this case, the sample_data buffer
 *          needs to contain the number of samples specified in the constructor (or more...)
 *          2. Incremental calculation.  Incrementally process sample data.
 *          Most calls to calc() will return Null.
 *          A final dBA result will be returned by calc() when the accumulated number of samples (defined in
 *          the constructor) have been processed by calc().
 *
 *          The state of the IIR filter is maintained between calls to calc()
 *
 *          When to use the incremental calculation option:
 *              #1:  reduce blocking time for co-routines in an asyncio design
 *              #2:  eliminate the need for a large sample buffer
 *
 *          CAUTION:  arguments have no validity checks
 *
 *          Performance Tests:  incremental runs of calc(), returning None:  250us
 *                              calc() producing final result:  600us
 *
 *                              conditions:  ESP32 with 4MB RAM @ 240MHz.
 *                                           sample_data buffer containing 256 samples, 16-bit
 *
 *          MicroPython Example: calculate dBA every 2 seconds, incrementally, 6th order IIR filter,
 *                               data sampled at 48kHz (use co-efficients for 48kHz)
 *          =====================================================================
 *                  NUMBER_OF_SAMPLES = 48000 * 2
 *                  spl = dba.DBA(samples=NUMBER_OF_SAMPLES, resolution=dba.B16,
 *                                coeffa=(1.0, -2.12979364760736134, 0.42996125885751674, 1.62132698199721426, -0.96669962900852902, 0.00121015844426781, 0.04400300696788968),
 *                                coeffb=(0.169994948147430, 0.280415310498794, -1.120574766348363, 0.131562559965936, 0.974153561246036, -0.282740857326553, -0.152810756202003))
 *
 *                  NUM_BYTES_IN_SAMPLE_BLOCK = 1000
 *                  samples = bytearray(NUM_BYTES_IN_SAMPLE_BLOCK)
 *
 *                  while True:
 *                      <fill samples array with 16-bit audio samples from I2S microphone, WAV file, etc>
 *                      res = spl.calc(samples)
 *                      if (res != None):
 *                          # dba result ready
 *                          print("dBA = {}".format(res))
 *          =====================================================================
 *
 *          The following IIR Filter Weighting Coefficients have been verified to work:
 *              10kHz (calculated using pyfilterbank)
 *                  coeffa=(1.0, -2.3604841 ,  0.83692802,  1.54849677, -0.96903429, -0.25092355,  0.1950274)
 *                  coeffb=(0.61367941, -1.22735882, -0.61367941,  2.45471764, -0.61367941, -1.22735882,  0.61367941)
 *              20kHz (calculated using pyfilterbank)
 *                  coeffa=(1.0, -3.11810631,  2.99441375, -0.33169269, -0.77271226, 0.15355108,  0.07454692)
 *                  coeffb=(0.47577598, -0.95155197, -0.47577598,  1.90310393, -0.47577598, -0.95155197,  0.47577598)
 *              48kHz (copied from https://dsp.stackexchange.com/questions/36077/design-of-a-digital-a-weighting-filter-with-arbitrary-sample-rate/36080)
 *                  coeffa=(1.0, -2.12979364760736134, 0.42996125885751674, 1.62132698199721426, -0.96669962900852902, 0.00121015844426781, 0.04400300696788968)
 *                  coeffb=(0.169994948147430, 0.280415310498794, -1.120574766348363, 0.131562559965936, 0.974153561246036, -0.282740857326553, -0.152810756202003)
 *
 *              Coefficients for samplings frequencies <= 20kHz can be calculated using the Python PyFilterbank package
 *              (beyond 20kHz this package produces filter coefficients that are unstable in this IIR implementation)
 *                  https://github.com/SiggiGue/pyfilterbank
 *
 *                  install python packages
 *                      pip install https://github.com/SiggiGue/pyfilterbank
 *                      pip install numpy
 *                      pip install scipy
 *
 *                  Python REPL.  Example for calculating b,a weighting coefficients for sampling frequency = 10kHz
 *                      >>> import splweighting
 *                      >>> splweighting.a_weighting_coeffs_design(10000)
 *
 *                      (array([ 0.61367941, -1.22735882, -0.61367941,  2.45471764, -0.61367941,
 *                      -1.22735882,  0.61367941]), array([ 1.        , -2.3604841 ,  0.83692802,  1.54849677, -0.96903429,
 *                      -0.25092355,  0.1950274 ]))
 *
 *          Acknowledgements:
 *
 *              1. Thanks to Ivan Kostoski for his excellent project describing how to use a
 *                 ESP32 device to calculate dB(A) using an IIR filter.
 *                    https://hackaday.io/project/166867-esp32-i2s-slm
 *
 *              2. Calculating A-weighting coefficients:  https://github.com/SiggiGue/pyfilterbank
 */

#include "py/nlr.h"
#include "py/obj.h"
#include "py/runtime.h"
#include "py/binary.h"
#include <math.h>

#define DBFS_TO_RMS_OFFSET  (3.0103) // offset to account for dB(A) calculations done using RMS math and not DBFS
#define MIC_REF_SPL_DB      (94)     // standard reference sound pressure level (SPL) for I2S MEMS microphones

// IIR filters limited to 7 weighting coefficients (up to 6th order filter)
#define MAX_NUMBER_COEFF_A (7)
#define MAX_NUMBER_COEFF_B (7)

typedef enum {
    RESOLUTION_16_BIT,
    RESOLUTION_24_BIT,
} resolution_t;

typedef struct _dba_obj_t {
    mp_obj_base_t base;
    uint32_t      refampl;
    uint32_t      num_samples_total;
    resolution_t  resolution;
    float         x[MAX_NUMBER_COEFF_B];        // sample input history, used in feedforward filter section
    float         y[MAX_NUMBER_COEFF_A-1];      // filter output history, used in feedback filter section
    float         a[2*(MAX_NUMBER_COEFF_A-1)-1];// a[i]s are IIR filter coefficients in the 'y' feedback section
    float         b[2*MAX_NUMBER_COEFF_B-1];    // b[i]s are IIR filter coefficients in the 'x' feedforward section
    uint8_t       num_coeff_b;
    uint8_t       num_coeff_a;
    uint8_t       i_a;          // maintains index into filter output history 'y'
    uint8_t       i_b;          // maintains index into filter input history 'x'
    float         sum_sqr;
    uint32_t      running_sample_count;
} dba_obj_t;

mp_obj_t dba_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args) {

    enum { ARG_samples, ARG_resolution, ARG_coeffa, ARG_coeffb };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_samples,      MP_ARG_REQUIRED | MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 10000} },
        { MP_QSTR_resolution,                     MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = RESOLUTION_16_BIT} },
        { MP_QSTR_coeffa,       MP_ARG_REQUIRED | MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_coeffb,       MP_ARG_REQUIRED | MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    dba_obj_t *self = m_new_obj(dba_obj_t);
    self->base.type = type;

    self->num_samples_total = args[ARG_samples].u_int;

    self->resolution = args[ARG_resolution].u_int;
    switch (self->resolution) {
        case RESOLUTION_16_BIT:
        default:
            self->refampl = 1642;
            break;
        case RESOLUTION_24_BIT:
            self->refampl = 420426;
            break;
    }

    mp_obj_t *coeff_a;
    mp_uint_t num_coeff_a;
    mp_obj_get_array(args[ARG_coeffa].u_obj, &num_coeff_a, &coeff_a);

    //
    // Start initialize IIR filter
    //
    self->num_coeff_a = num_coeff_a-1;  // -1 because a0 is not saved to object

    float coeff_a_temp[MAX_NUMBER_COEFF_A];
    for (uint8_t i=0; i<num_coeff_a; i++) {
        coeff_a_temp[i] = mp_obj_get_float(coeff_a[i]);
    }

    mp_obj_t *coeff_b;
    mp_uint_t num_coeff_b;
    mp_obj_get_array(args[ARG_coeffb].u_obj, &num_coeff_b, &coeff_b);

    self->num_coeff_b = num_coeff_b;

    float coeff_b_temp[MAX_NUMBER_COEFF_B];
    for (uint8_t i=0; i<num_coeff_b; i++) {
        coeff_b_temp[i] = mp_obj_get_float(coeff_b[i]);
    }

    const float *a = &coeff_a_temp[1];
    const float *b = coeff_b_temp;
    float a0 = coeff_a_temp[0];

    for (uint8_t i = 0; i < 2*self->num_coeff_a-1; i++) {
        self->a[i] = a[(2*self->num_coeff_a - 2 - i) % self->num_coeff_a] / a0;
    }

    for (uint8_t i = 0; i < self->num_coeff_a; i++) {
        self->y[i] = 0;
    }
    self->i_a = 0;

    for (uint8_t i = 0; i < 2*self->num_coeff_b-1; i++) {
        self->b[i] = b[(2*self->num_coeff_b - 1 - i) % self->num_coeff_b] / a0;
    }

    for (uint8_t i = 0; i < self->num_coeff_b; i++) {
        self->x[i] = 0;
    }
    self->i_b = 0;
    self->running_sample_count = 0;
    self->sum_sqr = 0;
    //
    // End initialize IIR filter
    //

    return MP_OBJ_FROM_PTR(self);
}

STATIC mp_obj_t dba_calc(mp_obj_t self_in, mp_obj_t samples_in) {
    dba_obj_t *self = self_in;

    mp_buffer_info_t bufinfo_in;
    mp_get_buffer_raise(samples_in, &bufinfo_in, MP_BUFFER_READ);

    uint16_t num_samples_in;
    if (self->resolution == RESOLUTION_24_BIT) {
        num_samples_in = bufinfo_in.len / 4;  // assume 24-bit samples appear in buffer as 32-bit values
    } else {
        num_samples_in = bufinfo_in.len / 2;  // assume 16-bit samples appear in buffer as 16-bit values
    }

    // Start IIR Filter algorithm
    for (uint16_t i = 0; i < num_samples_in; i++) {
        int32_t *buf_in_32 = bufinfo_in.buf;
        int16_t *buf_in_16 = bufinfo_in.buf;

        int32_t sample;
        if (self->resolution == RESOLUTION_24_BIT) {
            sample = buf_in_32[i] >> 8;
        } else {
            sample = (int32_t)buf_in_16[i];
        }

        self->x[self->i_b] = (float)sample;

        float b_terms = 0;
        float *b_shift = &self->b[self->num_coeff_b - self->i_b - 1];
        for(uint8_t i = 0; i < self->num_coeff_b; i++) {
            b_terms +=  self->x[i] * b_shift[i];
        }

        float a_terms = 0;
        float *a_shift = &self->a[self->num_coeff_a - self->i_a - 1];
        for(uint8_t i = 0; i < self->num_coeff_a; i++) {
            a_terms += self->y[i] * a_shift[i];
        }
        float filtered = b_terms - a_terms;
        self->y[self->i_a] = filtered;

        self->i_b++;
        if(self->i_b == self->num_coeff_b) {
            self->i_b = 0;
        }

        self->i_a++;
        if(self->i_a == self->num_coeff_a) {
            self->i_a = 0;
        }
        // End IIR Filter algorithm

        self->sum_sqr += filtered * filtered;
    }

    self->running_sample_count += num_samples_in;

    //  "period" calculation time reached?
    if (self->running_sample_count >= self->num_samples_total) {
        // yes:  calculate dbA and return result
        float dba = DBFS_TO_RMS_OFFSET + MIC_REF_SPL_DB + 20 * log10(sqrt(self->sum_sqr / self->running_sample_count) / self->refampl);
        self->running_sample_count = 0;
        self->sum_sqr = 0;
        return mp_obj_new_float(dba);
    }
    else {
        // no:  return None
        return mp_const_none;
    }
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(dba_calc_obj, dba_calc);


// Methods for DBA class
STATIC const mp_rom_map_elem_t dba_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_calc),   MP_ROM_PTR(&dba_calc_obj) },
};

STATIC MP_DEFINE_CONST_DICT(dba_locals_dict, dba_locals_dict_table);

const mp_obj_type_t dba_type = {
    { &mp_type_type },
    .name = MP_QSTR_DBA,
    .make_new = dba_make_new,
    .locals_dict = (mp_obj_dict_t*)&dba_locals_dict,
};

STATIC const mp_rom_map_elem_t dba_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),        MP_ROM_QSTR(MP_QSTR_dba) },
    { MP_ROM_QSTR(MP_QSTR_DBA),             MP_ROM_PTR(&dba_type) },
    { MP_ROM_QSTR(MP_QSTR_B16),             MP_ROM_INT(RESOLUTION_16_BIT) },
    { MP_ROM_QSTR(MP_QSTR_B24),             MP_ROM_INT(RESOLUTION_24_BIT) },
};

STATIC MP_DEFINE_CONST_DICT(mp_module_dba_globals, dba_globals_table);

const mp_obj_module_t mp_module_dba = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&mp_module_dba_globals,
};
