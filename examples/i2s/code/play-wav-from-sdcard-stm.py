# The MIT License (MIT)
# Copyright (c) 2021 Mike Teachman
# https://opensource.org/licenses/MIT

# Purpose:
# - read 16-bit audio samples from a mono formatted WAV file on SD card
# - write audio samples to an I2S amplifier or DAC module 
#
# Sample WAV files in wav_files folder:
#   "music-16k-16bits-mono.wav"
#   "music-16k-32bits-mono.wav"
#   "music-16k-16bits-stereo.wav"
#   "music-16k-32bits-stereo.wav"
#
# Hardware tested:
# - MAX98357A amplifier module (Adafruit I2S 3W Class D Amplifier Breakout)
# - PCM5102 stereo DAC module
#
# The WAV file will play continuously until a keyboard interrupt is detected or
# the STM32 is reset

import pyb
import uos
from machine import I2S
from machine import Pin
import time
import micropython
import gc
micropython.alloc_emergency_exception_buf(100)
pyb.fault_debug(True)

NON_BLOCKING = True

def i2s_callback(s):
    print('----------------- cb')
    global wait_tx
    wait_tx = 0

# for the Pyboard D hardware enable external 3.3v output 
if uos.uname().machine.find('PYBD') == 0:
    pyb.Pin('EN_3V3').on()
    uos.mount(pyb.SDCard(), '/sd')

#======= USER CONFIGURATION =======
WAV_FILE = 'music-16k-16bits-mono.wav'
WAV_SAMPLE_SIZE_IN_BITS = 16
FORMAT = I2S.MONO
SAMPLE_RATE_IN_HZ = 16000
#======= USER CONFIGURATION =======

#     SCK - W29 (SPI1 SCK)
#     WS -  W16 (SPI1 NSS)
#     SD -  Y4  (SPI1 MOSI)

#     SCK - Y6  (SPI2 SCK)
#     WS -  Y5  (SPI2 NSS)
#     SD -  Y8  (SPI2 MOSI)

sck_pin = Pin('W29') 
ws_pin = Pin('W16')  
sd_pin = Pin('Y4')

audio_out = I2S(
    1, # TODO add constant for this
    sck=sck_pin, ws=ws_pin, sd=sd_pin, 
    mode=I2S.TX,
    bits=WAV_SAMPLE_SIZE_IN_BITS,
    format=FORMAT,
    rate=SAMPLE_RATE_IN_HZ,
    bufferlen=50000)

if NON_BLOCKING:
    audio_out.irq(i2s_callback)
    wait_tx = 1

if uos.uname().machine.find('PYBD') == 0:
    wav_file = '/sd/{}'.format(WAV_FILE)
else:
    wav_file = WAV_FILE

wav = open(wav_file,'rb')
wav.seek(44) # advance to first byte of Data section in WAV file

# allocate sample array
#   memoryview used to reduce heap allocation in while loop
wav_samples = bytearray(10000)
wav_samples_mv = memoryview(wav_samples)

# continuously read audio samples from the WAV file 
# and write them to an I2S DAC
while True:
    try:
        num_read = wav.readinto(wav_samples_mv)
        # end of WAV file?
        if num_read == 0:
            # advance to first byte of Data section
            pos = wav.seek(44)
        else:
            if NON_BLOCKING:
                wait_tx = 1
            num_written = audio_out.write(wav_samples_mv[:num_read])
 
            # frequent garbage collection is needed to avoid a crash
            gc.collect()
            
        if NON_BLOCKING:
            while wait_tx:
                time.sleep_ms(1)
                    
    except (KeyboardInterrupt, Exception) as e:
        print('caught exception {} {}'.format(type(e).__name__, e))
        break
    
wav.close()
if uos.uname().machine.find('PYBD') == 0:
    uos.umount("/sd")
audio_out.deinit()
print('Done')    