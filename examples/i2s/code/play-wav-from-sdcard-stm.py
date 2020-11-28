# The MIT License (MIT)
# Copyright (c) 2020 Mike Teachman
# https://opensource.org/licenses/MIT

# Purpose:
# - read 16-bit audio samples from a mono formatted WAV file on SD card
# - write audio samples to an I2S amplifier or DAC module 
#
# Sample WAV files in wav_files folder:
#   "taunt-16k-16bits-mono.wav"
#   "taunt-16k-16bits-mono-12db.wav" (lower volume version)
#
# Hardware tested:
# - MAX98357A amplifier module (Adafruit I2S 3W Class D Amplifier Breakout)
# - PCM5102 stereo DAC module
#
# The WAV file will play continuously until a keyboard interrupt is detected or
# the STM32 is reset

import pyb
import time
import uos
from machine import I2S
from machine import Pin

def i2s_callback(s):
    print('callback worked')

# for the Pyboard D hardware enable external 3.3v output 
if uos.uname().machine.find('PYBD') == 0:
    pyb.Pin('EN_3V3').on()
    uos.mount(pyb.SDCard(), '/sd')

#======= USER CONFIGURATION =======
WAV_FILE = 'music-16k-16bits-stereo.wav'
WAV_SAMPLE_SIZE_IN_BITS = 16
FORMAT = I2S.STEREO
SAMPLE_RATE_IN_HZ = 16000
#======= USER CONFIGURATION =======

#     SCK -   A5/W6/X6,   B3/W29    (SPI1 SCK)
#     WS -    A4/W7/X5,   A15/W16   (SPI1 NSS)
#     SD -    A7/W14/X8,  B5/W46/Y4 (SPI1 MOSI)

#     SCK -   Y6  (SPI2 SCK)
#     WS -    Y5  (SPI2 NSS)
#     SD -    Y8  (SPI2 MOSI)

sck_pin = Pin('Y6') 
ws_pin = Pin('Y5')  
sd_pin = Pin('Y8')
'''
sck_pin = Pin('B3') 
ws_pin = Pin('A15')  
sd_pin = Pin('B5')
'''
buf_1 = bytearray(1024)
buf_2 = bytearray(1024)
buf_3 = bytearray(1024)
buf_4 = bytearray(1024)
buf_5 = bytearray(1024)

# TODO  define with memoryview?  see what happens with allocation in loop below.  GC?

audio_out = I2S(
    2, # TODO add constant for this
    sck=sck_pin, ws=ws_pin, sd=sd_pin, 
    mode=I2S.TX,
    bits=WAV_SAMPLE_SIZE_IN_BITS,
    format=FORMAT,
    rate=SAMPLE_RATE_IN_HZ,
    buffers = [buf_1, buf_2, buf_3, buf_4, buf_5],
    callback=i2s_callback)

if uos.uname().machine.find('PYBD') == 0:
    wav_file = '/sd/{}'.format(WAV_FILE)
else:
    wav_file = WAV_FILE

wav = open(wav_file,'rb')
wav.seek(44) # advance to first byte of Data section in WAV file
isStarted = False

# continuously read audio samples from the WAV file 
# and write them to an I2S DAC
while True:
    try:
        buffer = audio_out.getbuffer()
        if buffer != None:
            num_read = wav.readinto(buffer)
            num_written = audio_out.putbuffer(buffer)
            if isStarted == False:
                audio_out.start()
                isStarted = True
            # end of WAV file?
            if num_read == 0:
                # advance to first byte of Data section
                pos = wav.seek(44) 
                    
    except (KeyboardInterrupt, Exception) as e:
        print('caught exception {} {}'.format(type(e).__name__, e))
        break
    
wav.close()
if uos.uname().machine.find('PYBD') == 0:
    uos.umount("/sd")
audio_out.deinit()
print('Done')    