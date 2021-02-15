# The MIT License (MIT)
# Copyright (c) 2021 Mike Teachman
# https://opensource.org/licenses/MIT

# Purpose:
# - read audio samples from a WAV file on SD Card
# - write audio samples to an I2S amplifier or DAC module 
# - the WAV file will play continuously in a loop until 
#   a keyboard interrupt is detected or the board is reset
# - the write() method blocks until the entire sample buffer is written to I2S
#
# Sample WAV files in examples/i2s folder:
#   "music-16k-16bits-mono.wav"  (16,000 samples/sec, 16 bit samples, mono)
#   "music-16k-32bits-mono.wav"
#   "music-16k-16bits-stereo.wav"
#   "music-16k-32bits-stereo.wav"
#
# Boards tested:
# - Pyboard V1.1
# - Pyboard D SF2W
# - Lolin D32 Pro
# - Lolin D32 with external SD Card module
#
# DAC hardware tested:
# - MAX98357A amplifier module (Adafruit I2S 3W Class D Amplifier Breakout)
# - PCM5102 stereo DAC module
#
# --- MIC on PyBoard V1.1 and PyBoard D
#     SCK - Y6  (SPI2 SCK)
#     WS -  Y5  (SPI2 NSS)
#     SD -  Y8  (SPI2 MOSI)
#
# --- DAC on PyBoard D
#     SCK - W29 (SPI1 SCK)
#     WS -  W16 (SPI1 NSS)
#     SD -  Y4  (SPI1 MOSI)
#
# --- DAC on PyBoard V1.1 ---
#     SCK - Y9  (SPI2 SCK)
#     WS -  Y4  (SPI2 NSS)
#     SD -  X22 (SPI2 MOSI)

import gc
import time
from machine import I2S
from machine import Pin
import uos

if uos.uname().machine.find('PYBv1') == 0:
    pass
elif uos.uname().machine.find('PYBD') == 0:
    import pyb
    pyb.Pin('EN_3V3').on()  # provide 3.3V on 3V3 output pin
    uos.mount(pyb.SDCard(), '/sd')
elif uos.uname().machine.find('ESP32') == 0:
    from machine import SDCard
    sd = SDCard(slot=3, sck=Pin(18), mosi=Pin(23), miso=Pin(19), cs=Pin(4))
    uos.mount(sd, '/sd')
else:
    print('Warning: script not tested with this board')    

gc.collect()

#======= AUDIO CONFIGURATION =======
WAV_FILE = 'music-16k-32bits-stereo.wav'
WAV_SAMPLE_SIZE_IN_BITS = 32
FORMAT = I2S.STEREO
SAMPLE_RATE_IN_HZ = 16000
#======= AUDIO CONFIGURATION =======

#======= I2S CONFIGURATION =======
sck_pin = Pin('W29') 
ws_pin = Pin('W16')  
sd_pin = Pin('Y4')
I2S_ID = 1
#======= I2S CONFIGURATION =======

audio_out = I2S(
    I2S_ID,
    sck=sck_pin, ws=ws_pin, sd=sd_pin, 
    mode=I2S.TX,
    bits=WAV_SAMPLE_SIZE_IN_BITS,
    format=FORMAT,
    rate=SAMPLE_RATE_IN_HZ,
    bufferlen=50000)

wav = open('/sd/{}'.format(WAV_FILE),'rb')
pos = wav.seek(44) # advance to first byte of Data section in WAV file

# allocate sample array
# memoryview used to reduce heap allocation
wav_samples = bytearray(10000)
wav_samples_mv = memoryview(wav_samples)

# continuously read audio samples from the WAV file 
# and write them to an I2S DAC
print('==========  START PLAYBACK ==========')
try:
    while True:
        num_read = wav.readinto(wav_samples_mv)
        # end of WAV file?
        if num_read == 0:
            # end-of-file, advance to first byte of Data section
            pos = wav.seek(44)
        else:
            num_written = audio_out.write(wav_samples_mv[:num_read])

except (KeyboardInterrupt, Exception) as e:
    print('caught exception {} {}'.format(type(e).__name__, e))

# cleanup 
wav.close()
if uos.uname().machine.find('PYBD') == 0:
    uos.umount('/sd')
if uos.uname().machine.find('ESP32') == 0:
    uos.umount('/sd')
    sd.deinit()
#audio_out.deinit()  
print('Done')  