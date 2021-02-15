# The MIT License (MIT)
# Copyright (c) 2021 Mike Teachman
# https://opensource.org/licenses/MIT

# Purpose:
# - read audio samples from a WAV file on SD Card
# - write audio samples to an I2S amplifier or DAC module 
# - the WAV file will play continuously in a loop until 
#   a keyboard interrupt is detected or the board is reset
# - the write() method is non-blocking.  A callback() method sets a callback function
#   that is called when all the entire buffer has been written to the I2S peripheral
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
#     SD -  Y8  (SPI2 MOSI)  TODO check  MISO?
#
# --- DAC on PyBoard D
#     SCK - W29 (SPI1 SCK)
#     WS -  W16 (SPI1 NSS)
#     SD -  Y4  (SPI1 MOSI)  TODO check  MISO
#
# --- DAC on PyBoard V1.1 ---
#     SCK - Y9  (SPI2 SCK)
#     WS -  Y4  (SPI2 NSS)
#     SD -  X22 (SPI2 MOSI)  TODO check  MISO
#
# --- DAC on ESP32 ---
#     SCK - 33  (SPI1 or SP2)
#     WS -  25  (SPI1 or SP2)
#     SD -  32  (SPI1 or SP2)


import gc
import time
import micropython
from machine import I2S
from machine import Pin
import uos

PLAY = 0
PAUSE = 1
RESUME = 2
STOP = 3

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

def eof_callback(arg):
        global state
        print('end of audio file')
        #state = STOP  # uncomment to stop looping playback

def i2s_callback(arg):
    global state
    if state == PLAY:
        num_read = wav.readinto(wav_samples_mv)
        # end of WAV file?
        if num_read == 0:
            # end-of-file, advance to first byte of Data section
            pos = wav.seek(44)
            num_written = audio_out.write(silence)
            micropython.schedule(eof_callback, None)
        else:
            num_written = audio_out.write(wav_samples_mv[:num_read])
    elif state == RESUME:
        state = PLAY
        num_written = audio_out.write(silence)
    elif state == PAUSE:
        num_written = audio_out.write(silence)
    else: # STOP
        # cleanup 
        wav.close()
        if uos.uname().machine.find('PYBD') == 0:
            uos.umount('/sd')
        if uos.uname().machine.find('ESP32') == 0:
            uos.umount('/sd')
            sd.deinit()
        audio_out.deinit() 
        print('Done')  
          
#======= USER CONFIGURATION =======
WAV_FILE = 'music-16k-16bits-mono.wav'
WAV_SAMPLE_SIZE_IN_BITS = 16
FORMAT = I2S.MONO
SAMPLE_RATE_IN_HZ = 16000
#======= USER CONFIGURATION =======

#======= I2S CONFIGURATION =======
sck_pin = Pin('Y9') 
ws_pin = Pin('Y4')  
sd_pin = Pin('X22')
I2S_ID = 2
#======= I2S CONFIGURATION =======

audio_out = I2S(
    I2S_ID,
    sck=sck_pin, ws=ws_pin, sd=sd_pin, 
    mode=I2S.TX,
    bits=WAV_SAMPLE_SIZE_IN_BITS,
    format=FORMAT,
    rate=SAMPLE_RATE_IN_HZ,
    bufferlen=50000)

audio_out.callback(i2s_callback)
state = PAUSE

wav = open('/sd/{}'.format(WAV_FILE),'rb')
pos = wav.seek(44) # advance to first byte of Data section in WAV file

# allocate a small array of blank samples
silence = bytearray(1000)

# allocate sample array buffer
wav_samples = bytearray(10000)
wav_samples_mv = memoryview(wav_samples)

num_written = audio_out.write(silence)

# add runtime code here ....
# changing 'state' will affect playback of audio file

print("starting playback for 10s")
state = PLAY
time.sleep(10)
print("pausing playback for 10s")
state = PAUSE
time.sleep(10)
print("resuming playback")
state = RESUME