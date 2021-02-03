# The MIT License (MIT)
# Copyright (c) 2021 Mike Teachman
# https://opensource.org/licenses/MIT

# Purpose: Read audio samples from an I2S microphone and save to SD card
# - read audio samples from I2S hardware
# - convert 32-bit samples to specified bit size
# - write samples to a SD card file in WAV format
# - play file using I2S DAC
#
# Recorded WAV file is named based on USER CONFIGURATION:
#    examples
#       mic_stereo_16bits.wav
#       mic_mono_32bits.wav
#
# Hardware tested:
# - INMP441 microphone module 
# - MSM261S4030H0 microphone module
#

import uos
import pyb
from machine import Pin
from machine import I2S
import time
import gc

NON_BLOCKING = True

def i2s_callback_tx(i2s):
    global wait_tx
    wait_tx = 0

def i2s_callback_rx(i2s):
    global wait_rx
    wait_rx = 0

# for the Pyboard D hardware enable external 3.3v output 
if uos.uname().machine.find('PYBD') == 0:
    pyb.Pin('EN_3V3').on()
    uos.mount(pyb.SDCard(), '/sd')

num_channels = {I2S.MONO:1, I2S.STEREO:2}

#======= USER CONFIGURATION =======
RECORD_TIME_IN_SECONDS = 60
SAMPLE_RATE_IN_HZ = 22050
FORMAT = I2S.MONO
WAV_SAMPLE_SIZE_IN_BITS = 16
#======= USER CONFIGURATION =======

# TODO bug exists:  left channel appears on right channel for stereo 32 bits, alternates left and right channels on playback

NUM_CHANNELS = num_channels[FORMAT]
WAV_SAMPLE_SIZE_IN_BYTES = WAV_SAMPLE_SIZE_IN_BITS // 8
RECORDING_SIZE_IN_BYTES = RECORD_TIME_IN_SECONDS * SAMPLE_RATE_IN_HZ * WAV_SAMPLE_SIZE_IN_BYTES * NUM_CHANNELS

filename = {(I2S.MONO,16):'mic_mono_16bits.wav',
            (I2S.MONO,32):'mic_mono_32bits.wav',
            (I2S.STEREO,16):'mic_stereo_16bits.wav', 
            (I2S.STEREO,32):'mic_stereo_32bits.wav'} 

def create_wav_header(sampleRate, bitsPerSample, num_channels, num_samples):
    datasize = num_samples * num_channels * bitsPerSample // 8
    o = bytes("RIFF",'ascii')                                                   # (4byte) Marks file as RIFF
    o += (datasize + 36).to_bytes(4,'little')                                   # (4byte) File size in bytes excluding this and RIFF marker
    o += bytes("WAVE",'ascii')                                                  # (4byte) File type
    o += bytes("fmt ",'ascii')                                                  # (4byte) Format Chunk Marker
    o += (16).to_bytes(4,'little')                                              # (4byte) Length of above format data
    o += (1).to_bytes(2,'little')                                               # (2byte) Format type (1 - PCM)
    o += (num_channels).to_bytes(2,'little')                                    # (2byte)
    o += (sampleRate).to_bytes(4,'little')                                      # (4byte)
    o += (sampleRate * num_channels * bitsPerSample // 8).to_bytes(4,'little')  # (4byte)
    o += (num_channels * bitsPerSample // 8).to_bytes(2,'little')               # (2byte)
    o += (bitsPerSample).to_bytes(2,'little')                                   # (2byte)
    o += bytes("data",'ascii')                                                  # (4byte) Data Chunk Marker
    o += (datasize).to_bytes(4,'little')                                        # (4byte) Data size in bytes
    return o

WAV_FILE = filename[(FORMAT, WAV_SAMPLE_SIZE_IN_BITS)]
if uos.uname().machine.find('PYBD') == 0:
    wav_file = '/sd/{}'.format(WAV_FILE)
else:
    wav_file = WAV_FILE
    
wav = open(wav_file,'wb')

# create header for WAV file and write to SD card
wav_header = create_wav_header(
    SAMPLE_RATE_IN_HZ, 
    WAV_SAMPLE_SIZE_IN_BITS, 
    NUM_CHANNELS, 
    SAMPLE_RATE_IN_HZ * RECORD_TIME_IN_SECONDS
)
num_bytes_written = wav.write(wav_header)

#     SCK - Y6  (SPI2 SCK)
#     WS -  Y5  (SPI2 NSS)
#     SD -  Y8  (SPI2 MOSI)
sck_mic_pin = Pin('Y6') 
ws_mic_pin = Pin('Y5')  
sd_mic_pin = Pin('Y8')

audio_in = I2S(
    2, # TODO add constant for this
    sck=sck_mic_pin, ws=ws_mic_pin, sd=sd_mic_pin, 
    mode=I2S.RX,
    bits=WAV_SAMPLE_SIZE_IN_BITS,
    format=FORMAT,
    rate=SAMPLE_RATE_IN_HZ,
    bufferlen = 50000)

if NON_BLOCKING:
    audio_in.irq(i2s_callback_rx)
    wait_rx = 1
    
# allocate sample arrays
#   memoryview used to reduce heap allocation in while loop
mic_samples = bytearray(5000)
mic_samples_mv = memoryview(mic_samples)

num_sample_bytes_written_to_wav = 0

print('==========  START RECORDING ==========')
print('RECORDING_SIZE_IN_BYTES: ', RECORDING_SIZE_IN_BYTES)
while num_sample_bytes_written_to_wav < RECORDING_SIZE_IN_BYTES:
    try:
        # try to read a block of samples from the I2S microphone
        num_bytes_read_from_mic = audio_in.readinto(mic_samples_mv)
        
        if NON_BLOCKING:
            while wait_rx == 1:
                time.sleep_ms(1)
            wait_rx = 1
            
        if num_bytes_read_from_mic > 0:
            num_bytes_to_write = min(num_bytes_read_from_mic, RECORDING_SIZE_IN_BYTES - num_sample_bytes_written_to_wav)
            # write samples to WAV file
            num_bytes_written = wav.write(mic_samples_mv[:num_bytes_to_write])
            num_sample_bytes_written_to_wav += num_bytes_written

    except (KeyboardInterrupt, Exception) as e:
        print('caught exception {} {}'.format(type(e).__name__, e))
        break
    
wav.close()
uos.umount("/sd")
audio_in.deinit()
print('==========  DONE RECORDING ==========')
'''
# ===== PLAYBACK ======
#     SCK - W29 (SPI1 SCK)
#     WS -  W16 (SPI1 NSS)
#     SD -  Y4  (SPI1 MOSI)
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
    bufferlen = 50000)

if NON_BLOCKING:
    audio_out.irq(i2s_callback_tx)
    wait_tx = 0

wav = open(wav_file,'rb')
# advance to first byte of Data section in WAV file
pos = wav.seek(44) 

# allocate sample arrays
#   memoryview used to reduce heap allocation in while loop
wav_samples = bytearray(10000)
wav_samples_mv = memoryview(wav_samples)

print('==========  START PLAYBACK ==========')
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
            
            # frequent garbage collection is needed to avoid a crash in pyb platforms
            gc.collect()
            
        if NON_BLOCKING:
            while wait_tx:
                time.sleep_ms(1)

    except (KeyboardInterrupt, Exception) as e:
        print('caught exception {} {}'.format(type(e).__name__, e))
        break

wav.close()
uos.umount("/sd")
audio_out.deinit()
print('Done')
'''