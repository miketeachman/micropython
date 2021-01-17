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
from machine import Pin
from machine import SDCard
from machine import I2S
import time

NON_BLOCKING = False

def i2s_callback_tx(i2s):
    global wait_tx
    wait_tx = 0

def i2s_callback_rx(i2s):
    global wait_rx
    wait_rx = 0

# TODO - simplify by only recording the LEFT mic channel for MONO recordings

num_channels = {I2S.MONO:1, I2S.STEREO:2}

#======= USER CONFIGURATION =======
RECORD_TIME_IN_SECONDS = 10
SAMPLE_RATE_IN_HZ = 32000
FORMAT = I2S.MONO
WAV_SAMPLE_SIZE_IN_BITS = 32
#======= USER CONFIGURATION =======

NUM_CHANNELS = num_channels[FORMAT]
WAV_SAMPLE_SIZE_IN_BYTES = WAV_SAMPLE_SIZE_IN_BITS // 8
RECORDING_SIZE_IN_BYTES = RECORD_TIME_IN_SECONDS * SAMPLE_RATE_IN_HZ * WAV_SAMPLE_SIZE_IN_BYTES * NUM_CHANNELS

filename = {(I2S.MONO,16):'/sd/mic_mono_16bits.wav',
            (I2S.MONO,32):'/sd/mic_mono_32bits.wav',
            (I2S.STEREO,16):'/sd/mic_stereo_16bits.wav', 
            (I2S.STEREO,32):'/sd/mic_stereo_32bits.wav'} 

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

# configure SD card
#   slot=2 configures SD card to use the SPI3 controller (VSPI), DMA channel = 2
#   slot=3 configures SD card to use the SPI2 controller (HSPI), DMA channel = 1
sd = SDCard(slot=3, sck=Pin(18), mosi=Pin(23), miso=Pin(19), cs=Pin(4))
uos.mount(sd, "/sd")
wav_file = filename[(FORMAT, WAV_SAMPLE_SIZE_IN_BITS)]
wav = open(wav_file,'wb')

# create header for WAV file and write to SD card
wav_header = create_wav_header(
    SAMPLE_RATE_IN_HZ, 
    WAV_SAMPLE_SIZE_IN_BITS, 
    NUM_CHANNELS, 
    SAMPLE_RATE_IN_HZ * RECORD_TIME_IN_SECONDS
)
num_bytes_written = wav.write(wav_header)

sck_mic_pin = Pin(13)
ws_mic_pin = Pin(14)
sd_mic_pin = Pin(34)

audio_in = I2S(
    I2S.NUM0,
    sck=sck_mic_pin, ws=ws_mic_pin, sd=sd_mic_pin, 
    mode=I2S.RX,
    bits=WAV_SAMPLE_SIZE_IN_BITS,
    format=FORMAT,
    rate=SAMPLE_RATE_IN_HZ,
    buffer = 50000)

if NON_BLOCKING:
    audio_in.irq(i2s_callback_rx)
    wait_rx = 1
    
# allocate sample arrays
#   memoryview used to reduce heap allocation in while loop
mic_samples = bytearray(10000)
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
audio_in.deinit()
print('==========  DONE RECORDING ==========')

# ===== PLAYBACK ======
#PCM5102
sck_pin = Pin(33) 
ws_pin = Pin(25)  
sd_pin = Pin(32)

#MAX98357A
#sck_pin = Pin(21) 
#ws_pin = Pin(22)  
#sd_pin = Pin(27)

audio_out = I2S(
    I2S.NUM1,
    sck=sck_pin, ws=ws_pin, sd=sd_pin, 
    mode=I2S.TX,
    bits=WAV_SAMPLE_SIZE_IN_BITS,
    format=FORMAT,
    rate=SAMPLE_RATE_IN_HZ,
    buffer = 50000)

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
            
        if NON_BLOCKING:
            while wait_tx:
                time.sleep_ms(10)

                    
    except (KeyboardInterrupt, Exception) as e:
        print('caught exception {} {}'.format(type(e).__name__, e))
        break

wav.close()
uos.umount("/sd")
sd.deinit()
audio_in.deinit()
audio_out.deinit()
print('Done')