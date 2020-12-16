# The MIT License (MIT)
# Copyright (c) 2020 Mike Teachman
# https://opensource.org/licenses/MIT

# Purpose: Read audio samples from an I2S microphone and save to SD card
# - read 32 bit audio samples from I2S hardware
# - optionally convert 32-bit samples to 16-bit
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
import i2stools
import time

LEFT = 0
RIGHT = 1
LEFT_RIGHT = 2

def i2s_callback(s):
    pass

# TODO - simplify by only recording the LEFT mic channel for MONO recordings

num_channels = {LEFT:1, RIGHT:1, LEFT_RIGHT:2}
channel_map = {LEFT:i2stools.LEFT, RIGHT:i2stools.RIGHT, LEFT_RIGHT:i2stools.LEFT_RIGHT}
bit_map = {16:i2stools.B16, 32:i2stools.B32}

#======= USER CONFIGURATION =======
RECORD_TIME_IN_SECONDS = 10
SAMPLE_RATE_IN_HZ = 16000
MIC_CHANNEL = LEFT
FORMAT = I2S.MONO
WAV_SAMPLE_SIZE_IN_BITS = 16
#======= USER CONFIGURATION =======

NUM_CHANNELS = num_channels[MIC_CHANNEL]
MIC_SAMPLE_SIZE_IN_BITS = 32
MIC_BUFFER_SIZE_IN_BYTES = 1024
WAV_SAMPLE_SIZE_IN_BYTES = WAV_SAMPLE_SIZE_IN_BITS // 8
WAV_BUFFER_SIZE_IN_BYTES = MIC_BUFFER_SIZE_IN_BYTES // MIC_SAMPLE_SIZE_IN_BITS * WAV_SAMPLE_SIZE_IN_BITS
WAV_BYTES_TO_WRITE = RECORD_TIME_IN_SECONDS * SAMPLE_RATE_IN_HZ * WAV_SAMPLE_SIZE_IN_BYTES * NUM_CHANNELS

filename = {(LEFT,16):'/sd/mic_left_16bits.wav',
            (LEFT,32):'/sd/mic_left_32bits.wav',
            (RIGHT,16):'/sd/mic_right_16bits.wav',
            (RIGHT,32):'/sd/mic_right_32bits.wav', 
            (LEFT_RIGHT,16):'/sd/mic_stereo_16bits.wav', 
            (LEFT_RIGHT,32):'/sd/mic_stereo_32bits.wav'} 

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

sck_mic_pin = Pin(13)
ws_mic_pin = Pin(14)
sd_mic_pin = Pin(34)

buf_1 = bytearray(MIC_BUFFER_SIZE_IN_BYTES)
buf_2 = bytearray(MIC_BUFFER_SIZE_IN_BYTES)
buf_3 = bytearray(MIC_BUFFER_SIZE_IN_BYTES)
buf_4 = bytearray(MIC_BUFFER_SIZE_IN_BYTES)
buf_5 = bytearray(MIC_BUFFER_SIZE_IN_BYTES)

audio_in = I2S(
    I2S.NUM0,
    sck=sck_mic_pin, ws=ws_mic_pin, sd=sd_mic_pin, 
    mode=I2S.RX,
    bits=MIC_SAMPLE_SIZE_IN_BITS,
    format=I2S.STEREO,
    rate=SAMPLE_RATE_IN_HZ,
    buffers = [buf_1, buf_2, buf_3, buf_4, buf_5],
    callback=i2s_callback)

sck_pin = Pin(33) 
ws_pin = Pin(25)  
sd_pin = Pin(32)

buf_6 = bytearray(1024)
buf_7 = bytearray(1024)
buf_8 = bytearray(1024)
buf_9 = bytearray(1024)
buf_10 = bytearray(1024)

audio_out = I2S(
    I2S.NUM1,
    sck=sck_pin, ws=ws_pin, sd=sd_pin, 
    mode=I2S.TX,
    bits=WAV_SAMPLE_SIZE_IN_BITS,
    format=FORMAT,
    rate=SAMPLE_RATE_IN_HZ,
    buffers = [buf_6, buf_7, buf_8, buf_9, buf_10],
    callback=i2s_callback)

# configure SD card
#   slot=2 configures SD card to use the SPI3 controller (VSPI), DMA channel = 2
#   slot=3 configures SD card to use the SPI2 controller (HSPI), DMA channel = 1
sd = SDCard(slot=3, sck=Pin(18), mosi=Pin(23), miso=Pin(19), cs=Pin(4))
uos.mount(sd, "/sd")
wav_file = filename[(MIC_CHANNEL, WAV_SAMPLE_SIZE_IN_BITS)]
wav = open(wav_file,'wb')

# create header for WAV file and write to SD card
wav_header = create_wav_header(
    SAMPLE_RATE_IN_HZ, 
    WAV_SAMPLE_SIZE_IN_BITS, 
    NUM_CHANNELS, 
    SAMPLE_RATE_IN_HZ * RECORD_TIME_IN_SECONDS
)
num_bytes_written = wav.write(wav_header)

wav_bytes = bytearray(WAV_BUFFER_SIZE_IN_BYTES)
wav_bytes_mv = memoryview(wav_bytes)

isStarted = False
num_sample_bytes_written_to_wav = 0

print('==========  START RECORDING ==========')
while num_sample_bytes_written_to_wav < WAV_BYTES_TO_WRITE :
    try:
        if isStarted == False:
            audio_in.start()
            isStarted = True

        buffer = audio_in.getbuffer()
        
        if buffer != None:
            num_bytes_copied = i2stools.copy(bufin=buffer, 
                                             bufout=wav_bytes_mv, 
                                             channel=channel_map[MIC_CHANNEL], 
                                             format=bit_map[WAV_SAMPLE_SIZE_IN_BITS])
            
            num_bytes_to_write = min(num_bytes_copied, WAV_BYTES_TO_WRITE - num_sample_bytes_written_to_wav)
            num_bytes_written = wav.write(wav_bytes_mv[:num_bytes_to_write])
            audio_in.putbuffer(buffer)
            num_sample_bytes_written_to_wav += num_bytes_written
            
    except (KeyboardInterrupt, Exception) as e:
        print('caught exception {} {}'.format(type(e).__name__, e))
        break
    
wav.close()
audio_in.deinit()
print('==========  DONE RECORDING ==========')


# ===== PLAYBACK ======
wav = open(wav_file,'rb')
pos = wav.seek(44) # advance to first byte of Data section in WAV file
isStarted = False

print('==========  START PLAYBACK ==========')
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
uos.umount("/sd")
sd.deinit()
audio_in.deinit()
audio_out.deinit()
print('Done')