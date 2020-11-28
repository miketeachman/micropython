# The MIT License (MIT)
# Copyright (c) 2020 Mike Teachman
# https://opensource.org/licenses/MIT

# Purpose: Read audio samples from an I2S microphone and save to SD card
# - read 32-bit audio samples from I2S hardware
# - optionally convert 32-bit samples to 16-bit
# - write samples to a SD card file in WAV format
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
# For MONO recordings only the Left channel is used

import uos
from machine import Pin
from machine import SDCard
from machine import I2S
import i2stools
import time

LEFT = 0
RIGHT = 1
LEFT_RIGHT = 2

num_channels = {LEFT:1, RIGHT:1, LEFT_RIGHT:2}
channel_map = {LEFT:i2stools.LEFT, RIGHT:i2stools.RIGHT, LEFT_RIGHT:i2stools.LEFT_RIGHT}
bit_map = {16:i2stools.B16, 32:i2stools.B32}

#======= USER CONFIGURATION =======
RECORD_TIME_IN_SECONDS = 10
SAMPLE_RATE_IN_HZ = 16000
MIC_CHANNEL = LEFT_RIGHT
WAV_SAMPLE_SIZE_IN_BITS = 16
#======= USER CONFIGURATION =======

NUM_CHANNELS = num_channels[MIC_CHANNEL]
MIC_SAMPLE_SIZE_IN_BITS = 32
MIC_BUFFER_SIZE_IN_BYTES = 4096
WAV_SAMPLE_SIZE_IN_BYTES = WAV_SAMPLE_SIZE_IN_BITS // 8
WAV_BUFFER_SIZE_IN_BYTES = MIC_BUFFER_SIZE_IN_BYTES // MIC_SAMPLE_SIZE_IN_BITS * WAV_SAMPLE_SIZE_IN_BITS
WAV_BYTES_TO_WRITE = RECORD_TIME_IN_SECONDS * SAMPLE_RATE_IN_HZ * WAV_SAMPLE_SIZE_IN_BYTES * NUM_CHANNELS

filename = {(LEFT,16):'/sd/mic_left_16bits.wav',
            (LEFT,32):'/sd/mic_left_32bits.wav',
            (RIGHT,16):'/sd/mic_right_16bits.wav',
            (RIGHT,32):'/sd/mic_right_32bits.wav', 
            (LEFT_RIGHT,16):'/sd/mic_stereo_16bits.wav', 
            (LEFT_RIGHT,32):'/sd/mic_stereo_32bits.wav'} 

"""
# snip_16():  snip 16-bit samples from a 32-bit stereo sample stream
# assumption: I2S configuration for stereo microphone.  e.g. I2S channelformat = STEREO
# example snip:  
#   bytes_in[] =  [0x44, 0x55, 0xAB, 0x77, 0x99, 0xBB, 0x11, 0x22] = [Left channel, Right channel]           
#   bytes_out[] = [ 0xAB, 0x77, 0x11, 0x22] = [Left channel, Right channel]
#   notes:
#       bytes_in[] arranged in little endian format:  
#           0x77 is the most significant byte of the 32-bit sample
#           0x44 is the least significant byte of the 32-bit sample
#              and
#           LEFT Channel = 0x44, 0x55, 0xAB, 0x77
#           RIGHT Channel = 0x99, 0xBB, 0x11, 0x22
#
#       bytes_out[] is arranged in little endian format,
#
# returns:  number of bytes snipped
def snip_16(bytes_in, bytes_out, NUM_CHANNELS):
    num_samples = len(bytes_in) // 4
    for i in range(num_samples):
        bytes_out[2*i + 0] = bytes_in[4*i + 2]
        bytes_out[2*i + 1] = bytes_in[4*i + 3]
            
    return num_samples * 8
"""
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

sck_pin = Pin(13)
ws_pin = Pin(14)
sd_pin = Pin(34)

audio_in = I2S(
    I2S.NUM0, 
    sck=sck_pin, ws=ws_pin, sd=sd_pin,
    mode=I2S.RX,
    bits=MIC_SAMPLE_SIZE_IN_BITS,
    format=I2S.STEREO,
    rate=SAMPLE_RATE_IN_HZ,
)

# configure SD card
#   slot=2 configures SD card to use the SPI3 controller (VSPI), DMA channel = 2
#   slot=3 configures SD card to use the SPI2 controller (HSPI), DMA channel = 1
sd = SDCard(slot=3, sck=Pin(18), mosi=Pin(23), miso=Pin(19), cs=Pin(4))
uos.mount(sd, "/sd")
wav = open(filename[(MIC_CHANNEL, WAV_SAMPLE_SIZE_IN_BITS)],'wb')

# create header for WAV file and write to SD card
wav_header = create_wav_header(
    SAMPLE_RATE_IN_HZ, 
    WAV_SAMPLE_SIZE_IN_BITS, 
    NUM_CHANNELS, 
    SAMPLE_RATE_IN_HZ * RECORD_TIME_IN_SECONDS
)
num_bytes_written = wav.write(wav_header)

# allocate sample arrays
#   memoryview used to reduce heap allocation in while loop
mic_bytes = bytearray(MIC_BUFFER_SIZE_IN_BYTES)
mic_bytes_mv = memoryview(mic_bytes)
wav_bytes = bytearray(WAV_BUFFER_SIZE_IN_BYTES)
wav_bytes_mv = memoryview(wav_bytes)

num_sample_bytes_written_to_wav = 0

print('Starting')
while num_sample_bytes_written_to_wav < WAV_BYTES_TO_WRITE:
    try:
        # try to read a block of samples from the I2S microphone
        # readinto() method returns 0 if no DMA buffer is full
        t0 = time.ticks_us()
        num_bytes_read = audio_in.readinto(mic_bytes_mv)
        #print(time.ticks_diff(time.ticks_us(), t0)/1000)
        #print(num_bytes_read_from_mic)
        if num_bytes_read > 0:
            #if WAV_SAMPLE_SIZE_IN_BITS == 16:
            num_bytes_copied = i2stools.copy(bufin=mic_bytes_mv[:num_bytes_read], 
                                             bufout=wav_bytes_mv, 
                                             channel=channel_map[MIC_CHANNEL], 
                                             format=bit_map[WAV_SAMPLE_SIZE_IN_BITS])
            #num_bytes_snipped = snip_16(mic_bytes_mv[:num_bytes_read], wav_bytes_mv)
            num_bytes_to_write = min(num_bytes_copied, WAV_BYTES_TO_WRITE - num_sample_bytes_written_to_wav)
            num_bytes_written = wav.write(wav_bytes_mv[:num_bytes_to_write])
            """
            else:
                # assume 32 bits    
                num_bytes_to_write = min(num_bytes_read, WAV_BYTES_TO_WRITE - num_sample_bytes_written_to_wav)
                num_bytes_written = wav.write(mic_bytes_mv[:num_bytes_to_write])
                
            """
                
            num_sample_bytes_written_to_wav += num_bytes_written
    except (KeyboardInterrupt, Exception) as e:
        print('caught exception {} {}'.format(type(e).__name__, e))
        break

wav.close()
uos.umount("/sd")
sd.deinit()
audio_in.deinit()
print('Done')
print('%d sample bytes written to WAV file' % num_sample_bytes_written_to_wav)