# The MIT License (MIT)
# Copyright (c) 2020 Mike Teachman
# https://opensource.org/licenses/MIT

# Purpose: Read audio samples from SD card and write them to an I2S DAC
#
# Sample WAV files in examples/i2s/sound_files folder:
#   "music-16k-16bits-mono.wav"
#   "music-16k-32bits-mono.wav"
#   "music-16k-16bits-stereo.wav"
#   "music-16k-32bits-stereo.wav"
#
# Hardware tested:
# - MAX98357A mono amplifier module (Adafruit I2S 3W Class D Amplifier Breakout)
# - PCM5102 stereo DAC module
#
# The WAV file will play continuously until a keyboard interrupt is detected or
# the ESP32 is reset
  
import uos
from machine import Pin
from machine import SDCard
from machine import I2S

MONO = 0
STEREO = 1

format_map = {MONO:I2S.MONO, STEREO:I2S.STEREO}

#======= USER CONFIGURATION =======
SAMPLE_RATE_IN_HZ = 16000
FORMAT = STEREO
WAV_SAMPLE_SIZE_IN_BITS = 16
WAV_FILE = "music-16k-16bits-stereo.wav"
#======= USER CONFIGURATION =======

sck_pin = Pin(33) 
ws_pin = Pin(25)  
sd_pin = Pin(32)

audio_out = I2S(
    I2S.NUM0, 
    sck=sck_pin, ws=ws_pin, sd=sd_pin, 
    mode=I2S.TX,
    bits=WAV_SAMPLE_SIZE_IN_BITS, 
    format=format_map[FORMAT],
    rate=SAMPLE_RATE_IN_HZ)

# configure SD card
#   slot=2 configures SD card to use the SPI3 controller (VSPI), DMA channel = 2
#   slot=3 configures SD card to use the SPI2 controller (HSPI), DMA channel = 1
sd = SDCard(slot=3, sck=Pin(18), mosi=Pin(23), miso=Pin(19), cs=Pin(4))
uos.mount(sd, "/sd")
wav_file = '/sd/{}'.format(WAV_FILE)
wav = open(wav_file,'rb')

# advance to first byte of Data section in WAV file
pos = wav.seek(44) 

# allocate sample arrays
#   memoryview used to reduce heap allocation in while loop
wav_samples = bytearray(4096)
wav_samples_mv = memoryview(wav_samples)

print('Starting')
# continuously read audio samples from the WAV file 
# and write them to an I2S DAC
while True:
    try:
        num_read = wav.readinto(wav_samples_mv)
        num_written = 0
        # end of WAV file?
        if num_read == 0:
            # advance to first byte of Data section
            pos = wav.seek(44) 
        else:
            # loop until all samples are written to the I2S peripheral
            while num_written < num_read:
                num_written += audio_out.write(wav_samples_mv[num_written:num_read])
    except (KeyboardInterrupt, Exception) as e:
        print('caught exception {} {}'.format(type(e).__name__, e))
        break
    
wav.close()
uos.umount("/sd")
sd.deinit()
audio_out.deinit()
print('Done')