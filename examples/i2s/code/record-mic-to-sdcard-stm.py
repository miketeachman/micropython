import pyb
import time
import uos
from machine import I2S
from machine import Pin

# TODO - correct example taking into account that Mics should have 32 SCK cycles per channel 
# Use itools or new built in functionality?
# perhaps standardize on putting out 32 SCK cycles/word, and then the bits argument indicates how many samples 
# you want to store.  TBD....
# "There must be 64 SCK cycles in each WS stereo frame, or 32 SCK cycles per data-word"

num_channels = {I2S.MONO:1, I2S.STEREO:2}

def i2s_callback(s):
    pass

# for the Pyboard D hardware enable external 3.3v output 
if uos.uname().machine.find('PYBD') == 0:
    pyb.Pin('EN_3V3').on()
    uos.mount(pyb.SDCard(), '/sd')

#======= USER CONFIGURATION =======
WAV_FILE = 'test.wav'
WAV_SAMPLE_SIZE_IN_BITS = 32
FORMAT = I2S.MONO
SAMPLE_RATE_IN_HZ = 16000
RECORD_TIME_IN_SECONDS = 10
#======= USER CONFIGURATION =======

NUM_CHANNELS = num_channels[FORMAT]


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


#     SCK - Y6  (SPI2 SCK)
#     WS -  Y5  (SPI2 NSS)
#     SD -  Y8  (SPI2 MOSI)
sck_mic_pin = Pin('Y6') 
ws_mic_pin = Pin('Y5')  
sd_mic_pin = Pin('Y8')


buf_1 = bytearray(1024)
buf_2 = bytearray(1024)
buf_3 = bytearray(1024)
buf_4 = bytearray(1024)
buf_5 = bytearray(1024)

# TODO  define with memoryview?  see what happens with allocation in loop below.  GC?

audio_in = I2S(
    2, # TODO add constant for this
    sck=sck_mic_pin, ws=ws_mic_pin, sd=sd_mic_pin, 
    mode=I2S.RX,
    bits=WAV_SAMPLE_SIZE_IN_BITS,
    format=FORMAT,
    rate=SAMPLE_RATE_IN_HZ,
    buffers = [buf_1, buf_2, buf_3, buf_4, buf_5],
    callback=i2s_callback)


#     SCK - W29 (SPI1 SCK)
#     WS -  W16 (SPI1 NSS)
#     SD -  Y4  (SPI1 MOSI)
sck_pin = Pin('W29') 
ws_pin = Pin('W16')  
sd_pin = Pin('Y4')

buf_6 = bytearray(1024)
buf_7 = bytearray(1024)
buf_8 = bytearray(1024)
buf_9 = bytearray(1024)
buf_10 = bytearray(1024)

audio_out = I2S(
    1, # TODO add constant for this
    sck=sck_pin, ws=ws_pin, sd=sd_pin, 
    mode=I2S.TX,
    bits=WAV_SAMPLE_SIZE_IN_BITS,
    format=FORMAT,
    rate=SAMPLE_RATE_IN_HZ,
    buffers = [buf_6, buf_7, buf_8, buf_9, buf_10],
    callback=i2s_callback)

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
isStarted = False
num_sample_bytes_written_to_wav = 0

print('==========  START RECORDING ==========')
while num_sample_bytes_written_to_wav < SAMPLE_RATE_IN_HZ * RECORD_TIME_IN_SECONDS * NUM_CHANNELS * (WAV_SAMPLE_SIZE_IN_BITS // 8):
    try:
        if isStarted == False:
            audio_in.start()
            isStarted = True

        buffer = audio_in.getbuffer()
        
        if buffer != None:
            # can write to WAV file when buffer is flagged as being FULL
            # - use callback to indicate this?
            num_bytes_written = wav.write(buffer)
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
if uos.uname().machine.find('PYBD') == 0:
    uos.umount("/sd")
audio_in.deinit()
audio_out.deinit()
print('Done')