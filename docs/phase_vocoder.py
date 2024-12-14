import wave
import numpy as np

sound = wave.open('all_pedalboard_samples/A1.wav', 'rb')
nframes = sound.getnframes()
frames = sound.readframes(nframes=nframes)

num_channels = sound.getnchannels()
sample_width = sound.getsampwidth()
frame_rate = sound.getframerate()
num_frames = sound.getnframes()

stft_len = 50
hop_len = 


print(frames)
exit(1)

# Write the frames to a WAV file
with wave.open('output_test.wav', 'wb') as wav_file:
    wav_file.setnchannels(num_channels)
    wav_file.setsampwidth(sample_width)
    wav_file.setframerate(frame_rate)
    wav_file.writeframes(frames)

