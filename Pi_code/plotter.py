import numpy as np
import matplotlib.pyplot as plt
from scipy.io import wavfile

filename = "Test_recordings/test2/audio_1.wav"

sample_rate, data = wavfile.read(filename)

if data.ndim != 2 or data.shape[1] != 2:
    raise ValueError("The audio file is not stereo.")

left_channel = data[:, 0]
right_channel = data[:, 1]

time = np.arange(len(data)) / sample_rate

fig, axes = plt.subplots(2, 1, sharex=True, sharey=True, figsize=(12, 6))

axes[0].plot(time, left_channel)
axes[0].set_title("Left Channel")
axes[0].set_ylabel("Amplitude")
axes[0].grid(True)

axes[1].plot(time, right_channel)
axes[1].set_title("Right Channel")
axes[1].set_ylabel("Amplitude")
axes[1].set_xlabel("Time (s)")
axes[1].grid(True)

plt.tight_layout()
plt.show()