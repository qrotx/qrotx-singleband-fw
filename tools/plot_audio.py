#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2025-2026 Christian Riesch, christian@riesch.at
#
# plot_audio.py — Load, filter, and plot the raw audio captured by capture_audio.py.
#
# The input file contains 10-sample frames at 20 kHz; each sample is a signed
# 16-bit little-endian word (the upper 16 bits of the Q31 FIR-decimation output).
#
# Usage:
#   python tools/plot_audio.py          # reads audio.bin in the current directory

import numpy as np
import matplotlib.pyplot as plt
import soundfile as sf
import scipy.signal as sig

# Read binary data as signed 16-bit little-endian (matches firmware i16 output)
data = np.fromfile('audio.bin', dtype='<i2')
print(f"Read {len(data)} samples from binary file.")

# Plot the raw data
plt.figure()
plt.plot(data)
plt.xlabel('Sample Index')
plt.ylabel('Value')
plt.title('Raw data from binary file')
plt.grid(True)
plt.show()

# Trim first/last 0.5 s to discard capture-start transients, convert to float
audio_data = data.astype(np.float32)[10000:-10000]

mean = np.mean(audio_data)
swing = (np.max(audio_data) - np.min(audio_data)) / 2
print(f"Mean: {mean}, Swing: {swing}")

# Scale to roughly [-1, 1] assuming ADC mid-scale ≈ 2048 and swing ≈ 2048
# (ADC is 12-bit unipolar; Q31 conversion shifts by 16, so i16 range ≈ 0..4095)
scaled_data = audio_data / 4069.0 * 2

simple_data = (scaled_data - np.mean(scaled_data)) / np.max(np.abs(scaled_data - np.mean(scaled_data)))

fs = 20e3      # Sample frequency
fn = fs / 2    # Nyquist frequency
f_low = 200
f_high = 2800
order = 4
sos_hp = sig.cheby1(order, .3, f_low / fn, btype='highpass', output='sos')
sos_lp = sig.cheby1(order, .3, f_high / fn, btype='lowpass', output='sos')

hp_data = sig.sosfilt(sos_hp, scaled_data)
lp_data = sig.sosfilt(sos_lp, hp_data)
output = lp_data[2000:]  # skip filter transient (~100 ms)

output_wav_file = 'output.wav'
sampling_rate = 20000  # 20 kSamples/s
sf.write(output_wav_file, output, sampling_rate)

# Plot the filtered output
plt.figure()
plt.plot(output)
plt.xlabel('Sample Index')
plt.ylabel('Value')
plt.title('Output data')
plt.grid(True)
plt.show()

# Compute and plot FFT of raw audio (pre-filter)
fft_output = np.fft.fft(audio_data)
fft_freq = np.fft.fftfreq(len(audio_data), 1/sampling_rate)

# Only plot positive frequencies
positive_freq_idx = fft_freq > 0
fft_magnitude = np.abs(fft_output[positive_freq_idx])
fft_freq_positive = fft_freq[positive_freq_idx]

plt.figure()
plt.plot(fft_freq_positive, 20 * np.log10(fft_magnitude))
plt.xlabel('Frequency (Hz)')
plt.ylabel('Magnitude (dB)')
plt.title('FFT of Output Signal')
plt.grid(True)
plt.show()
