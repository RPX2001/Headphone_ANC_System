import numpy as np
import matplotlib.pyplot as plt
from scipy import signal as scipy_signal

# Read data from file
filename = 'calibration_data_9.txt'
print(f"Reading data from {filename}...")

with open(filename, 'r') as f:
    data_str = f.read().strip()
    # Parse comma-separated values
    raw_data = np.array([int(x) for x in data_str.split(',')])

print(f"Loaded {len(raw_data)} samples")

# Constants
ADC_RESOLUTION = 4095
SAMPLE_RATE = 10000  # ~10kHz code

# Calculate statistics
mean_val = np.mean(raw_data)
std_val = np.std(raw_data)
peak_to_peak = np.ptp(raw_data)
dc_offset = mean_val - (ADC_RESOLUTION / 2)

print(f"\n=== Statistics ===")
print(f"Mean: {mean_val:.2f}")
print(f"DC Offset from center (2048): {dc_offset:+.2f}")
print(f"Std Dev: {std_val:.2f}")
print(f"Peak-to-Peak: {peak_to_peak:.0f}")
print(f"Min: {np.min(raw_data)}, Max: {np.max(raw_data)}")

# Normalize data (center around 0)
normalized_data = (raw_data - mean_val) / 2048.0

# Time vector
time = np.arange(len(raw_data)) / SAMPLE_RATE * 1000  # in milliseconds

# Create figure with subplots
fig = plt.figure(figsize=(14, 10))

# 1. Raw ADC values
ax1 = plt.subplot(3, 2, 1)
ax1.plot(time, raw_data, 'b-', linewidth=0.8)
ax1.axhline(ADC_RESOLUTION/2, color='r', linestyle='--', alpha=0.6, label='Ideal Center (2048)')
ax1.axhline(mean_val, color='g', linestyle='--', alpha=0.6, label=f'Mean ({mean_val:.1f})')
ax1.fill_between([time[0], time[-1]], ADC_RESOLUTION*0.25, ADC_RESOLUTION*0.75, 
                  color='green', alpha=0.1, label='Good Range')
ax1.set_xlabel('Time (ms)')
ax1.set_ylabel('Raw ADC Value')
ax1.set_title('Raw Microphone ADC Data')
ax1.grid(True, alpha=0.3)
ax1.legend(fontsize=8)

# 2. Normalized signal
ax2 = plt.subplot(3, 2, 2)
ax2.plot(time, normalized_data, 'b-', linewidth=0.8)
ax2.axhline(0, color='r', linestyle='--', alpha=0.6, label='Zero Line')
ax2.set_xlabel('Time (ms)')
ax2.set_ylabel('Normalized Value')
ax2.set_title('Normalized Signal (DC Removed)')
ax2.grid(True, alpha=0.3)
ax2.legend(fontsize=8)

# 3. Histogram of raw values
ax3 = plt.subplot(3, 2, 3)
ax3.hist(raw_data, bins=50, color='skyblue', edgecolor='black', alpha=0.7)
ax3.axvline(mean_val, color='r', linestyle='--', linewidth=2, label=f'Mean: {mean_val:.1f}')
ax3.axvline(ADC_RESOLUTION/2, color='g', linestyle='--', linewidth=2, label='Center: 2048')
ax3.set_xlabel('ADC Value')
ax3.set_ylabel('Frequency')
ax3.set_title('Distribution of Raw ADC Values')
ax3.legend()
ax3.grid(True, alpha=0.3)

# 4. FFT / Frequency spectrum
ax4 = plt.subplot(3, 2, 4)
# Compute FFT
fft_vals = np.fft.rfft(normalized_data)
fft_freq = np.fft.rfftfreq(len(normalized_data), 1/SAMPLE_RATE)
fft_magnitude = np.abs(fft_vals)

# Plot spectrum
ax4.plot(fft_freq, 20*np.log10(fft_magnitude + 1e-10), 'b-', linewidth=1)
ax4.set_xlabel('Frequency (Hz)')
ax4.set_ylabel('Magnitude (dB)')
ax4.set_title('Frequency Spectrum (FFT)')
ax4.set_xlim([0, 2000])  # Focus on 0-2kHz
ax4.grid(True, alpha=0.3)

# Find peak frequency
peak_idx = np.argmax(fft_magnitude[1:]) + 1  # Skip DC component
peak_freq = fft_freq[peak_idx]
ax4.axvline(peak_freq, color='r', linestyle='--', alpha=0.6, 
            label=f'Peak: {peak_freq:.1f} Hz')
ax4.legend(fontsize=8)

print(f"\nDominant frequency: {peak_freq:.1f} Hz")

# 5. Zoomed view (first 100ms)
ax5 = plt.subplot(3, 2, 5)
zoom_samples = min(1000, len(raw_data))  # First 1000 samples or less
ax5.plot(time[:zoom_samples], raw_data[:zoom_samples], 'b-', linewidth=1.5, marker='o', markersize=2)
ax5.axhline(mean_val, color='g', linestyle='--', alpha=0.6, label=f'Mean: {mean_val:.1f}')
ax5.set_xlabel('Time (ms)')
ax5.set_ylabel('Raw ADC Value')
ax5.set_title(f'Zoomed View (First {zoom_samples} samples)')
ax5.grid(True, alpha=0.3)
ax5.legend(fontsize=8)

# 6. Spectrogram
ax6 = plt.subplot(3, 2, 6)
f, t_spec, Sxx = scipy_signal.spectrogram(normalized_data, SAMPLE_RATE, nperseg=256)
im = ax6.pcolormesh(t_spec * 1000, f, 10*np.log10(Sxx + 1e-10), shading='gouraud', cmap='viridis')
ax6.set_ylabel('Frequency (Hz)')
ax6.set_xlabel('Time (ms)')
ax6.set_title('Spectrogram')
ax6.set_ylim([0, 2000])
plt.colorbar(im, ax=ax6, label='Power (dB)')

plt.tight_layout()

# Summary text box
summary_text = f"""
Sample Count: {len(raw_data)}
Duration: {len(raw_data)/SAMPLE_RATE*1000:.1f} ms
Mean ADC: {mean_val:.2f}
DC Offset: {dc_offset:+.2f}
Std Dev: {std_val:.2f}
Peak-to-Peak: {peak_to_peak}
Dominant Freq: {peak_freq:.1f} Hz
"""

fig.text(0.02, 0.02, summary_text, fontsize=9, family='monospace',
         bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

plt.savefig('calibration_data_analysis.png', dpi=150, bbox_inches='tight')
print(f"\nPlot saved as 'calibration_data_analysis.png'")

plt.show()
