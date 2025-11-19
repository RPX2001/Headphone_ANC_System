import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import serial
import serial.tools.list_ports
import time
import numpy as np

# ==========================================
# CONFIGURATION (EDIT THESE)
# ==========================================
SERIAL_PORT = 'COM8'      # Windows: 'COM3', Mac/Linux: '/dev/ttyUSB0'
BAUD_RATE = 250000        # Must match your Arduino/ESP code exactly
ADC_RESOLUTION = 4095     # 4095 for ESP32/STM32, 1023 for Arduino Uno
# ==========================================

# Auto-detect port if the user forgot to set it
if SERIAL_PORT == 'COM8':
    ports = list(serial.tools.list_ports.comports())
    if ports:
        print("Available ports found:")
        for p in ports:
            print(f" - {p.device}")

print(f"Attempting to connect to {SERIAL_PORT} at {BAUD_RATE}...")

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    time.sleep(2) # Give connection time to settle
    print("Connected!")
except Exception as e:
    print(f"\nERROR: Could not open serial port {SERIAL_PORT}.")
    print(f"Details: {e}")
    print("Please check your USB cable and ensure the 'SERIAL_PORT' variable matches your device.")
    exit()

# Setup dual plot (Raw + Normalized)
x_len = 500
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))

# Raw ADC data
raw_data = deque([ADC_RESOLUTION/2] * x_len, maxlen=x_len)
line_raw, = ax1.plot([], [], color='#ff7f0e', lw=1.5, label='Raw ADC')

# Normalized signal data
norm_data = deque([0.0] * x_len, maxlen=x_len)
line_norm, = ax2.plot([], [], color='#1f77b4', lw=1.5, label='Normalized Signal')

# Configure Raw ADC plot
ax1.set_ylim([0, ADC_RESOLUTION])
ax1.set_xlim([0, x_len-1])
ax1.set_title('Microphone DC Bias Calibration - Raw ADC Value', fontsize=12, fontweight='bold')
ax1.set_ylabel('Raw ADC Value', fontsize=10)
ax1.grid(True, alpha=0.3)
ax1.axhline(ADC_RESOLUTION/2, color='red', linestyle='--', alpha=0.6, linewidth=1, label='Ideal Center (2048)')
ax1.axhspan(ADC_RESOLUTION * 0.25, ADC_RESOLUTION * 0.75, color='green', alpha=0.1, label='Good Range')
ax1.legend(loc='upper right', fontsize=8)

# Text annotations for statistics
raw_mean_text = ax1.text(0.02, 0.95, '', transform=ax1.transAxes, 
                         fontsize=10, verticalalignment='top',
                         bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

# Configure Normalized Signal plot
ax2.set_ylim([-1.2, 1.2])
ax2.set_xlim([0, x_len-1])
ax2.set_title('Normalized Signal (After DC Removal)', fontsize=12, fontweight='bold')
ax2.set_ylabel('Normalized Value', fontsize=10)
ax2.set_xlabel('Sample Index', fontsize=10)
ax2.grid(True, alpha=0.3)
ax2.axhline(0, color='red', linestyle='--', alpha=0.6, linewidth=1, label='Zero Line')
ax2.axhspan(-1.0, 1.0, color='green', alpha=0.1, label='Valid Range')
ax2.legend(loc='upper right', fontsize=8)

# Text annotations for normalized signal
norm_stats_text = ax2.text(0.02, 0.95, '', transform=ax2.transAxes,
                           fontsize=10, verticalalignment='top',
                           bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.5))

plt.tight_layout()

def get_real_data():
    try:
        if ser.in_waiting:
            line_str = ser.readline().decode('utf-8', errors='ignore').strip()
            
            if not line_str or '>>>' in line_str or '===' in line_str:
                return None, None
            
            # Parse format: "signal,raw"
            parts = line_str.split(',')
            if len(parts) >= 2:
                try:
                    signal = float(parts[0].strip())
                    raw = float(parts[1].strip())
                    return signal, raw
                except ValueError:
                    return None, None
            return None, None
    except Exception:
        return None, None

frame_count = 0

def update(frame):
    global frame_count
    frame_count += 1
    
    # Fetch multiple points per frame for smoother plotting
    points_read = 0
    while points_read < 20:  # Read up to 20 points per frame
        signal, raw = get_real_data()
        if signal is not None and raw is not None:
            raw_data.append(raw)
            norm_data.append(signal)
            points_read += 1
        else:
            break
    
    # Update plots
    line_raw.set_data(range(len(raw_data)), raw_data)
    line_norm.set_data(range(len(norm_data)), norm_data)
    
    # Update statistics every 10 frames (~200ms)
    if frame_count % 10 == 0 and len(raw_data) > 0:
        raw_array = np.array(raw_data)
        norm_array = np.array(norm_data)
        
        raw_mean = np.mean(raw_array)
        raw_std = np.std(raw_array)
        raw_pp = np.ptp(raw_array)  # peak-to-peak
        
        norm_mean = np.mean(norm_array)
        norm_std = np.std(norm_array)
        norm_pp = np.ptp(norm_array)
        
        # Update raw stats
        dc_offset = raw_mean - (ADC_RESOLUTION/2)
        raw_mean_text.set_text(
            f'Mean: {raw_mean:.1f} (DC offset: {dc_offset:+.1f})\n'
            f'Std Dev: {raw_std:.1f}\n'
            f'Peak-to-Peak: {raw_pp:.1f}'
        )
        
        # Update normalized stats
        norm_stats_text.set_text(
            f'Mean: {norm_mean:.4f}\n'
            f'Std Dev: {norm_std:.4f}\n'
            f'Peak-to-Peak: {norm_pp:.4f}\n'
            f'1kHz amplitude: ~{norm_pp/2:.4f}'
        )
    
    return line_raw, line_norm

ani = animation.FuncAnimation(fig, update, interval=20, blit=True, cache_frame_data=False)
print("\nPlotting live data...")
print("Instructions:")
print("1. Play a 1kHz sine wave into the microphone")
print("2. Watch the Raw ADC plot - it should oscillate around 2048")
print("3. The DC offset will auto-adjust every 5 seconds")
print("4. Normalized signal should oscillate around 0.0")
print("\nClose the window to stop.")
plt.show()
ser.close()
