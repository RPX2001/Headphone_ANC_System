import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import random # Replaced with serial for actual use

# ---------------------------------------------------------
# CONFIGURATION
# ---------------------------------------------------------
# If using real Serial, set USE_DUMMY_DATA = False
USE_DUMMY_DATA = False 
SERIAL_PORT = 'COM8'   # Change to your port (e.g., 'COM3' or '/dev/ttyUSB0')
BAUD_RATE = 250000
ADC_MAX = 4095         # Set to 1023 for Arduino Uno, 4095 for ESP32/STM32

# ---------------------------------------------------------
# SETUP
# ---------------------------------------------------------
if not USE_DUMMY_DATA:
    import serial
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
    except:
        print("Could not open Serial port. Switching to dummy data.")
        USE_DUMMY_DATA = True

# Data storage
x_len = 200  # Number of points to display
y_range = [0, ADC_MAX + 100]
data = deque([ADC_MAX/2] * x_len, maxlen=x_len)

# Create figure
fig, ax = plt.subplots()
line, = ax.plot([], [], lw=2)
ax.set_ylim(y_range)
ax.set_xlim(0, x_len-1)
ax.set_title(f'Microphone Bias Calibration\nGoal: Center the wave in the green zone')
ax.set_ylabel('Raw ADC Value')
ax.grid(True, alpha=0.3)

# Add "Safe Zone" visual
# Ideally, the wave should center around ADC_MAX / 2
center = ADC_MAX / 2
safe_min = ADC_MAX * 0.1
safe_max = ADC_MAX * 0.9
plt.axhspan(safe_min, safe_max, color='green', alpha=0.1, label='Safe Dynamic Range')
plt.axhline(center, color='red', linestyle='--', alpha=0.5, label='Ideal Center')
plt.legend(loc='upper right')

def get_data():
    """Fetch data from Serial or generate dummy data for testing."""
    if USE_DUMMY_DATA:
        # Simulates the user's current issue: Data centered too low (at 800)
        # causing clipping at 0.
        import math
        import time
        t = time.time() * 10
        # Wave centered at 800, amplitude 1200. 
        # This causes it to go from 2000 down to -400 (clipped to 0)
        val = 800 + 1200 * math.sin(t)
        return max(0, min(ADC_MAX, int(val)))
    else:
        try:
            if ser.in_waiting:
                line_str = ser.readline().decode('utf-8').strip()
                if not line_str: return None
                
                # Expected format: "signal_val, raw_val"
                # We only need raw_val for calibration
                parts = line_str.split(',') 
                
                # Try to find the raw integer value
                for p in parts:
                    if p.strip().isdigit():
                        return int(p)
                return None
        except:
            return None

def update(frame, data):
    val = get_data()
    if val is not None:
        data.append(val)
    
    line.set_data(range(len(data)), data)
    return line,

# Start Animation
ani = animation.FuncAnimation(fig, update, fargs=(data,), interval=20, blit=True)
plt.show()