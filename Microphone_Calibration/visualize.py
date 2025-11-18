import matplotlib.pyplot as plt
import os

# ---------------------------------------------------------
# 1. SETUP: Create a dummy file for demonstration
# (You can skip this part if you already have your real file)
# ---------------------------------------------------------
filename = "calibration_data_1.txt"

# ---------------------------------------------------------
# 2. PARSING: Read the file and extract data
# ---------------------------------------------------------
signals = []
raw_values = []

try:
    with open(filename, "r") as f:
        lines = f.readlines()
        
    for i, line in enumerate(lines):
        line = line.strip()
        if not line: continue # Skip empty lines
        
        # Parse format: "Signal: -0.1952 | Raw: 592"
        try:
            # Split into ["Signal: -0.1952 ", " Raw: 592"]
            parts = line.split(',')
            
            # Extract the number after the colon for Signal
            sig_val = float(parts[0])
            
            # Extract the number after the colon for Raw
            raw_val = int(parts[1])
            
            signals.append(sig_val)
            raw_values.append(raw_val)
            
        except (ValueError, IndexError) as e:
            print(f"Skipping malformed line {i+1}: '{line}' - Error: {e}")

except FileNotFoundError:
    print(f"Error: File '{filename}' not found.")
except Exception as e:
    print(f"Error reading file: {e}")

# ---------------------------------------------------------
# 3. PLOTTING: Create the 2 plots
# ---------------------------------------------------------
if not signals:
    print("No data found to plot.")
else:
    # Create a figure with 2 subplots (stacked vertically)
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    # Plot 1: Normalized Signal
    ax1.plot(signals, color='#1f77b4', marker='o', linestyle='-', linewidth=2, label='Signal')
    ax1.set_title('Normalized Signal Input', fontsize=14)
    ax1.set_ylabel('Amplitude (Normalized)', fontsize=12)
    ax1.grid(True, linestyle='--', alpha=0.7)
    ax1.axhline(0, color='black', linewidth=1) # Add a zero line for reference

    # Plot 2: Raw Values
    ax2.plot(raw_values, color='#ff7f0e', marker='s', linestyle='-', linewidth=2, label='Raw')
    ax2.set_title('Raw Microphone Input', fontsize=14)
    ax2.set_xlabel('Sample Index', fontsize=12)
    ax2.set_ylabel('Raw Value (Int)', fontsize=12)
    ax2.grid(True, linestyle='--', alpha=0.7)

    # Adjust layout to prevent overlapping
    plt.tight_layout()

    # Show the plot
    print("Displaying plots...")
    plt.show()