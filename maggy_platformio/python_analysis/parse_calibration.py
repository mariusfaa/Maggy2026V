import re
import sys
import matplotlib.pyplot as plt
import numpy as np

def parse_calibration_data(lines):
    """Parse calibration data from serial output lines."""
    data = []
    in_calibration = False

    for line in lines:
        line = line.strip()
        if line == "START_CALIBRATION_DATA":
            in_calibration = True
            continue
        if line == "END_CALIBRATION_DATA":
            in_calibration = False
            continue

        if in_calibration and line.startswith('t:'):
            matches = re.findall(r'(\w+):(-?[\d.]+)', line)
            if matches:
                values = {k: float(v) for k, v in matches}
                data.append(values)

    return data

def load_from_file(filename):
    """Load data from a saved file."""
    with open(filename, 'r') as f:
        return f.readlines()

def load_from_serial(port, baudrate=115200, timeout=120):
    """Load data directly from serial port."""
    import serial

    print(f"Connecting to {port} at {baudrate} baud...")
    ser = serial.Serial(port, baudrate, timeout=1)

    lines = []
    in_calibration = False

    print("Waiting for calibration data...")
    start_time = None

    while True:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line:
            print(line)
            lines.append(line)

            if line == "START_CALIBRATION_DATA":
                in_calibration = True
                start_time = __import__('time').time()
                print("\n>>> Calibration started <<<\n")
            elif line == "END_CALIBRATION_DATA":
                print("\n>>> Calibration complete <<<\n")
                break

        if start_time and (__import__('time').time() - start_time > timeout):
            print("Timeout waiting for calibration data")
            break

    ser.close()
    return lines

def plot_calibration(data):
    """Plot calibration data."""
    if not data:
        print("No data to plot!")
        return

    # Convert to numpy arrays for easier manipulation
    t = np.array([d['t'] for d in data])
    t = (t - t[0]) / 1e6  # Convert to seconds from start

    bx = np.array([d['bx'] for d in data])
    by = np.array([d['by'] for d in data])
    bz = np.array([d['bz'] for d in data])

    i1 = np.array([d['i1'] for d in data])
    i2 = np.array([d['i2'] for d in data])
    i3 = np.array([d['i3'] for d in data])
    i4 = np.array([d['i4'] for d in data])

    sol = np.array([int(d['sol']) for d in data])
    sp = np.array([d['sp'] for d in data])

    solenoid_names = ['X+ (sol 0)', 'X- (sol 1)', 'Y+ (sol 2)', 'Y- (sol 3)']
    currents = [i1, i2, i3, i4]

    # Figure 1: Time series overview
    fig1, axes1 = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    fig1.suptitle('Calibration Time Series')

    axes1[0].plot(t, bx, label='Bx', alpha=0.8)
    axes1[0].plot(t, by, label='By', alpha=0.8)
    axes1[0].plot(t, bz, label='Bz', alpha=0.8)
    axes1[0].set_ylabel('Magnetic Field (mT)')
    axes1[0].legend()
    axes1[0].grid(True, alpha=0.3)

    axes1[1].plot(t, i1, label='I1 (X+)', alpha=0.8)
    axes1[1].plot(t, i2, label='I2 (X-)', alpha=0.8)
    axes1[1].plot(t, i3, label='I3 (Y+)', alpha=0.8)
    axes1[1].plot(t, i4, label='I4 (Y-)', alpha=0.8)
    axes1[1].set_ylabel('Current (A)')
    axes1[1].legend()
    axes1[1].grid(True, alpha=0.3)

    axes1[2].plot(t, sp, label='Setpoint', alpha=0.8)
    axes1[2].plot(t, sol, label='Active Solenoid', alpha=0.8)
    axes1[2].set_ylabel('Setpoint / Solenoid')
    axes1[2].set_xlabel('Time (s)')
    axes1[2].legend()
    axes1[2].grid(True, alpha=0.3)

    plt.tight_layout()

    # Figure 2: Per-solenoid B-field vs current
    fig2, axes2 = plt.subplots(2, 2, figsize=(12, 10))
    fig2.suptitle('Magnetic Field vs Measured Current (per solenoid)')

    for sol_idx in range(4):
        ax = axes2[sol_idx // 2, sol_idx % 2]
        mask = sol == sol_idx

        if not np.any(mask):
            ax.set_title(f'{solenoid_names[sol_idx]} - No data')
            continue

        current = currents[sol_idx][mask]

        ax.plot(current, bx[mask], 'r.', label='Bx', alpha=0.5, markersize=2)
        ax.plot(current, by[mask], 'g.', label='By', alpha=0.5, markersize=2)
        ax.plot(current, bz[mask], 'b.', label='Bz', alpha=0.5, markersize=2)

        ax.set_xlabel(f'Current I{sol_idx+1} (A)')
        ax.set_ylabel('Magnetic Field (mT)')
        ax.set_title(solenoid_names[sol_idx])
        ax.legend()
        ax.grid(True, alpha=0.3)

    plt.tight_layout()

    # Figure 3: B-field vs setpoint
    fig3, axes3 = plt.subplots(2, 2, figsize=(12, 10))
    fig3.suptitle('Magnetic Field vs Setpoint (per solenoid)')

    for sol_idx in range(4):
        ax = axes3[sol_idx // 2, sol_idx % 2]
        mask = sol == sol_idx

        if not np.any(mask):
            ax.set_title(f'{solenoid_names[sol_idx]} - No data')
            continue

        ax.plot(sp[mask], bx[mask], 'r.', label='Bx', alpha=0.5, markersize=2)
        ax.plot(sp[mask], by[mask], 'g.', label='By', alpha=0.5, markersize=2)
        ax.plot(sp[mask], bz[mask], 'b.', label='Bz', alpha=0.5, markersize=2)

        ax.set_xlabel('Setpoint')
        ax.set_ylabel('Magnetic Field (mT)')
        ax.set_title(solenoid_names[sol_idx])
        ax.legend()
        ax.grid(True, alpha=0.3)

    plt.tight_layout()

    plt.show()

def main():
    if len(sys.argv) < 2:
        print("Usage:")
        print(f"  {sys.argv[0]} <filename>     - Parse from saved file")
        print(f"  {sys.argv[0]} --serial PORT  - Read from serial port")
        print(f"\nExample:")
        print(f"  {sys.argv[0]} calibration.txt")
        print(f"  {sys.argv[0]} --serial COM3")
        sys.exit(1)

    if sys.argv[1] == '--serial':
        if len(sys.argv) < 3:
            print("Error: Please specify serial port")
            sys.exit(1)
        port = sys.argv[2]
        lines = load_from_serial(port)

        # Save raw data for later analysis
        with open('calibration_raw.txt', 'w') as f:
            f.write('\n'.join(lines))
        print("Raw data saved to calibration_raw.txt")
    else:
        lines = load_from_file(sys.argv[1])

    data = parse_calibration_data(lines)
    print(f"Parsed {len(data)} data points")

    if data:
        plot_calibration(data)

if __name__ == '__main__':
    main()
