
import sys
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from scipy.optimize import least_squares
from scipy.signal import butter, filtfilt


def lowpass_filter(data, cutoff_hz, fs, order=2):
    """
    Apply Butterworth low-pass filter.

    Parameters:
        data: Input signal
        cutoff_hz: Cutoff frequency in Hz
        fs: Sampling frequency in Hz
        order: Filter order

    Returns:
        Filtered signal
    """
    nyq = 0.5 * fs
    normalized_cutoff = cutoff_hz / nyq
    if normalized_cutoff >= 1:
        normalized_cutoff = 0.99  # Clamp to valid range
    b, a = butter(order, normalized_cutoff, btype='low')
    return filtfilt(b, a, data)


def filter_magnetic_field(df, cutoff_hz=50.0):
    """
    Apply low-pass filter to bx, by, bz to reduce Gaussian noise.

    Parameters:
        df: DataFrame with bx, by, bz and time_s columns
        cutoff_hz: Cutoff frequency in Hz

    Returns:
        bx_filt, by_filt, bz_filt: Filtered signals
    """
    # Estimate sampling frequency from data
    dt_median = np.median(np.diff(df['time_s'].values))
    fs = 1.0 / dt_median if dt_median > 0 else 1000.0

    print(f"Estimated sampling frequency: {fs:.1f} Hz")
    print(f"Applying low-pass filter with cutoff: {cutoff_hz} Hz")

    bx_filt = lowpass_filter(df['bx'].values, cutoff_hz, fs)
    by_filt = lowpass_filter(df['by'].values, cutoff_hz, fs)
    bz_filt = lowpass_filter(df['bz'].values, cutoff_hz, fs)

    return bx_filt, by_filt, bz_filt


def compute_mag_derivatives(df):
    """
    Compute time derivatives of magnetic field bx, by, bz.

    Returns:
        dbx, dby, dbz, d2bx, d2by, d2bz: First and second derivatives
    """
    time_s = df['time_s'].values

    # Use diff for more robust derivative (handles non-uniform spacing)
    dt = np.diff(time_s)
    dt = np.where(dt == 0, 1e-6, dt)  # Replace zero dt with small value

    # First derivatives using central difference where possible
    dbx = np.zeros(len(df))
    dby = np.zeros(len(df))
    dbz = np.zeros(len(df))

    # Forward difference for first point
    dbx[0] = (df['bx'].iloc[1] - df['bx'].iloc[0]) / dt[0]
    dby[0] = (df['by'].iloc[1] - df['by'].iloc[0]) / dt[0]
    dbz[0] = (df['bz'].iloc[1] - df['bz'].iloc[0]) / dt[0]

    # Central difference for middle points
    for i in range(1, len(df) - 1):
        dt_avg = (dt[i-1] + dt[i]) / 2
        if dt_avg > 0:
            dbx[i] = (df['bx'].iloc[i+1] - df['bx'].iloc[i-1]) / (2 * dt_avg)
            dby[i] = (df['by'].iloc[i+1] - df['by'].iloc[i-1]) / (2 * dt_avg)
            dbz[i] = (df['bz'].iloc[i+1] - df['bz'].iloc[i-1]) / (2 * dt_avg)

    # Backward difference for last point
    dbx[-1] = (df['bx'].iloc[-1] - df['bx'].iloc[-2]) / dt[-1]
    dby[-1] = (df['by'].iloc[-1] - df['by'].iloc[-2]) / dt[-1]
    dbz[-1] = (df['bz'].iloc[-1] - df['bz'].iloc[-2]) / dt[-1]

    # Second derivatives
    d2bx = np.zeros(len(df))
    d2by = np.zeros(len(df))
    d2bz = np.zeros(len(df))

    for i in range(1, len(df) - 1):
        dt_avg = (dt[i-1] + dt[i]) / 2
        if dt_avg > 0:
            d2bx[i] = (dbx[i+1] - dbx[i-1]) / (2 * dt_avg)
            d2by[i] = (dby[i+1] - dby[i-1]) / (2 * dt_avg)
            d2bz[i] = (dbz[i+1] - dbz[i-1]) / (2 * dt_avg)

    # Replace any NaN/Inf with 0
    dbx = np.nan_to_num(dbx, nan=0.0, posinf=0.0, neginf=0.0)
    dby = np.nan_to_num(dby, nan=0.0, posinf=0.0, neginf=0.0)
    dbz = np.nan_to_num(dbz, nan=0.0, posinf=0.0, neginf=0.0)
    d2bx = np.nan_to_num(d2bx, nan=0.0, posinf=0.0, neginf=0.0)
    d2by = np.nan_to_num(d2by, nan=0.0, posinf=0.0, neginf=0.0)
    d2bz = np.nan_to_num(d2bz, nan=0.0, posinf=0.0, neginf=0.0)

    return dbx, dby, dbz, d2bx, d2by, d2bz


def identify_state_space(df):
    """
    Identify state-space model: xdot = A*x + B*u

    State vector: x = [bx, by, bz, dbx, dby, dbz]
    Input vector: u = [i1, i2, i3, i4]
    State derivative: xdot = [dbx, dby, dbz, d2bx, d2by, d2bz]

    Solves for A (6x6) and B (6x4) using least squares.

    Returns:
        A, B matrices and residual info
    """
    # Compute derivatives
    dbx, dby, dbz, d2bx, d2by, d2bz = compute_mag_derivatives(df)

    # Build state matrix X: each row is [bx, by, bz, dbx, dby, dbz]
    X = np.column_stack([
        df['bx'].values,
        df['by'].values,
        df['bz'].values,
        dbx, dby, dbz
    ])

    # Build input matrix U: each row is [i1, i2, i3, i4]
    U = np.column_stack([
        df['i1'].values,
        df['i2'].values,
        df['i3'].values,
        df['i4'].values
    ])

    # Build state derivative matrix Xdot: each row is [dbx, dby, dbz, d2bx, d2by, d2bz]
    Xdot = np.column_stack([dbx, dby, dbz, d2bx, d2by, d2bz])

    # Solve xdot = A*x + B*u  =>  xdot = [X, U] @ [A.T; B.T]
    # Rearrange as: xdot = Phi @ theta, where Phi = [X, U], theta = [A.T, B.T]
    Phi = np.column_stack([X, U])  # N x 10

    # Solve for each row of [A, B] independently using least squares
    n_states = 6
    n_inputs = 4
    A = np.zeros((n_states, n_states))
    B = np.zeros((n_states, n_inputs))

    residuals = []
    for i in range(n_states):
        # Solve: Xdot[:, i] = Phi @ theta_i
        theta_i, res, rank, s = np.linalg.lstsq(Phi, Xdot[:, i], rcond=None)
        A[i, :] = theta_i[:n_states]
        B[i, :] = theta_i[n_states:]
        if len(res) > 0:
            residuals.append(res[0])
        else:
            residuals.append(np.sum((Xdot[:, i] - Phi @ theta_i)**2))

    return A, B, Xdot, X, U, residuals, (dbx, dby, dbz)


def print_state_space_results(A, B, residuals):
    """Print identified state-space matrices with interpretation."""
    state_names = ['bx', 'by', 'bz', 'dbx', 'dby', 'dbz']
    input_names = ['i1', 'i2', 'i3', 'i4']
    deriv_names = ['dbx', 'dby', 'dbz', 'd2bx', 'd2by', 'd2bz']

    print("\n" + "="*70)
    print("STATE-SPACE MODEL: xdot = A*x + B*u")
    print("="*70)
    print(f"States x = {state_names}")
    print(f"Inputs u = {input_names}")
    print(f"xdot   = {deriv_names}")

    print("\n--- A Matrix (6x6) - State dynamics ---")
    print("       ", "  ".join(f"{s:>8}" for s in state_names))
    for i, row_name in enumerate(deriv_names):
        row_str = "  ".join(f"{A[i,j]:>8.4f}" for j in range(6))
        print(f"{row_name:>6}: {row_str}")

    print("\n--- B Matrix (6x4) - Input influence ---")
    print("       ", "  ".join(f"{s:>8}" for s in input_names))
    for i, row_name in enumerate(deriv_names):
        row_str = "  ".join(f"{B[i,j]:>8.4f}" for j in range(4))
        print(f"{row_name:>6}: {row_str}")

    print("\n--- Key findings ---")
    # Check if currents affect magnetic field derivatives (rows 0-2 of B)
    B_mag = B[:3, :]  # Effect on dbx, dby, dbz
    B_mag_norm = np.linalg.norm(B_mag)
    print(f"B matrix norm for mag derivatives (dbx,dby,dbz): {B_mag_norm:.4f}")

    # Check individual effects
    print("\nInput effects on magnetic field derivatives:")
    for j, inp in enumerate(input_names):
        effect = np.sqrt(B[0,j]**2 + B[1,j]**2 + B[2,j]**2)
        print(f"  {inp}: |effect| = {effect:.4f} (dbx:{B[0,j]:.4f}, dby:{B[1,j]:.4f}, dbz:{B[2,j]:.4f})")

    print(f"\nResiduals per state: {[f'{r:.2f}' for r in residuals]}")

    return B_mag


def detrend_with_state_space(df, B):
    """
    Apply state-space based detrending.
    Removes B*u contribution from magnetic field derivatives,
    then integrates to get detrended magnetic field.

    Parameters:
        df: DataFrame with bx, by, bz, i1-i4
        B: B matrix from state-space identification (6x4)

    Returns:
        bx_det, by_det, bz_det: Detrended signals
    """
    U = np.column_stack([
        df['i1'].values,
        df['i2'].values,
        df['i3'].values,
        df['i4'].values
    ])

    # Compute input contribution to derivatives: B[:3,:] @ u.T
    B_mag = B[:3, :]
    correction = (B_mag @ U.T).T  # N x 3

    # The model says: dbx = ... + B[0,:] @ u
    # So the input contribution to bx is the integral of B[0,:] @ u
    # For simplicity, use cumulative sum approximation
    dt = np.gradient(df['time_s'].values)

    # Integrate correction to get position-level correction
    bx_correction = np.cumsum(correction[:, 0] * dt)
    by_correction = np.cumsum(correction[:, 1] * dt)
    bz_correction = np.cumsum(correction[:, 2] * dt)

    # Remove mean of correction (we already removed bias from mag)
    bx_correction -= bx_correction.mean()
    by_correction -= by_correction.mean()
    bz_correction -= bz_correction.mean()

    bx_det = df['bx'].values - bx_correction
    by_det = df['by'].values - by_correction
    bz_det = df['bz'].values - bz_correction

    return bx_det, by_det, bz_det


def parse_data(filename):
    """Parse serial data file into pandas DataFrame."""
    with open(filename, 'r', encoding='utf-16') as f:
        lines = f.readlines()

    rows = []
    for line in lines:
        line = line.strip()
        if line.startswith('t:'):
            row = {}
            parts = line.split(',')
            for part in parts:
                if ':' in part:
                    key, val = part.split(':')
                    row[key.strip()] = float(val.strip())
            rows.append(row)

    return pd.DataFrame(rows)


# Parse data into DataFrame, using filename from args if supplied
if __name__ == "__main__":
    if len(sys.argv) > 1:
        filename = sys.argv[1]
        print(f"Using file from argument: {filename}")
    else:
        filename = 'data.txt'
        print("No filename supplied, using default: data.txt")
    df = parse_data(filename)
    # Convert time to relative seconds
    df['time_s'] = (df['t'] - df['t'].iloc[0]) / 1e6  # assuming microseconds
    # Remove bias from magnetic field data
    df['bx'] -= df['bx'].mean()
    df['by'] -= df['by'].mean()
    df['bz'] -= df['bz'].mean()

    # Store raw (unfiltered) data
    df['bx_raw'] = df['bx'].copy()
    df['by_raw'] = df['by'].copy()
    df['bz_raw'] = df['bz'].copy()

    # Apply low-pass filter to reduce Gaussian noise
    print("\n=== Filtering magnetic field data ===")
    df['bx'], df['by'], df['bz'] = filter_magnetic_field(df, cutoff_hz=50.0)

    # ===== STATE-SPACE IDENTIFICATION =====
    print("\n" + "="*70)
    print("IDENTIFYING STATE-SPACE MODEL")
    print("="*70)

    A, B, Xdot, X, U, residuals, (dbx, dby, dbz) = identify_state_space(df)

    # Store derivatives in dataframe
    df['dbx'], df['dby'], df['dbz'] = dbx, dby, dbz

    # Print results
    B_mag = print_state_space_results(A, B, residuals)

    # Apply state-space based detrending
    print("\n=== Applying state-space detrending ===")
    df['bx_ss'], df['by_ss'], df['bz_ss'] = detrend_with_state_space(df, B)

    # Compare raw vs state-space detrended
    raw_var = df['bx'].var() + df['by'].var() + df['bz'].var()
    ss_var = df['bx_ss'].var() + df['by_ss'].var() + df['bz_ss'].var()
    improvement = (1 - ss_var / raw_var) * 100

    print(f"Raw signal total variance:     {raw_var:.6f}")
    print(f"SS detrended total variance:   {ss_var:.6f}")
    print(f"Variance reduction: {improvement:.2f}%")

    # Use state-space detrended as main output
    df['bx_detrend'] = df['bx_ss']
    df['by_detrend'] = df['by_ss']
    df['bz_detrend'] = df['bz_ss']
    print(df.head())
    print(f"\nTotal samples: {len(df)}")

    # Compute error (correction applied) = raw - detrended
    df['error_x'] = df['bx'] - df['bx_detrend']
    df['error_y'] = df['by'] - df['by_detrend']
    df['error_z'] = df['bz'] - df['bz_detrend']

    # Compute derivative of error
    time_s = df['time_s'].values
    dt = np.diff(time_s)
    dt = np.where(dt == 0, 1e-6, dt)

    derror_x = np.zeros(len(df))
    derror_y = np.zeros(len(df))
    derror_z = np.zeros(len(df))
    for i in range(1, len(df) - 1):
        dt_avg = (dt[i-1] + dt[i]) / 2
        if dt_avg > 0:
            derror_x[i] = (df['error_x'].iloc[i+1] - df['error_x'].iloc[i-1]) / (2 * dt_avg)
            derror_y[i] = (df['error_y'].iloc[i+1] - df['error_y'].iloc[i-1]) / (2 * dt_avg)
            derror_z[i] = (df['error_z'].iloc[i+1] - df['error_z'].iloc[i-1]) / (2 * dt_avg)

    df['derror_x'] = np.nan_to_num(derror_x, nan=0.0, posinf=0.0, neginf=0.0)
    df['derror_y'] = np.nan_to_num(derror_y, nan=0.0, posinf=0.0, neginf=0.0)
    df['derror_z'] = np.nan_to_num(derror_z, nan=0.0, posinf=0.0, neginf=0.0)

    # Create single figure with 6 rows
    fig, axes = plt.subplots(6, 1, figsize=(14, 16), sharex=True)

    # Row 1: mag xyz
    axes[0].plot(df['time_s'], df['bx'], label='bx')
    axes[0].plot(df['time_s'], df['by'], label='by')
    axes[0].plot(df['time_s'], df['bz'], label='bz')
    axes[0].set_ylabel('Mag [uT]')
    axes[0].legend(loc='upper right')
    axes[0].grid(True)
    axes[0].set_title('Magnetic Field (filtered)')

    # Row 2: dmag xyz
    axes[1].plot(df['time_s'], df['dbx'], label='dbx')
    axes[1].plot(df['time_s'], df['dby'], label='dby')
    axes[1].plot(df['time_s'], df['dbz'], label='dbz')
    axes[1].set_ylabel('dMag/dt')
    axes[1].legend(loc='upper right')
    axes[1].grid(True)
    axes[1].set_title('Magnetic Field Derivatives')

    # Row 3: solenoid i1234
    axes[2].plot(df['time_s'], df['i1'], label='i1')
    axes[2].plot(df['time_s'], df['i2'], label='i2')
    axes[2].plot(df['time_s'], df['i3'], label='i3')
    axes[2].plot(df['time_s'], df['i4'], label='i4')
    axes[2].set_ylabel('Current [A]')
    axes[2].legend(loc='upper right')
    axes[2].grid(True)
    axes[2].set_title('Solenoid Currents (measured)')

    # Row 4: solenoid u1234
    if 'u1' in df.columns:
        axes[3].plot(df['time_s'], df['u1'], label='u1')
        axes[3].plot(df['time_s'], df['u2'], label='u2')
        axes[3].plot(df['time_s'], df['u3'], label='u3')
        axes[3].plot(df['time_s'], df['u4'], label='u4')
    axes[3].set_ylabel('PWM Input')
    axes[3].legend(loc='upper right')
    axes[3].grid(True)
    axes[3].set_title('Solenoid Inputs (commanded)')

    # Row 5: error xyz (correction = raw - detrended)
    axes[4].plot(df['time_s'], df['error_x'], label='error_x')
    axes[4].plot(df['time_s'], df['error_y'], label='error_y')
    axes[4].plot(df['time_s'], df['error_z'], label='error_z')
    axes[4].set_ylabel('Error')
    axes[4].legend(loc='upper right')
    axes[4].grid(True)
    axes[4].set_title('Error (raw - detrended)')

    # Row 6: derror xyz
    axes[5].plot(df['time_s'], df['derror_x'], label='derror_x')
    axes[5].plot(df['time_s'], df['derror_y'], label='derror_y')
    axes[5].plot(df['time_s'], df['derror_z'], label='derror_z')
    axes[5].set_xlabel('Time (s)')
    axes[5].set_ylabel('dError/dt')
    axes[5].legend(loc='upper right')
    axes[5].grid(True)
    axes[5].set_title('Error Derivative')

    plt.tight_layout()
    plt.show()
