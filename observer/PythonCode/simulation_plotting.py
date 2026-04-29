import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
import matplotlib.patches as mpatches
import numpy as np

# Mapping of state name to latex symbols for plotting
nameMap = {
    'x': '$x$',
    'y': '$y$',
    'z': '$z$',
    'alpha': '$\\alpha$',
    'beta': '$\\beta$',
    'x_dot': '$\\dot x$',
    'y_dot': '$\\dot y$',
    'z_dot': '$\\dot z$',
    'alpha_dot': '$\\dot\\alpha$',
    'beta_dot': '$\\dot\\beta$',
    'bx': '$b_x$',
    'by': '$b_y$',
    'bz': '$b_z$'
}

# Artists whose handles make the legends
true_marker = mlines.Line2D([], [], color='blue', ls='-', label='True', lw=2)
est_marker = mlines.Line2D([], [], color='red', ls='--', label='Estimated', lw=2)
s0_marker = mlines.Line2D([], [], color='red', ls='--', label='Sensor 0', lw=2)
s1_marker = mlines.Line2D([], [], color='green', ls=':', label='Sensor 1', lw=2)
s2_marker = mlines.Line2D([], [], color='purple', ls='-.', label='Sensor 2', lw=2)
ci_marker = mpatches.Patch(color='red', alpha=0.3, label='1$\\sigma$ bounds')
s0_ci_marker = mpatches.Patch(color='red', alpha=0.3, label='Sensor 0 1$\\sigma$ bounds')
s1_ci_marker = mpatches.Patch(color='green', alpha=0.3, label='Sensor 1 1$\\sigma$ bounds')
s2_ci_marker = mpatches.Patch(color='purple', alpha=0.3, label='Sensor 2 1$\\sigma$ bounds')
legend_handles = [true_marker, est_marker, ci_marker]
sensor_legend_handles = [s0_marker, s0_ci_marker,
                         s1_marker, s1_ci_marker,
                         s2_marker, s2_ci_marker,
                         true_marker]


fig1Title = '6 State Linear Kalman Filter on Maggy Simulator'
fig2Title = fig1Title
fig3Title = fig1Title


def plot_displacement(csv):
    """
    Load CSV file and create plots for true vs estimated states with confidence bounds
    """
    # Load the CSV file
    df = pd.read_csv(csv)
    
    # Extract time column
    t = df['t'].values
    
    # Define state names and their corresponding columns
    states = ['x', 'y', 'z']
    states_dot = ['x_dot', 'y_dot', 'z_dot']
    
    # Create figure for states and their derivatives
    fig1, axes = plt.subplots(3, 2, figsize=(20, 10))
    fig1.suptitle(fig1Title, fontsize=16)
    
    # Plot position
    axes[0, 0].set_title('Position')
    for i, state in enumerate(states):
        ax = axes[i, 0]
        
        # Get true and estimated values
        true_val = df[state].values
        est_val = df[f'{state}_est'].values
        std_val = df[f'{state}_std'].values
        
        # Plot true and estimated
        ax.plot(t, true_val, 'b-', label='True', linewidth=2)
        ax.plot(t, est_val, 'r--', label='Estimated', linewidth=2)
        
        # Add confidence bounds (mean ± 1σ)
        ax.fill_between(t, est_val - std_val, est_val + std_val, alpha=0.3, color='red', label='1$\\sigma$ bounds')

        #ax.set_ylim()
        ax.set_ylabel(f'{nameMap[state]} [m]')
        ax.grid(True, alpha=0.3)
    ax.set_xlabel('Time (s)')
    
    # Plot derivatives
    axes[0, 1].set_title('Linear Velocities')
    for i, state_dot in enumerate(states_dot):
        ax = axes[i, 1]
        
        # Get true and estimated values
        true_val = df[state_dot].values
        est_val = df[f'{state_dot}_est'].values
        std_val = df[f'{state_dot}_std'].values
        
        # Plot true and estimated
        ax.plot(t, true_val, 'b-', label='True', linewidth=2)
        ax.plot(t, est_val, 'r--', label='Estimated', linewidth=2)
        
        # Add confidence bounds (mean ± 1σ)
        ax.fill_between(t, est_val - std_val, est_val + std_val, alpha=0.3, color='red', label='1$\\sigma$ bounds')

        ax.set_ylabel(f'{nameMap[state_dot]} [m/s]')
        ax.grid(True, alpha=0.3)
    ax.set_xlabel('Time (s)')
    
    fig1.legend(handles=legend_handles, ncols=len(legend_handles), bbox_to_anchor=(0.5, -0.005), loc='lower center')

    plt.tight_layout()
    
    # Show the plots
    plt.show()
    
    return fig1


def plot_attitude(csv):
    """
    Load CSV file and create plots for true vs estimated states with confidence bounds (angle edition!)
    """
    # Load the CSV file
    df = pd.read_csv(csv)
    
    # Extract time column
    t = df['t'].values
    
    # Define state names and their corresponding columns
    states = ['alpha', 'beta']
    states_dot = ['alpha_dot', 'beta_dot']
    
    # Create figure for states and their derivatives
    fig2, axes = plt.subplots(2, 2, figsize=(20, 10))
    fig2.suptitle(fig2Title, fontsize=16)
    
    # Plot angles
    axes[0, 0].set_title('Attitude')
    for i, state in enumerate(states):
        ax = axes[i, 0]
        
        # Get true and estimated values
        true_val = df[state].values
        try:
            est_val = df[f'{state}_est'].values
            std_val = df[f'{state}_std'].values
        except KeyError:
            pass
        
        # Plot true and estimated
        ax.plot(t, true_val, 'b-', label='True', linewidth=2)
        if 'est_val' in locals():
            ax.plot(t, est_val, 'r--', label='Estimated', linewidth=2)
        
        # Add confidence bounds (mean ± 1σ)
        if 'est_val' in locals():
            ax.fill_between(t, est_val - std_val, est_val + std_val, alpha=0.3, color='red', label='1$\\sigma$ bounds')

        ax.set_ylabel(f'{nameMap[state]} [rad]')
        ax.grid(True, alpha=0.3)
    ax.set_xlabel('Time (s)')
    
    # Plot derivatives
    axes[0, 1].set_title('Angular Velocities')
    for i, state_dot in enumerate(states_dot):
        ax = axes[i, 1]
        
        # Get true and estimated values
        true_val = df[state_dot].values
        if 'est_val' in locals():
            est_val = df[f'{state_dot}_est'].values
            std_val = df[f'{state_dot}_std'].values
        
        # Plot true and estimated
        ax.plot(t, true_val, 'b-', label='True', linewidth=2)
        if 'est_val' in locals():
            ax.plot(t, est_val, 'r--', label='Estimated', linewidth=2)
        
        # Add confidence bounds (mean ± 1σ)
        if 'est_val' in locals():
            ax.fill_between(t, est_val - std_val, est_val + std_val, alpha=0.3, color='red', label='1$\\sigma$ bounds')

        ax.set_ylabel(f'{nameMap[state_dot]} [rad/s]')
        ax.grid(True, alpha=0.3)
    ax.set_xlabel('Time (s)')

    fig2.legend(handles=legend_handles, ncols=len(legend_handles), bbox_to_anchor=(0.5, -0.005), loc='lower center')
    
    plt.tight_layout()
    
    # Show the plots
    plt.show()
    
    return fig2

def plot_measurements(csv, N, n):
    """
    Load CSV file and create plots for true vs estimated measurements with confidence bounds
    """
    # Load the CSV file
    df = pd.read_csv(csv)
    
    # Extract time column
    t = df['t'].values
    
    # Define state names and their corresponding columns
    
    if N == 1:
        n = ""
    measurements = [f'bx{n}', f'by{n}', f'bz{n}']

    # Create figure for measurements
    fig3, axes = plt.subplots(3, 1, figsize=(20, 10))
    fig3.suptitle(fig3Title, fontsize=16)
    
    # Plot magnetic field
    axes[0].set_title('Magnetic Field')
    for i, meas in enumerate(measurements):
        ax = axes[i]
        
        # Get true and estimated values
        true_val = df[meas].values
        est_val = df[f'{meas}_est'].values
        std_val = df[f'{meas}_std'].values
        
        # Plot true and estimated
        ax.plot(t[1:], true_val[1:], 'b-', label='True', linewidth=2)
        ax.plot(t[1:], est_val[1:], 'r--', label='Estimated', linewidth=2)
        
        # Add confidence bounds (mean ± 1σ)
        ax.fill_between(t[1:], est_val[1:] - std_val[1:], est_val[1:] + std_val[1:], alpha=0.3, color='red', label='1$\\sigma$ bounds')

        ax.set_ylabel(f'{meas} [T]')
        ax.grid(True, alpha=0.3)
    ax.set_xlabel('Time (s)')


    fig3.legend(handles=legend_handles, ncols=len(legend_handles), bbox_to_anchor=(0.5, -0.005), loc='lower right')
    
    plt.tight_layout()
    
    # Show the plots
    plt.show()
    
    return fig3

def plot_error_statistics(csv):
    """
    Load CSV file and create comprehensive error statistics plots including:
    - Error time series with confidence bounds
    - Error histograms with Gaussian fits
    - Normalized estimation error squared (NEES)
    - Root mean square error (RMSE) over time
    - Error correlation analysis
    """
    # Load the CSV file
    df = pd.read_csv(csv)
    
    # Extract time column
    t = df['t'].values
    
    # Define state names and their corresponding columns
    states = ['x', 'y', 'z']
    states_dot = ['x_dot', 'y_dot', 'z_dot']
    all_states = states + states_dot
    
    # Create figure with multiple subplots
    fig = plt.figure(figsize=(16, 12))
    gs = fig.add_gridspec(3, 3, hspace=0.3, wspace=0.3)
    
    # 1. Error time series (top row)
    ax_error = []
    for i, state in enumerate(all_states[:3]):  # First three states (positions)
        ax = fig.add_subplot(gs[0, i])
        
        # Calculate error (estimated - true)
        error = df[f'err_{state}'].values
        std_val = df[f'{state}_std'].values
        
        # Plot error
        ax.plot(t, error, 'b-', linewidth=1.5, alpha=0.7, label='Error')
        
        # Plot ±1σ bounds
        ax.fill_between(t, -std_val, std_val, alpha=0.3, color='red', label='±1$\\sigma$ (theoretical)')
        ax.fill_between(t, -2*std_val, 2*std_val, alpha=0.15, color='red', label='±2$\\sigma$ (theoretical)')
        
        # Add zero line
        ax.axhline(y=0, color='k', linestyle='--', linewidth=0.8, alpha=0.5)
        
        ax.set_ylabel(f'{nameMap[state]} Error [m]')
        ax.set_xlabel('Time (s)')
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)
        ax.set_title(f'{nameMap[state]} Estimation Error')
        
        # Add RMSE annotation
        rmse = np.sqrt(np.mean(error**2))
        ax.text(0.05, 0.95, f'RMSE = {rmse:.3f} m', transform=ax.transAxes, 
                verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    # 2. Error histograms with Gaussian fits (middle row)
    for i, state in enumerate(all_states[:3]):
        ax = fig.add_subplot(gs[1, i])
        
        error = df[f'err_{state}'].values 
        std_theoretical = df[f'{state}_std'].values.mean()  # Average theoretical std
        
        # Plot histogram
        n, bins, patches = ax.hist(error, bins=50, density=True, alpha=0.7, 
                                   color='blue', edgecolor='black', linewidth=0.5, label='Error histogram')
        
        # Fit Gaussian to the error
        mu, std_actual = np.mean(error), np.std(error)
        x = np.linspace(bins[0], bins[-1], 100)
        gaussian_fit = (1/(std_actual * np.sqrt(2*np.pi))) * np.exp(-0.5*((x-mu)/std_actual)**2)
        ax.plot(x, gaussian_fit, 'r-', linewidth=2, label=f'Gaussian fit\nμ={mu:.3f}, σ={std_actual:.3f}')
        
        # Plot theoretical Gaussian
        gaussian_theoretical = (1/(std_theoretical * np.sqrt(2*np.pi))) * np.exp(-0.5*((x)/std_theoretical)**2)
        ax.plot(x, gaussian_theoretical, 'g--', linewidth=2, label=f'1$\\sigma$ theory\nσ={std_theoretical:.3f}')
        
        ax.set_xlabel(f'{nameMap[state]} Error [m]')
        ax.set_ylabel('Probability Density')
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=7)
        ax.set_title(f'{nameMap[state]} Error Distribution')
    

    # 4. Cumulative RMSE (bottom middle)
    ax = fig.add_subplot(gs[2, 1])
    
    for state in all_states[:3]:
        error = df[f'{state}_est'].values - df[state].values
        cumulative_rmse = np.sqrt(np.cumsum(error**2) / np.arange(1, len(error)+1))
        ax.plot(t, cumulative_rmse, label=f'{nameMap[state]}', linewidth=1.5)
    
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Cumulative RMSE [m]')
    ax.set_title('Cumulative RMSE Over Time')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)
    
    # 5. Error autocorrelation (bottom right)
    ax = fig.add_subplot(gs[2, 2])
    
    max_lag = 100
    for state in all_states[:3]:
        error = df[f'{state}_est'].values - df[state].values
        autocorr = np.correlate(error - np.mean(error), error - np.mean(error), mode='full')
        autocorr = autocorr[len(autocorr)//2:] / autocorr[len(autocorr)//2]
        
        lags = np.arange(min(max_lag, len(autocorr)))
        ax.plot(lags, autocorr[:len(lags)], label=f'{nameMap[state]}', linewidth=1.5, alpha=0.7)
    
    ax.axhline(y=0, color='k', linestyle='-', linewidth=0.5, alpha=0.3)
    ax.axhline(y=0.2, color='r', linestyle='--', linewidth=0.8, alpha=0.5, label='95% significance')
    ax.axhline(y=-0.2, color='r', linestyle='--', linewidth=0.8, alpha=0.5)
    ax.fill_between([0, max_lag], -0.2, 0.2, alpha=0.1, color='gray')
    
    ax.set_xlabel('Lag [samples]')
    ax.set_ylabel('Autocorrelation')
    ax.set_title('Error Autocorrelation')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)
    ax.set_xlim([0, max_lag])
    
    # Add summary statistics table as text
    fig.text(0.02, 0.02, 
             f"Summary Statistics:\n"
             f"Position RMSE - X: {np.sqrt(np.mean((df['x_est']-df['x'])**2)):.3f} m, "
             f"Y: {np.sqrt(np.mean((df['y_est']-df['y'])**2)):.3f} m, "
             f"Z: {np.sqrt(np.mean((df['z_est']-df['z'])**2)):.3f} m\n"
             f"Mean Error - X: {np.mean(df['x_est']-df['x']):.4f} m, "
             f"Y: {np.mean(df['y_est']-df['y']):.4f} m, "
             f"Z: {np.mean(df['z_est']-df['z']):.4f} m\n"
             f"Error Std Dev - X: {np.std(df['x_est']-df['x']):.4f} m, "
             f"Y: {np.std(df['y_est']-df['y']):.4f} m, "
             f"Z: {np.std(df['z_est']-df['z']):.4f} m",
             fontsize=9, bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    plt.suptitle('Error Statistics Analysis', fontsize=16, y=0.98)
    plt.tight_layout()
    
    # Optional: Create separate figure for velocity errors
    fig2 = plt.figure(figsize=(12, 8))
    gs2 = fig2.add_gridspec(2, 3, hspace=0.3, wspace=0.3)
    
    # Velocity error time series
    for i, state in enumerate(states_dot):
        ax = fig2.add_subplot(gs2[0, i])
        
        error = df[f'err_{state}'].values
        std_val = df[f'{state}_std'].values
        
        ax.plot(t, error, 'b-', linewidth=1.5, alpha=0.7)
        ax.fill_between(t, -std_val, std_val, alpha=0.3, color='red', label='±1$\\sigma$')
        ax.axhline(y=0, color='k', linestyle='--', linewidth=0.8, alpha=0.5)
        
        ax.set_ylabel(f'{nameMap[state]} Error [m/s]')
        ax.set_xlabel('Time (s)')
        ax.grid(True, alpha=0.3)
        ax.set_title(f'{nameMap[state]} Velocity Error')
        
        rmse = np.sqrt(np.mean(error**2))
        ax.text(0.05, 0.95, f'RMSE = {rmse:.3f} m/s', transform=ax.transAxes, 
                verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    # Velocity error histograms
    for i, state in enumerate(states_dot):
        ax = fig2.add_subplot(gs2[1, i])
        
        error = df[f'err_{state}'].values
        ax.hist(error, bins=50, density=True, alpha=0.7, color='blue', edgecolor='black')
        
        # Fit Gaussian
        mu, std_actual = np.mean(error), np.std(error)
        x = np.linspace(np.min(error), np.max(error), 100)
        gaussian_fit = (1/(std_actual * np.sqrt(2*np.pi))) * np.exp(-0.5*((x-mu)/std_actual)**2)
        ax.plot(x, gaussian_fit, 'r-', linewidth=2, label=f'μ={mu:.3f}, σ={std_actual:.3f}')
        
        ax.set_xlabel(f'{nameMap[state]} Error [m/s]')
        ax.set_ylabel('Density')
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)
        ax.set_title(f'{nameMap[state]} Error Distribution')
    
    plt.suptitle('Velocity Error Statistics', fontsize=14)
    plt.tight_layout()
    
    plt.show()
    
    return fig, fig2


def main():
    # Specify the CSV filename
    csv = "~/MSc/Maggy2026V/observer/PCobserver/results/simulation_results.csv"  # Change this to your actual filename
    
    try:
        fig1 = plot_displacement(csv)
        #fig1_err = plot_error_statistics(csv)
        fig2 = plot_attitude(csv)
        N = 1
        fig31 = plot_measurements(csv, N, 0)
        if N > 1:
            fig32 = plot_measurements(csv, N, 1)
            fig33 = plot_measurements(csv, N, 2)
        print("Plots created successfully!")
        
        # Saving figures
        fig1.savefig('displacement.pdf', dpi=300, bbox_inches='tight')
        fig2.savefig('attitude.pdf', dpi=300, bbox_inches='tight')

        fig31.savefig('measurements0.pdf', dpi=300, bbox_inches='tight')
        if N > 1:
            fig32.savefig('measurements1.pdf', dpi=300, bbox_inches='tight')
            fig33.savefig('measurements2.pdf', dpi=300, bbox_inches='tight')
        
    except FileNotFoundError:
        print(f"Error: File '{csv}' not found!")
    #except Exception as e:
        #print(f"Error: {e}")

if __name__ == "__main__":
    main()
