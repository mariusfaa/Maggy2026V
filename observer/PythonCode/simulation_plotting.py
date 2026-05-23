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

# CSV filename
csv = "~/MSc/Maggy2026V/observer/MatlabCode/simulation_results.csv"
data = pd.read_csv(csv)

if 'alpha_est' in data:
    n = 10
else:
    n = 6 # no attitude estimates

filterIndex = int(data.filterVariant[0])
filterVariants = ['Linear', 'Extended', 'Unscented']

fig1Title = f'{n} State {filterVariants[filterIndex]} Kalman Filter on Maggy Simulator'
fig2Title = fig1Title
fig3Title = fig1Title


def plot_displacement(df):
    """
    Create plots for true vs estimated states with confidence bounds
    """
    
    # Extract time column
    t = df['t'].values
    
    # Define state names and their corresponding columns
    states = ['x', 'y', 'z']
    states_dot = ['x_dot', 'y_dot', 'z_dot']
    
    # Create figure for states and their derivatives
    fig, axes = plt.subplots(3, 2, figsize=(20, 10))
    fig.suptitle(fig1Title, fontsize=14)
    
    # Plot position
    axes[0, 0].set_title('Position')
    for i, state in enumerate(states):
        ax = axes[i, 0]
        
        # Get true and estimated values
        true_val = df[state].values
        est_val = df[f'{state}_est'].values
        std_val = df[f'{state}_std'].values

        # Error
        error = df[f'err_{state}'].values
        rmse = np.sqrt(np.mean(np.square(error)))

        ax.text(0.05, 0.95, f'RMSE = {rmse:.3f} mm', transform=ax.transAxes, 
                verticalalignment='center', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

        
        # Plot true and estimated
        ax.plot(t, true_val, 'b-', label='True', linewidth=2)
        ax.plot(t, est_val, 'r--', label='Estimated', linewidth=2)
        
        # Add confidence bounds (mean ± 1σ)
        ax.fill_between(t, est_val - std_val, est_val + std_val, alpha=0.3, color='red', label='1$\\sigma$ bounds')

        #ax.set_ylim()
        ax.set_ylabel(f'{nameMap[state]} [mm]')
        ax.grid(True, alpha=0.3)
    ax.set_xlabel('Time [s]')
    
    # Plot derivatives
    axes[0, 1].set_title('Linear Velocities')
    for i, state_dot in enumerate(states_dot):
        ax = axes[i, 1]
        
        # Get true and estimated values
        true_val = df[state_dot].values
        est_val = df[f'{state_dot}_est'].values
        std_val = df[f'{state_dot}_std'].values

        # Error
        error = df[f'err_{state_dot}'].values
        rmse = np.sqrt(np.mean(np.square(error)))

        ax.text(0.05, 0.95, f'RMSE = {rmse:.3f} mm/s', transform=ax.transAxes, 
                verticalalignment='center', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        # Plot true and estimated
        ax.plot(t, true_val, 'b-', label='True', linewidth=2)
        ax.plot(t, est_val, 'r--', label='Estimated', linewidth=2)
        
        # Add confidence bounds (mean ± 1σ)
        ax.fill_between(t, est_val - std_val, est_val + std_val, alpha=0.3, color='red', label='1$\\sigma$ bounds')

        ax.set_ylabel(f'{nameMap[state_dot]} [mm/s]')
        ax.grid(True, alpha=0.3)
    ax.set_xlabel('Time (s)')
    
    fig.legend(handles=legend_handles, ncols=len(legend_handles), bbox_to_anchor=(0.5, -0.005), loc='lower center')

    plt.tight_layout()
    
    # Show the plots
    plt.show()
    
    return fig


def plot_attitude(df):
    """
    Create plots for true vs estimated states with confidence bounds (angle edition!)
    """
    
    # Extract time column
    t = df['t'].values
    
    # Define state names and their corresponding columns
    states = ['alpha', 'beta']
    states_dot = ['alpha_dot', 'beta_dot']
    
    # Create figure for states and their derivatives
    fig, axes = plt.subplots(2, 2, figsize=(20, 10))
    fig.suptitle(fig2Title, fontsize=14)
    
    # Plot angles
    axes[0, 0].set_title('Attitude', fontsize=14)
    for i, state in enumerate(states):
        ax = axes[i, 0]
        
        # Get true and estimated values
        true_val = df[state].values
        try:
            est_val = df[f'{state}_est'].values
            std_val = df[f'{state}_std'].values

            # Error
            error = df[f'err_{state}'].values
            rmse = np.sqrt(np.mean(np.square(error)))

            ax.text(0.05, 0.95, f'RMSE = {rmse:.3f} mrad', transform=ax.transAxes, 
                    verticalalignment='center', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        except KeyError:
            pass
        
        # Plot true and estimated
        ax.plot(t, true_val, 'b-', label='True', linewidth=2)
        if 'est_val' in locals():
            ax.plot(t, est_val, 'r--', label='Estimated', linewidth=2)
        
        # Add confidence bounds (mean ± 1σ)
        if 'est_val' in locals():
            ax.fill_between(t, est_val - std_val, est_val + std_val, alpha=0.3, color='red', label='1$\\sigma$ bounds')

        ax.set_ylabel(f'{nameMap[state]} [mrad]')
        ax.grid(True, alpha=0.3)
    ax.set_xlabel('Time [s]')
    
    # Plot derivatives
    axes[0, 1].set_title('Angular Velocities', fontsize=14)
    for i, state_dot in enumerate(states_dot):
        ax = axes[i, 1]
        
        # Get true and estimated values
        true_val = df[state_dot].values
        if 'est_val' in locals():
            est_val = df[f'{state_dot}_est'].values
            std_val = df[f'{state_dot}_std'].values

            # Error
            error = df[f'err_{state}'].values
            rmse = np.sqrt(np.mean(np.square(error)))

            ax.text(0.05, 0.95, f'RMSE = {rmse:.3f} mrad/s', transform=ax.transAxes, 
                    verticalalignment='center', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        # Plot true and estimated
        ax.plot(t, true_val, 'b-', label='True', linewidth=2)
        if 'est_val' in locals():
            ax.plot(t, est_val, 'r--', label='Estimated', linewidth=2)
        
        # Add confidence bounds (mean ± 1σ)
        if 'est_val' in locals():
            ax.fill_between(t, est_val - std_val, est_val + std_val, alpha=0.3, color='red', label='1$\\sigma$ bounds')

        ax.set_ylabel(f'{nameMap[state_dot]} [mrad/s]')
        ax.grid(True, alpha=0.3)
    ax.set_xlabel('Time (s)')

    fig.legend(handles=legend_handles, ncols=len(legend_handles), bbox_to_anchor=(0.5, -0.005), loc='lower center')
    
    plt.tight_layout()
    
    # Show the plots
    plt.show()
    
    return fig

def plot_measurements(df, N, n):
    """
    Create plots for true vs estimated measurements with confidence bounds
    """
    
    # Extract time column
    t = df['t'].values
    
    # Define state names and their corresponding columns
    
    if N == 1:
        n = ""
    measurements = [f'bx{n}', f'by{n}', f'bz{n}']

    # Create figure for measurements
    fig, axes = plt.subplots(3, 1, figsize=(20, 10))
    fig.suptitle(fig3Title, fontsize=14)
    
    # Plot magnetic field
    axes[0].set_title('Magnetic Field Measurements', fontsize=14)
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

        ax.set_ylabel(f'{meas} [mT]')
        ax.grid(True, alpha=0.3)
        #ax.set_ylim(-10, 10)
    ax.set_xlabel('Time [s]')
    #ax.set_ylim(0, 30)


    fig.legend(handles=legend_handles, ncols=len(legend_handles), bbox_to_anchor=(0.5, -0.005), loc='lower right')
    
    plt.tight_layout()
    
    # Show the plots
    plt.show()
    
    return fig


def main():
    
    try:
        fig1 = plot_displacement(data)
        fig2 = plot_attitude(data)
        N = 1
        fig31 = plot_measurements(data, N, 0)
        if N > 1:
            fig32 = plot_measurements(data, N, 1)
            fig33 = plot_measurements(data, N, 2)
        print("Plots created successfully!")
        
        # Saving figures
        fig1.savefig('displacement.pdf', dpi=300, bbox_inches='tight')
        fig2.savefig('attitude.pdf', dpi=300, bbox_inches='tight')

        fig31.savefig('measurements.pdf', dpi=300, bbox_inches='tight')
        if N > 1:
            fig32.savefig('measurements1.pdf', dpi=300, bbox_inches='tight')
            fig33.savefig('measurements2.pdf', dpi=300, bbox_inches='tight')
        
    except FileNotFoundError:
        print(f"Error: File '{csv}' not found!")


if __name__ == "__main__":
    main()
