import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def load_and_plot_simulation_results(csv_filename="simulation_results.csv"):
    """
    Load simulation results from CSV and plot true states vs estimates.
    
    Parameters:
    csv_filename (str): Path to the CSV file containing simulation results
    """
    # Load the CSV file
    df = pd.read_csv(csv_filename)
    
    # Extract time and states
    t = df['t'].values
    x_true = df[['x', 'y', 'z']].values  # True states
    
    # Check if we have estimate columns
    # Note: The original C++ code only saved true states. We need to modify the C++ code
    # to also save estimates. Here's the updated Python code that expects estimates as well.
    
    # If estimates are present in the CSV, plot them
    if 'x_est' in df.columns and 'y_est' in df.columns and 'z_est' in df.columns:
        x_est = df[['x_est', 'y_est', 'z_est']].values
        has_estimates = True
    else:
        has_estimates = False
        print("Warning: No estimate columns found in CSV file.")
        print("Only true states will be plotted.")
    
    # Create subplots for each state
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    state_names = ['X', 'Y', 'Z']
    
    for i, ax in enumerate(axes):
        ax.plot(t, x_true[:, i], 'b-', linewidth=2, label=f'True {state_names[i]}')
        
        if has_estimates:
            ax.plot(t, x_est[:, i], 'r--', linewidth=2, label=f'Estimated {state_names[i]}')
            ax.legend(loc='best')
        
        ax.set_xlabel('Time (s)')
        ax.set_ylabel(f'{state_names[i]} State')
        ax.set_title(f'{state_names[i]} State: True vs Estimated')
        ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()
    
    # Optional: Plot estimation errors if estimates are available
    if has_estimates:
        fig, axes = plt.subplots(3, 1, figsize=(12, 8))
        errors = x_true - x_est
        
        for i, ax in enumerate(axes):
            ax.plot(t, errors[:, i], 'g-', linewidth=1.5)
            ax.set_xlabel('Time (s)')
            ax.set_ylabel(f'{state_names[i]} Error')
            ax.set_title(f'{state_names[i]} Estimation Error')
            ax.grid(True, alpha=0.3)
            ax.axhline(y=0, color='k', linestyle='--', alpha=0.5)
        
        plt.tight_layout()
        plt.show()
        
        # Print error statistics
        print("\nEstimation Error Statistics:")
        print("-" * 40)
        for i, name in enumerate(state_names):
            rmse = np.sqrt(np.mean(errors[:, i]**2))
            max_error = np.max(np.abs(errors[:, i]))
            print(f"{name}: RMSE = {rmse:.6f}, Max Error = {max_error:.6f}")




if __name__ == "__main__":
    path = "~/MSc/Maggy2026V/observer/PCobserver/simulation_results.csv"
    load_and_plot_simulation_results(path)