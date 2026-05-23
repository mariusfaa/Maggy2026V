import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import matplotlib.transforms as transforms
import autograd.numpy as anp
from autograd import jacobian

def polar_to_cartesian(points):
    """
    Convert from polar coordinates (r, theta) to Cartesian (x, y)
    """
    points = anp.array(points)
    if points.ndim == 1:
        points = points.reshape(-1, 2)
    
    r = points[:, 0]
    theta = points[:, 1]
    
    x = r * anp.cos(theta)
    y = r * anp.sin(theta)
    
    return anp.column_stack([x, y])

def ekf_transform(mean, covariance):
    """
    Perform EKF-style linearized transform using Jacobian
    """
    # Calculate Jacobian at the mean point
    J = jacobian(polar_to_cartesian)(mean).reshape(2, 2)
    
    # Transform mean
    transformed_mean = polar_to_cartesian(mean).reshape(2,)
    
    # Transform covariance using linear approximation
    transformed_covariance = J @ covariance @ J.T
    
    return transformed_mean, transformed_covariance, J

def unscented_transform(mean, covariance, alpha=1.0, beta=2.0, kappa=0.0):
    """
    Perform Unscented Transform for a 2D to 2D transformation
    
    Parameters:
    -----------
    mean : array_like
        Mean of the input distribution
    covariance : array_like
        Covariance of the input distribution
    alpha : float, default=1.0
        Primary scaling parameter (determines spread of sigma points)
        Typical range: 1e-4 to 1
    beta : float, default=2.0
        Incorporates prior knowledge (beta=2 is optimal for Gaussian)
    kappa : float, default=0.0
        Secondary scaling parameter (usually 0 or 3-n)
    
    Returns:
    --------
    transformed_mean, transformed_covariance, sigma_points, transformed_sigma
    """
    n = len(mean)  # state dimension
    
    # Calculate lambda parameter
    lambda_ = alpha**2 * (n + kappa) - n
    
    # Calculate sigma points
    sigma_points = np.zeros((2*n + 1, n))
    sigma_points[0] = mean
    
    # Matrix square root of (n+lambda)*covariance
    sqrt_matrix = np.linalg.cholesky((n + lambda_) * covariance)
    
    for i in range(n):
        sigma_points[i + 1] = mean + sqrt_matrix[i]
        sigma_points[i + 1 + n] = mean - sqrt_matrix[i]
    
    # Transform sigma points
    transformed_sigma = polar_to_cartesian(sigma_points)
    
    # Calculate weights
    Wm = np.zeros(2*n + 1)
    Wc = np.zeros(2*n + 1)
    
    Wm[0] = lambda_ / (n + lambda_)
    Wc[0] = Wm[0] + (1 - alpha**2 + beta)
    
    for i in range(1, 2*n + 1):
        Wm[i] = 1 / (2 * (n + lambda_))
        Wc[i] = Wm[i]
    
    # Calculate transformed mean
    transformed_mean = np.sum(Wm[:, np.newaxis] * transformed_sigma, axis=0)
    
    # Calculate transformed covariance
    transformed_covariance = np.zeros((n, n))
    for i in range(2*n + 1):
        diff = transformed_sigma[i] - transformed_mean
        transformed_covariance += Wc[i] * np.outer(diff, diff)
    
    return transformed_mean, transformed_covariance, sigma_points, transformed_sigma

def plot_ukf_vs_ekf(mean_cart, cov_cart, n_samples=1000, random_seed=42):
    """
    Plot EKF vs UKF with default UT parameters
    """
    np.random.seed(random_seed)
    
    # Generate random samples to illustrate true transformation
    samples_cart = np.random.multivariate_normal(mean_cart, cov_cart, n_samples)
    samples_transformed_true = polar_to_cartesian(samples_cart)
    
    # Apply Unscented Transform (default parameters)
    mean_transformed_ut, cov_transformed_ut, sigma_points_cart, sigma_points_transformed = unscented_transform(mean_cart, cov_cart)
    
    # Apply EKF-style transform
    mean_transformed_ekf, cov_transformed_ekf, J = ekf_transform(mean_cart, cov_cart)
    
    # Calculate true mean and covariance from samples (for comparison)
    mean_transformed_true = np.mean(samples_transformed_true, axis=0)
    cov_transformed_true = np.cov(samples_transformed_true.T)
    
    # Create visualizations
    fig, axes = plt.subplots(1, 2, figsize=(18, 6))
    
    # Plot 1: Cartesian space with sigma points
    ax1 = axes[0]
    ax1.scatter(samples_cart[:, 0], samples_cart[:, 1], alpha=0.3, s=10, label='Random Samples')
    ax1.scatter(sigma_points_cart[:, 0], sigma_points_cart[:, 1], color='blue', s=100, 
               marker='*', label='Sigma Points', zorder=5)
    ax1.scatter(mean_cart[0], mean_cart[1], color='black', s=200, marker='x', 
               label='Mean', linewidth=2, zorder=5)
    
    # Plot covariance ellipse in Polar space
    eigenvals, eigenvecs = np.linalg.eigh(cov_cart)
    angle = np.degrees(np.arctan2(eigenvecs[1, 0], eigenvecs[0, 0]))
    width, height = 2 * np.sqrt(5.991 * eigenvals)  # 95% confidence ellipse
    
    ellipse = Ellipse(xy=mean_cart, width=width, height=height, angle=angle,
                      edgecolor='black', fc='None', lw=2, label='95% Confidence')
    ax1.add_patch(ellipse)
    
    ax1.set_xlabel('Radius')
    ax1.set_ylabel('Angle')
    ax1.set_title('Polar Space\nOriginal Distribution', fontsize=14)
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')
    ax1.legend()
    
    # Plot 2: Cartesian space - Comparison of methods
    ax2 = axes[1]
    
    # Convert samples to cartesian for plotting
    ax2.scatter(samples_transformed_true[:, 0], samples_transformed_true[:, 1], alpha=0.2, s=5, 
               color='gray', label='Transformed Samples')
    
    # Plot transformed sigma points
    ax2.scatter(sigma_points_transformed[:, 0], sigma_points_transformed[:, 1], color='blue', s=100, 
               marker='*', label='UT Sigma Points', zorder=5, alpha=0.7)
    
    # Plot means
    ax2.scatter(mean_transformed_true[0], mean_transformed_true[1], color='black', s=200, marker='x', 
               label='True Mean', linewidth=2, zorder=5)
    ax2.scatter(mean_transformed_ut[0], mean_transformed_ut[1], color='blue', s=200, marker='x', 
               label='UT Mean', linewidth=2, zorder=5)
    ax2.scatter(mean_transformed_ekf[0], mean_transformed_ekf[1], color='red', s=200, marker='x', 
               label='EKF Mean', linewidth=2, zorder=5)
    
    # Plot covariance ellipses
    # True covariance ellipse
    eigenvals_true, eigenvecs_true = np.linalg.eigh(cov_transformed_true)
    angle_true = np.degrees(np.arctan2(eigenvecs_true[1, 0], eigenvecs_true[0, 0]))
    width_true, height_true = 2 * np.sqrt(5.991 * eigenvals_true)
    ellipse_true = Ellipse(xy=mean_transformed_true, width=width_true, height=height_true, 
                           angle=angle_true, edgecolor='black', fc='None', lw=2, 
                           label='True 95%', linestyle='-')
    ax2.add_patch(ellipse_true)
    
    # UT covariance ellipse
    eigenvals_ut, eigenvecs_ut = np.linalg.eigh(cov_transformed_ut)
    angle_ut = np.degrees(np.arctan2(eigenvecs_ut[1, 0], eigenvecs_ut[0, 0]))
    width_ut, height_ut = 2 * np.sqrt(5.991 * eigenvals_ut)
    ellipse_ut = Ellipse(xy=mean_transformed_ut, width=width_ut, height=height_ut, 
                         angle=angle_ut, edgecolor='blue', fc='None', lw=2, 
                         linestyle='--', label='UT 95%')
    ax2.add_patch(ellipse_ut)
    
    # EKF covariance ellipse
    eigenvals_ekf, eigenvecs_ekf = np.linalg.eigh(cov_transformed_ekf)
    angle_ekf = np.degrees(np.arctan2(eigenvecs_ekf[1, 0], eigenvecs_ekf[0, 0]))
    width_ekf, height_ekf = 2 * np.sqrt(5.991 * eigenvals_ekf)
    ellipse_ekf = Ellipse(xy=mean_transformed_ekf, width=width_ekf, height=height_ekf, 
                         angle=angle_ekf, edgecolor='red', fc='None', lw=2, 
                         linestyle=':', label='EKF 95%')
    ax2.add_patch(ellipse_ekf)
    
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_title('Cartesian Space\nTransformed distribution', fontsize=14)
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc='upper right', fontsize=8)
    ax2.autoscale()
    fig.suptitle('Unscented Transform vs EKF-style Linearization', fontsize=14, y=0.99)
    
    
    return fig

def plot_ukf_comparison(mean_cart, cov_cart, ut_configs, n_samples=1000, random_seed=42):
    """
    Plot comparison of different Unscented Transform configurations
    
    Parameters:
    -----------
    mean_cart : array_like
        Mean of the input distribution
    cov_cart : array_like
        Covariance of the input distribution
    ut_configs : list of dicts
        List of configurations, each with 'alpha', 'beta', 'kappa', and 'label'
    n_samples : int, default=1000
        Number of random samples for ground truth
    random_seed : int, default=42
        Random seed for reproducibility
    """
    np.random.seed(random_seed)
    
    # Generate random samples for ground truth
    samples_cart = np.random.multivariate_normal(mean_cart, cov_cart, n_samples)
    samples_transformed_true = polar_to_cartesian(samples_cart)
    
    # Calculate true statistics
    mean_transformed_true = np.mean(samples_transformed_true, axis=0)
    cov_transformed_true = np.cov(samples_transformed_true.T)
    
    # Create figure
    n_configs = len(ut_configs)
    n_cols = min(2, n_configs)
    n_rows = (n_configs + n_cols - 1) // n_cols
    fig, axes = plt.subplots(n_rows, n_cols, figsize=(6*n_cols, 6*n_rows))
    
    if n_configs == 1:
        axes = np.array([axes])
    axes = axes.flatten()
    
    # For each configuration, compute and plot UT result
    for idx, config in enumerate(ut_configs):
        ax = axes[idx]
        
        # Apply UT with specified parameters
        mean_transformed_ut, cov_transformed_ut, sigma_points_cart, sigma_points_transformed = unscented_transform(
            mean_cart, cov_cart, 
            alpha=config['alpha'], 
            beta=config['beta'], 
            kappa=config['kappa']
        )
        
        # Plot transformed samples (ground truth)
        ax.scatter(samples_transformed_true[:, 0], samples_transformed_true[:, 1], 
                  alpha=0.15, s=3, color='gray', label=('True Samples' if idx==1 else ""))
        
        # Plot transformed sigma points
        ax.scatter(sigma_points_transformed[:, 0], sigma_points_transformed[:, 1], 
                  color='blue', s=60, marker='*', label=('UT Sigma Points' if idx==1 else ""), 
                  zorder=5, alpha=0.7)
        
        # Plot means
        ax.scatter(mean_transformed_true[0], mean_transformed_true[1], 
                  color='black', s=150, marker='x', label=('True Mean' if idx==1 else ""), 
                  linewidth=2, zorder=5)
        ax.scatter(mean_transformed_ut[0], mean_transformed_ut[1], 
                  color='blue', s=150, marker='x', label=('UT Mean' if idx==1 else ""), 
                  linewidth=2, zorder=5)
        
        # True covariance ellipse
        eigenvals_true, eigenvecs_true = np.linalg.eigh(cov_transformed_true)
        angle_true = np.degrees(np.arctan2(eigenvecs_true[1, 0], eigenvecs_true[0, 0]))
        width_true, height_true = 2 * np.sqrt(5.991 * eigenvals_true)
        ellipse_true = Ellipse(xy=mean_transformed_true, width=width_true, height=height_true, 
                               angle=angle_true, edgecolor='black', fc='None', lw=2, 
                               label=('True 95%' if idx==1 else ""), linestyle='-')
        ax.add_patch(ellipse_true)
        
        # UT covariance ellipse
        eigenvals_ut, eigenvecs_ut = np.linalg.eigh(cov_transformed_ut)
        angle_ut = np.degrees(np.arctan2(eigenvecs_ut[1, 0], eigenvecs_ut[0, 0]))
        width_ut, height_ut = 2 * np.sqrt(5.991 * eigenvals_ut)
        ellipse_ut = Ellipse(xy=mean_transformed_ut, width=width_ut, height=height_ut, 
                             angle=angle_ut, edgecolor='blue', fc='None', lw=2, 
                             linestyle='--', label=('UT 95%' if idx==1 else ""))
        ax.add_patch(ellipse_ut)
        
        # Add configuration info to title
        ax.set_title(f'{config["label"]}')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.grid(True, alpha=0.3)
        ax.set_xlim(-7, 5)
        ax.set_ylim(-4, 6.7) # 67!!
        #ax.autoscale()
    
    # Hide any unused subplots
    for idx in range(n_configs, len(axes)):
        axes[idx].axis('off')
    
    plt.suptitle('Comparison of Different Unscented Transform Configurations', fontsize=14, y=0.99)
    plt.tight_layout()
    fig.subplots_adjust(bottom=0.06)
    fig.legend(ncols=6, bbox_to_anchor=(0.5, -0.005), loc='lower center')
    
    return fig

def main():
    """
    Main function to run the UKF vs EKF comparison and UT parameter comparison
    """
    # ============================================
    # USER TUNABLE PARAMETERS - Easily modify these
    # ============================================
    
    # Initial distribution parameters (in polar coordinates)
    mean_cart = np.array([3.0, 2.0])  # [r, theta] - modify as needed
    cov_cart = np.array([[0.5, 0.3],   # Covariance matrix - modify as needed
                         [0.3, 0.8]])
    
    # Random sampling parameters
    n_samples = 1000  # Number of Monte Carlo samples for ground truth
    random_seed = 42  # For reproducibility
    
    # ============================================
    # UT CONFIGURATIONS TO COMPARE
    # Add/modify/remove configurations here as needed
    # ============================================
    ut_configs = [
        {
            'alpha': 1.0,
            'beta': 2.0,
            'kappa': 0.0,
            'label': 'Standard'
        },
        {
            'alpha': 1e-1,
            'beta': 2.0,
            'kappa': 0.0,
            'label': 'Tighter sigma spread'
        },
        {
            'alpha': 1.0,
            'beta': 2.0,
            'kappa': 1.0,
            'label': 'Adjusted $\\kappa$'
        },
        {
            'alpha': 1e-1,
            'beta': 2.0,
            'kappa': 1.0,
            'label': 'Tighter spread with adjusted $\\kappa$'
        }
    ]
    
    # ============================================
    # RUN THE COMPARISONS
    # ============================================
    
    print("Generating UKF vs EKF comparison plot...")
    fig1 = plot_ukf_vs_ekf(mean_cart, cov_cart, n_samples, random_seed)
    fig1.savefig('ukf_vs_ekf_transform.pdf', dpi=300, bbox_inches='tight')
    plt.show()
    
    print("\nGenerating UT configuration comparison plot...")
    #fig2 = plot_ukf_comparison(mean_cart, cov_cart, ut_configs, n_samples, random_seed)
    #fig2.savefig('ut_configuration_comparison.pdf', dpi=300, bbox_inches='tight')
    plt.show()
    
    print("\nDone! Check the generated PDF files for results.")

if __name__ == "__main__":
    main()