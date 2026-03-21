import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import scipy

read_path = "~/MSc/Maggy2026V/observer/PCobserver/"
data = pd.read_csv(read_path + "simulation_results.csv")
nis_values = data["nis"].values
nees_values = data["nees"].values


# Compute NIS and NEES and plot on a logarithmic scale with 95% CI
plt.figure(figsize=(10, 6))

# prepare arrays
timesteps = len(nis_values)

# degrees of freedom
# measurement dim (m) and state dim (n)
m = 3
n = 10

# 95% confidence interval from chi-square
nis_ci = scipy.stats.chi2.ppf([0.025, 0.975], df=m)
nees_ci = scipy.stats.chi2.ppf([0.025, 0.975], df=n)

eps = 1e-12

# NIS plot (log scale)
plt.subplot(2, 1, 1)
plt.semilogy(np.maximum(nis_values, eps), label='NIS')
inside_nis = np.mean((nis_values >= nis_ci[0]) & (nis_values <= nis_ci[1])) * 100.0
plt.hlines(nis_ci[0], 0, timesteps - 1, colors='C1', linestyles='--', label=rf'$\chi^2$' + f' 95% CI' + f" ({inside_nis:.1f}% inside)")
plt.hlines(nis_ci[1], 0, timesteps - 1, colors='C1', linestyles='--')
plt.title('NIS')
plt.ylabel('NIS')
plt.legend(loc='upper right')

# NEES plot (log scale)
plt.subplot(2, 1, 2)
plt.semilogy(np.maximum(nees_values, eps), label='NEES')
inside_nees = np.mean((nees_values >= nees_ci[0]) & (nees_values <= nees_ci[1])) * 100.0
plt.hlines(nees_ci[0], 0, timesteps - 1, colors='C1', linestyles='--', label=rf'$\chi^2$' + f' 95% CI' + f" ({inside_nees:.1f}% inside)")
plt.hlines(nees_ci[1], 0, timesteps - 1, colors='C1', linestyles='--')
plt.title('NEES')
plt.xlabel('Timestep')
plt.ylabel('NEES')
plt.legend(loc='upper right')

plt.show()