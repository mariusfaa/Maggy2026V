import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import scipy

read_path = "~/MSc/Maggy2026V/observer/MatlabCode"
data = pd.read_csv(read_path + "/simulation_results.csv")
nis_values = data["nis"].values
nees_values = data["nees"].values
t = data['t'].values

# Select filter name
filterIndex = int(data.filterVariant[0])
filterVariants = ['Linear', 'Extended', 'Unscented']

# Compute NIS and NEES and plot on a logarithmic scale with 95% CI
plt.figure(figsize=(10, 6))

# prepare arrays
timesteps = len(nis_values)


# popping first element for better visualization
nis_values = nis_values[1:]
nees_values = nees_values[1:]
t = t[1:]
timesteps -= 1

# degrees of freedom
# measurement dim (m) and state dim (n)
m = 3
if 'alpha_est' in data:
    n = 10
else:
    n = 6 # no attitude estimates

figTitle = f'{n} State {filterVariants[filterIndex]} Kalman Filter on Maggy Simulator'


# 95% confidence interval from chi-square
nis_ci = scipy.stats.chi2.ppf([0.025, 0.975], df=m)
nees_ci = scipy.stats.chi2.ppf([0.025, 0.975], df=n)


eps = 1e-12

# NIS plot (log scale)
plt.subplot(2, 1, 1)
plt.semilogy(t, np.maximum(nis_values, eps), label='NIS')
inside_nis = np.mean((nis_values >= nis_ci[0]) & (nis_values <= nis_ci[1])) * 100.0
plt.hlines(nis_ci[0], 0, t[-1], colors='C1', linestyles='--', label=rf'$\chi^2$' + f' 95% CI' + f" ({inside_nis:.1f}% inside)")
plt.hlines(nis_ci[1], 0, t[-1], colors='C1', linestyles='--')
plt.ylabel('NIS')
plt.legend(loc='upper right')
plt.gca().set_ylim(nis_ci[0]*1e-1, nis_ci[1]*1e+1)
#plt.gca().set_ylim(min(nis_values)*1e-1, max(nis_values)*1e+1)
#plt.gca().set_ylim(nis_ci[0]*1e-1, max(nis_values)*1e+1)

plt.title(figTitle, fontsize=14)

# NEES plot (log scale)
plt.subplot(2, 1, 2)
plt.semilogy(t, np.maximum(nees_values, eps), label='NEES')
inside_nees = np.mean((nees_values >= nees_ci[0]) & (nees_values <= nees_ci[1])) * 100.0
plt.hlines(nees_ci[0], 0, t[-1], colors='C1', linestyles='--', label=rf'$\chi^2$' + f' 95% CI' + f" ({inside_nees:.1f}% inside)")
plt.hlines(nees_ci[1], 0, t[-1], colors='C1', linestyles='--')
plt.xlabel('Time [s]')
plt.ylabel('NEES')
plt.legend(loc='upper right')
plt.gca().set_ylim(nees_ci[0]*1e-1, nees_ci[1]*1e+1)
#plt.gca().set_ylim(nees_values*1e-1, max(nees_values)*1e+1)
#plt.gca().set_ylim(nees_ci[0]*1e-1, max(nees_values)*1e+1)

plt.tight_layout()

plt.savefig('chi2.pdf', dpi=300, bbox_inches='tight')

plt.show()