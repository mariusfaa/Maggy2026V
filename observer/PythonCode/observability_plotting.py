#!/usr/bin/env python3
"""
plot_obsv.py
Reads obsv_eigenvalues.csv and obsv_cond_time.csv exported from MATLAB.
 
Figure layout
─────────────
  Fig 1 – Combined CT stem plot  (all three systems on one axes)
  Fig 2 – Combined DT stem plot  (all three systems on one axes)
  Fig 3 – Condition number κ(Wₒ) vs time (three subplots)
 
Canonical index mapping (eigenvalues sorted descending → position in 1..12)
  12-state : 1  2  3  4  5  6  7  8  9 10 11 12
  10-state : 1  2  3  4  5     7  8  9 10 11        (removes 6, 12)
   6-state : 1  2  3            7  8  9              (removes 4,5,6,10,11,12)
"""
 
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker
from matplotlib.lines import Line2D
 
# ─────────────────────────────────────────────────────────────────────────── #
#  Load data
# ─────────────────────────────────────────────────────────────────────────── #
eig_df  = pd.read_csv('obsv_eigenvalues.csv')
cond_df = pd.read_csv('obsv_cond_time.csv')
 
systems = list(dict.fromkeys(eig_df['system']))   # preserve insertion order
n_sys   = len(systems)
 
# ─────────────────────────────────────────────────────────────────────────── #
#  Canonical index mapping
#  The j-th sorted eigenvalue of each system occupies the j-th slot below.
# ─────────────────────────────────────────────────────────────────────────── #
CANON_IDX = {
    '12-state': [1, 2, 3, 4, 5,  6,  7, 8, 9, 10, 11, 12],
    '10-state': [1, 2, 3, 4, 5,      7, 8, 9, 10, 11     ],
    '6-state':  [1, 2, 3,             7, 8, 9              ],
}
 
# ─────────────────────────────────────────────────────────────────────────── #
#  Per-system style table
#  Four independent visual channels: color · line-width · line-style · marker
# ─────────────────────────────────────────────────────────────────────────── #
STYLES = {
    '12-state': dict(color='#d62728', lw=1.4, ls='-',   marker='o', ms=7,  offset=-0.20),
    '10-state': dict(color='#2ca02c', lw=1.2, ls='--',  marker='^', ms=6,  offset= 0.0 ),
    '6-state':  dict(color='#1f77b4', lw=1.0, ls=':',   marker='s', ms=6,  offset= 0.20),
}
 
ALL_CANON  = list(range(1, 13))
 
# ─────────────────────────────────────────────────────────────────────────── #
#  Global style
# ─────────────────────────────────────────────────────────────────────────── #
plt.rcParams.update({
    'font.size':       11,
    'axes.titlesize':  12,
    'axes.labelsize':  11,
    'legend.fontsize': 10,
    'xtick.labelsize':  9,
    'ytick.labelsize':  9,
})
 
CT_COLOR = '#1f77b4'
DT_COLOR = '#d62728'
 
 
def lambda_labels(indices):
    return [rf'$\lambda_{{{k}}}$' for k in indices]
 
 
# ─────────────────────────────────────────────────────────────────────────── #
#  Combined stem plot
# ─────────────────────────────────────────────────────────────────────────── #
def combined_stem(ax, sig_type, ylim):
    """
    All three systems on one axes.  Each system uses its canonical x-positions
    (1..12 space), with a small per-system x-offset so overlapping stems are
    visually separated.  Stems: vlines + scatter for full style control.
    """
    ybase = ylim[0]
 
    for name in systems:
        st  = STYLES[name]
        sub = (eig_df[(eig_df['system'] == name) & (eig_df['type'] == sig_type)]
               .sort_values('index'))
        vals  = np.maximum(sub['value'].values, 1e-300)
        x_pos = np.array(CANON_IDX[name], dtype=float) + st['offset']
 
        # Vertical stem lines
        ax.vlines(x_pos, ybase, vals,
                  color=st['color'], linewidth=st['lw'],
                  linestyle=st['ls'], alpha=0.85, zorder=2)
 
        # Marker at tip – also registers the legend entry
        ax.plot(x_pos, vals,
                marker=st['marker'], color=st['color'],
                markersize=st['ms'], linestyle='none',
                label=name, zorder=3)
 
    # Baseline at bottom of y-range
    ax.axhline(ybase, color='#333333', linewidth=0.8, zorder=1)
 
    ax.set_yscale('log')
    ax.set_ylim(ylim)
    ax.set_xlim([0.5, 12.5])
    ax.set_xticks(ALL_CANON)
    ax.set_xticklabels(lambda_labels(ALL_CANON))
    ax.set_xlabel('Eigenvalue index')
    ax.set_ylabel('Eigenvalue')
    ax.yaxis.set_major_locator(mticker.LogLocator(numticks=8))
    ax.grid(True, which='both', alpha=0.3, linewidth=0.6)
    ax.legend(loc='upper right', framealpha=0.9)
 
 
# ─────────────────────────────────────────────────────────────────────────── #
#  Figure 1 – Combined Continuous-Time Spectrum
# ─────────────────────────────────────────────────────────────────────────── #
fig1, ax1 = plt.subplots(figsize=(12, 5))
combined_stem(ax1, 'ct', ylim=(1e-20, 1.0))
fig1.suptitle('Continuous-Time Observability Spectrum',
              fontsize=14)
fig1.tight_layout()
fig1.savefig('obsv_spectra_cont.pdf', dpi=300, bbox_inches='tight')
print('Saved obsv_spectra_cont.pdf')
 
# ─────────────────────────────────────────────────────────────────────────── #
#  Figure 2 – Combined Discrete-Time Spectrum
# ─────────────────────────────────────────────────────────────────────────── #
fig2, ax2 = plt.subplots(figsize=(12, 5))
combined_stem(ax2, 'dt', ylim=(1e-18, 1e2))
fig2.suptitle('Discrete-Time Observability Spectrum',
              fontsize=14)
fig2.tight_layout()
fig2.savefig('obsv_spectra_disc.pdf', dpi=150, bbox_inches='tight')
print('Saved  obsv_spectra_disc.pdf')


# --------------------------------------------------------------------------- #
#  Figure 3 – Condition Number κ(Wo) vs Time
# --------------------------------------------------------------------------- #
fig3, ax = plt.subplots(figsize=(13, 7))

# Define markers and line styles for CT and DT
colors = ['red', 'green', 'blue']
markers = ['o', '^', 's']  # circle, triangle, diamond

for idx, name in enumerate(systems):
    ct = cond_df[(cond_df['system'] == name) & (cond_df['type'] == 'ct')]
    dt = cond_df[(cond_df['system'] == name) & (cond_df['type'] == 'dt')]

    # Find minimum for CT
    ct_cond = ct['cond'].values[39:]
    ct_time = ct['time'].values[39:]

    ct_min_idx = ct_cond.argmin()
    ct_min_time = ct_time[ct_min_idx]
    ct_min_val = ct_cond[ct_min_idx]
    
    # Vertical line and annotation for CT
    ax.axvline(x=ct_min_time, color=colors[idx], linestyle=':', alpha=0.5, linewidth=1)
    ax.annotate(f'(min CT {ct_min_time:.2f}, {ct_min_val:.2e})',
                xy=(ct_min_time, ct_min_val),
                xytext=(10, 10), textcoords='offset points',
                fontsize=7, color=colors[idx],
                bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.7))
    ax.plot(ct_min_time, ct_min_val,
            marker="*", color=colors[idx], ls='', markersize=12, zorder=10)

    # CT: solid line with distinct marker
    ax.semilogy(ct['time'].values, ct['cond'].values,
                color=colors[idx], linewidth=1.5, marker=markers[idx], markersize=4,
                markevery=15, label=f'{name} CT', linestyle='-')
    
    # Find minimum for DT
    dt_cond = dt['cond'].values[3:]
    dt_time = dt['time'].values[3:]

    dt_min_idx = dt_cond.argmin()
    dt_min_time = dt_time[dt_min_idx]
    dt_min_val = dt_cond[dt_min_idx]

    # Vertical line and annotation for DT
    ax.axvline(x=dt_min_time, color=colors[idx], linestyle=':', alpha=0.5, linewidth=1)
    ax.annotate(f'(min DT {dt_min_time:.2f}, {dt_min_val:.2e})',
                xy=(dt_min_time, dt_min_val*5 if '6' in name else dt_min_val),
                xytext=(10, 10), textcoords='offset points',
                fontsize=7, color=colors[idx],
                bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.7))
    ax.plot(dt_min_time, dt_min_val,
            marker="*", color=colors[idx], ls='', markersize=12, zorder=10)
    
    # DT: dashed line with distinct marker
    ax.step(dt['time'].values, dt['cond'].values,
            color=colors[idx], linewidth=1.2, marker=markers[idx], markersize=4,
            markevery=15, label=f'{name} DT', where='post', linestyle='--')

ax.set_xlabel('Time (s)')
ax.set_ylabel(r'$\kappa(W_o)$')
ax.set_xlim(left=0.02)
ax.legend(loc='best', framealpha=0.8)
ax.yaxis.set_major_locator(mticker.LogLocator(numticks=8))
ax.grid(True, which='both', alpha=0.3, linewidth=0.6)

fig3.suptitle(r'Observability Gramian Condition Number Over Time',
              fontsize=14)
fig3.tight_layout()
fig3.savefig('obsv_cond_time.pdf', dpi=300, bbox_inches='tight')
print('Saved  obsv_cond_time.pdf')

plt.show()