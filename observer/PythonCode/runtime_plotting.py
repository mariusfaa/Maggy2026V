import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
import matplotlib.patches as mpatches
import numpy as np
import os

resultsPath = "~/MSc/Maggy2026V/observer/MatlabCode/simulation_results/"

files = ["extended 6-state",
         "extended 10-state",
         "unscented 6-state",
         "unscented 10-state",
         "linear 6-state",
         "linear 10-state"
         ]

# Which files go on the right (secondary) y-axis
LINEAR_LABEL = 'linear'

VIOLIN_COLORS = {
    "EKF": "#4C72B0",
    "UKF": "#DD8452",
    "LKF": "#55A868"
}

MEDIAN_STYLE = dict(color='black',   linewidth=2.0, linestyle='-',  zorder=6)
MEAN_STYLE   = dict(color='#d62728', linewidth=2.0, linestyle='--', zorder=6)

VIOLIN_HALF_WIDTH = 0.35   # x-span of the stat lines


def load_runtime_data(csv):
    df = pd.read_csv(csv)
    runtime = df['runtime'].values
    return runtime[1:]          # drop first element (always 0)


def calculate_statistics(data):
    return {
        'mean':   np.mean(data),
        'median': np.median(data),
        'std':    np.std(data),
        'min':    np.min(data),
        'max':    np.max(data),
        'q1':     np.percentile(data, 25),
        'q3':     np.percentile(data, 75),
        'count':  len(data),
    }


def plot_runtime_comparison(files, resultsPath, figsize=(13, 7)):
    # ── Load data ──────────────────────────────────────────────────────────────
    all_runtimes, valid_files, all_stats = [], [], {}

    for file in files:
        csv_path = os.path.expanduser(resultsPath + file + "/simulation_results.csv")
        try:
            data = load_runtime_data(csv_path)
            all_runtimes.append(data)
            valid_files.append(file)
            all_stats[file] = calculate_statistics(data)
            print(f"Loaded {file}: {len(data)} samples")
        except FileNotFoundError:
            print(f"Warning: File not found – {csv_path}")
        except Exception as e:
            print(f"Error loading {file}: {e}")

    if not all_runtimes:
        raise ValueError("No valid data files found")

    # ── Axis assignment ────────────────────────────────────────────────────────
    linear_idx = [i for i, f in enumerate(valid_files) if LINEAR_LABEL in f.lower()]
    other_idx  = [i for i, f in enumerate(valid_files) if LINEAR_LABEL not in f.lower()]

    positions = np.arange(1, len(all_runtimes) + 1)

    # ── Figure ─────────────────────────────────────────────────────────────────
    fig, ax_left = plt.subplots(figsize=figsize)
    ax_right = ax_left.twinx()

    # ── Draw violins ───────────────────────────────────────────────────────────
    for i, (data, pos, file) in enumerate(zip(all_runtimes, positions, valid_files)):
        ax = ax_right if i in linear_idx else ax_left

        if "extended" in file:
            plot_color = VIOLIN_COLORS["EKF"]
        elif "unscented" in file:
            plot_color = VIOLIN_COLORS["UKF"]
        elif "linear" in file:
            plot_color = VIOLIN_COLORS["LKF"]

        if i == 0: plt.axvspan(4.5, 6.7, facecolor=VIOLIN_COLORS["LKF"], alpha=0.2) # 67!!

        parts = ax.violinplot([data], positions=[pos],
                              showmeans=False, showmedians=False, showextrema=False, points=len(data), side='high')
        # Thank you Peder and Bjørnar @PVV for the idea of diffusing the x axis in the scatterplot!
        ax.scatter(pos*np.ones_like(data)+np.random.uniform(-0.1,0.1,data.shape)-VIOLIN_HALF_WIDTH/3, data,
                   alpha=0.5, label="samples", color=plot_color)
                   
        for pc in parts['bodies']:
            if "extended" in file:
                pc.set_facecolor(plot_color)
            elif "unscented" in file:
                pc.set_facecolor(plot_color)
            elif "linear" in file:
                pc.set_facecolor(plot_color)
            
            pc.set_edgecolor('black')
            pc.set_linewidth(0.8)
            pc.set_alpha(0.72)

        s = all_stats[file]
        #lo, hi = pos - VIOLIN_HALF_WIDTH, pos + VIOLIN_HALF_WIDTH
        #ax.hlines(s['median'], lo, hi, **MEDIAN_STYLE)
        #ax.hlines(s['mean'],   lo, hi, **MEAN_STYLE)
        ax.plot(pos, s['mean'],
                marker="*", color="red", ls='', label="mean", markersize=9)
        ax.plot(pos, s['median'],
                marker="D", color="black", ls='', label="median", markersize=9, zorder=1)
        if i == 0:
            ax.legend(ncols=3, bbox_to_anchor=(0.5, -0.005), loc='lower center')

    # Separate LKF by vertical line
    ax.axvline(np.mean([positions[-2], positions[-3]]), color=VIOLIN_COLORS["LKF"], linestyle='-', linewidth=1.5)

    # ── Axis labels & ticks ────────────────────────────────────────────────────
    ax_right.set_xticks(positions)
    ax_right.set_xticklabels(valid_files, rotation=15, ha='right', fontsize=10)

    for i, label in enumerate(ax_left.get_xticklabels()):
        if i in [4, 5]:
            label.set_color(VIOLIN_COLORS["LKF"])

    ax_left.set_ylabel('Runtime [μs]  –  Extended / Unscented', fontsize=11)
    ax_right.set_ylabel('Runtime [μs]  –  Linear', fontsize=11,
                        color=VIOLIN_COLORS["LKF"])          # tint to match its violins
    ax_right.tick_params(axis='y', labelcolor=VIOLIN_COLORS["LKF"])

    ax_left.set_title('Runtime Across Observers', fontsize=14)
    ax_left.grid(True, alpha=0.3, axis='y')

    # Push x-limits slightly so violins at pos 1 and pos N aren't clipped
    ax_left.set_xlim(0.3, len(valid_files) + 0.7)


    fig.subplots_adjust(bottom=0.06)
    #ax_left.legend(handles=stat_lines, ncols=2, bbox_to_anchor=(0.5, -0.005), loc='lower center')
    # ── Stats text box ─────────────────────────────────────────────────────────
    lines = ["Observer             Mean    Median"]
    lines.append("─" * 38)
    for i, file in enumerate(valid_files):
        s = all_stats[file]
        lines.append(f"{file:<22} {s['mean']:>6.2f}    {s['median']:>6.2f}")

    textstr = "\n".join(lines)
    ax_right.text(
        0.98, 0.97, textstr,
        transform=ax_right.transAxes,
        fontsize=8,
        verticalalignment='top',
        horizontalalignment='right',
        family='monospace',
        bbox=dict(boxstyle='round,pad=0.5', facecolor='white', edgecolor='gray', alpha=0.85)
    )
    plt.tight_layout()
    plt.show()

    return fig, all_stats


def main():
    fig, stats = plot_runtime_comparison(files, resultsPath)
    fig.savefig('runtime.pdf', dpi=300, bbox_inches='tight')


if __name__ == "__main__":
    main()