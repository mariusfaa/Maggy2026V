import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
import matplotlib.patches as mpatches
import numpy as np
import os

resultsPath = "~/MSc/Maggy2026V/observer/PCobserver/results/simulation/"

files = ["linear 6-state",
         "linear 10-state",
         "extended 6-state",
         "extended 10-state",
         "unscented 6-state",
         "unscented 10-state"
         ]

# Which files go on the right (secondary) y-axis
LINEAR_LABEL = 'linear'

VIOLIN_COLORS = ['#4C72B0', '#DD8452', '#55A868', '#C44E52', '#8172B2', '#937860']

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

        parts = ax.violinplot([data], positions=[pos],
                              showmeans=False, showmedians=False, showextrema=False)

        for pc in parts['bodies']:
            pc.set_facecolor(VIOLIN_COLORS[i % len(VIOLIN_COLORS)])
            pc.set_edgecolor('black')
            pc.set_linewidth(0.8)
            pc.set_alpha(0.72)

        s = all_stats[file]
        lo, hi = pos - VIOLIN_HALF_WIDTH, pos + VIOLIN_HALF_WIDTH
        ax.hlines(s['median'], lo, hi, **MEDIAN_STYLE)
        ax.hlines(s['mean'],   lo, hi, **MEAN_STYLE)

    # ── Axis labels & ticks ────────────────────────────────────────────────────
    ax_right.set_xticks(positions)
    ax_right.set_xticklabels(valid_files, rotation=15, ha='right', fontsize=10)

    for i, label in enumerate(ax_left.get_xticklabels()):
        if i in [0, 1]:
            label.set_color(VIOLIN_COLORS[1])

    ax_left.set_ylabel('Runtime [μs]  –  Extended / Unscented', fontsize=11)
    ax_right.set_ylabel('Runtime [μs]  –  Linear', fontsize=11,
                        color=VIOLIN_COLORS[1])          # tint to match its violins
    ax_right.tick_params(axis='y', labelcolor=VIOLIN_COLORS[1])

    ax_left.set_title('Runtime Comparison Across Filters', fontsize=14)
    ax_left.grid(True, alpha=0.3, axis='y')

    # Push x-limits slightly so violins at pos 1 and pos N aren't clipped
    ax_left.set_xlim(0.3, len(valid_files) + 0.7)

    # ── Legend ─────────────────────────────────────────────────────────────────
    violin_patches = [
        mpatches.Patch(facecolor=VIOLIN_COLORS[i % len(VIOLIN_COLORS)],
                       edgecolor='black', alpha=0.72, label=file)
        for i, file in enumerate(valid_files)
    ]
    stat_lines = [
        mlines.Line2D([], [], **MEDIAN_STYLE, label='Median'),
        mlines.Line2D([], [], **MEAN_STYLE,   label='Mean'),
    ]

    fig.subplots_adjust(bottom=0.06)
    ax_left.legend(handles=stat_lines, ncols=2, bbox_to_anchor=(0.5, -0.005), loc='lower center')
    # ── Stats text box ─────────────────────────────────────────────────────────
    lines = ["Filter               Median    Mean"]
    lines.append("─" * 42)
    for i, file in enumerate(valid_files):
        s = all_stats[file]
        lines.append(f"{file:<22} {s['median']:>6.2f}    {s['mean']:>6.2f}")

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