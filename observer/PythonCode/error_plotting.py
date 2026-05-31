import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

resultsPath = "~/MSc/Maggy2026V/observer/MatlabCode/simulation_results/"

files = [
    "extended 6-state",
    "unscented 6-state",
    "linear 6-state"
]

observer_labels = {
    "extended 6-state": "EKF",
    "unscented 6-state": "UKF",
    "linear 6-state": "LKF"
}

LINEAR_LABEL = "linear"

VIOLIN_COLORS = {
    "EKF": "#4C72B0",
    "UKF": "#DD8452",
    "LKF": "#55A868"
}

states_position = [
    "err_x",
    "err_y",
    "err_z"
]

states_velocity = [
    "err_x_dot",
    "err_y_dot",
    "err_z_dot"
]

nameMap = {
    'err_x': r'$x$',
    'err_y': r'$y$',
    'err_z': r'$z$',
    'err_x_dot': r'${\dot{x}}$',
    'err_y_dot': r'${\dot{y}}$',
    'err_z_dot': r'${\dot{z}}$'
}

state_key = {
    'err_x':     'ex',
    'err_y':     'ey',
    'err_z':     'ez',
    'err_x_dot': 'exdot',
    'err_y_dot': 'eydot',
    'err_z_dot': 'ezdot',
}

VIOLIN_HALF_WIDTH = 0.35   # x-span of the stat lines


def load_data(file):

    csv_path = os.path.expanduser(
        resultsPath + file + "/simulation_results.csv"
    )

    return pd.read_csv(csv_path)


def calculate_statistics(data):

    rmse = np.sqrt(np.mean(np.square(data)))

    return {
        "mean": np.mean(data),
        "median": np.median(data),
        "std": np.std(data),
        "rmse": rmse
    }


def export_statistics_csv(dataframes, output_csv):
    """
    Wide format: one row per observer, columns grouped by state.
    6 states x 4 stats = 24 data columns.
    """
    file_by_observer = {observer_labels[f]: f for f in files}
    all_states = states_position + states_velocity

    rows = []
    for obs in ["EKF", "UKF", "LKF"]:
        row = {"Observer": obs}
        for state in all_states:
            data = dataframes[file_by_observer[obs]][state].values
            s = calculate_statistics(data)
            key = state_key[state]
            row[f"{key}Mean"]   = f"{s['mean']:+.4f}"
            row[f"{key}Median"] = f"{s['median']:+.4f}"
            row[f"{key}RMSE"]   = f"{s['rmse']:.4f}"
            row[f"{key}Std"]    = f"{s['std']:.4f}"
        rows.append(row)

    pd.DataFrame(rows).to_csv(output_csv, index=False)
    print(f"Saved statistics to {output_csv}")


def plot_group(states, figure_title):

    fig, axes = plt.subplots(
        nrows=3,
        ncols=1,
        figsize=(10, 10),
        sharex=True,
        constrained_layout=True
    )

    dataframes = {
        file: load_data(file)
        for file in files
    }

    for ax, state in zip(axes, states):

        ax_right = ax.twinx()

        stats_dict = {}

        left_positions = [1, 2]
        right_positions = [3]

        # EKF + UKF on left axis
        for pos, file in zip(left_positions, files[:2]):

            label = observer_labels[file]
            data = dataframes[file][state].values

            vp = ax.violinplot([data], positions=[pos],
                showmeans=False, showmedians=False, showextrema=False, points=len(data), side='high')
            
            ax.scatter(pos*np.ones_like(data)+np.random.uniform(-0.1,0.1,data.shape)-VIOLIN_HALF_WIDTH/3, data,
                             alpha=0.5, label="samples", color=VIOLIN_COLORS[label])


            for body in vp['bodies']:
                body.set_facecolor(VIOLIN_COLORS[label])
                body.set_edgecolor('black')
                body.set_alpha(0.75)

            median = np.median(data)
            mean = np.mean(data)
            
            ax.plot(pos, mean,
                    marker="*", color="red", ls='', label="mean", markersize=9)
            
            ax.plot(pos, median,
                    marker="D", color="black", ls='', label="median", markersize=9, zorder=1)

            stats_dict[label] = calculate_statistics(data)

            fig.subplots_adjust(bottom=0.06)
            if pos == 1 and "z" in state:
                ax.legend(ncols=3, bbox_to_anchor=(0.5, -0.005), loc='lower center')

        # LKF on right axis
        file = files[2]
        label = observer_labels[file]
        data = dataframes[file][state].values

        vp = ax_right.violinplot([data], positions=right_positions,
                showmeans=False, showmedians=False, showextrema=False, points=len(data), side='high')
        
        ax_right.scatter(right_positions*np.ones_like(data)+np.random.uniform(-0.1,0.1,data.shape)-VIOLIN_HALF_WIDTH/3, data,
                         alpha=0.5, label="samples", color=VIOLIN_COLORS[label])


        for body in vp['bodies']:
            body.set_facecolor(VIOLIN_COLORS[label])
            body.set_edgecolor('black')
            body.set_alpha(0.75)

        median = np.median(data)
        mean = np.mean(data)

        ax_right.plot(right_positions, mean,
                marker="*", color="red", ls='', label="mean", markersize=9)
            
        ax_right.plot(right_positions, median,
                marker="D", color="black", ls='', label="median", markersize=9, zorder=1)

        stats_dict[label] = calculate_statistics(data)

        ax.axhline(0, color='gray', linestyle='--', linewidth=1)

        # Separate LKF from the rest
        ax.axvline(np.mean([right_positions[-1], left_positions[-1]]), color=VIOLIN_COLORS["LKF"], linestyle='-', linewidth=1.5)
        plt.axvspan(2.5, 3.5, facecolor=VIOLIN_COLORS["LKF"], alpha=0.2)

        ax.grid(True, alpha=0.3)

        ax.set_xlim(0.5, 3.5)

        ax.set_xticks([1, 2, 3])
        ax.set_xticklabels(["EKF", "UKF", "LKF"])

        # Color LKF tick label
        for i, label in enumerate(ax.get_xticklabels()):
            if i == 2:
                label.set_color(VIOLIN_COLORS["LKF"])

        velocityUnit = ""
        if "dot" in state:
            velocityUnit = "/s"

        ax.set_ylabel(f'{nameMap[state]} [mm{velocityUnit}]  –  Extended / Unscented', fontsize=11)
        ax_right.set_ylabel(f'{nameMap[state]} [mm{velocityUnit}]  –  Linear', fontsize=11, color=VIOLIN_COLORS["LKF"])

        ax_right.tick_params(axis='y', colors=VIOLIN_COLORS["LKF"])

        
        # ── Stats text box ─────────────────────────────────────────────────────────
        lines = ["Observer  Mean    Median"]
        lines.append("─" * 22)
        for obs, statName in stats_dict.items():
            if obs == "LKF":
                if "dot" in state:
                    lines.append(f"{obs}   -         -   ")
                elif "z" in state:
                    lines.append(f"{obs} {statName['mean']:+.2f}    {statName['median']:+.2f}")
                else:
                    lines.append(f"{obs} {statName['mean']:+.3f}    {statName['median']:+.3f}")
            else:            
                lines.append(f"{obs} {statName['mean']:+.3f}    {statName['median']:+.3f}")
            

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

    axes[0].set_title(f"{figure_title}", fontsize=16)

    return fig


def main():

    dataframes = {
        file: load_data(file)
        for file in files
    }

    export_statistics_csv(dataframes, "observer_statistics.csv")

    fig_pos = plot_group(states_position, "Positional Errors")

    fig_vel = plot_group(states_velocity, "Velocity Errors")

    fig_pos.savefig( "position_error_comparison.pdf", dpi=300, bbox_inches='tight')

    fig_vel.savefig( "velocity_error_comparison.pdf", dpi=300, bbox_inches='tight')

    plt.show()


if __name__ == "__main__":
    main()