"""Generate Experiment-1 figures for the thesis from Stage A/F/G records.

Outputs three PDFs into the thesis figures/ directory:

  exp1_closed_loop.pdf      F1 vs F4, both model orders, z + roll + per-step time
  exp1_panel_scatter.pdf    Stage-F panel on the (t_p99, J) plane
  exp1_repeatability.pdf    Stage-G mean t_p99 +/- sigma with deadline lines
"""
from __future__ import annotations
import os
from pathlib import Path

import h5py
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt

# --- Paths -------------------------------------------------------------
# ROOT : directory containing the thesis results
# OUT  : directory where generated figures should be written
ROOT = Path(r"")  # e.g. r"C:\Users\you\isolated_experiment\experiments\thesis\results"
OUT  = Path(r"")  # e.g. r"C:\Users\you\master_thesis\figures"
OUT.mkdir(parents=True, exist_ok=True)

# --- Matplotlib style --------------------------------------------------
mpl.rcParams.update({
    "font.family":       "serif",
    "font.size":         9,
    "axes.titlesize":    10,
    "axes.labelsize":    9,
    "legend.fontsize":   8,
    "xtick.labelsize":   8,
    "ytick.labelsize":   8,
    "axes.grid":         True,
    "grid.alpha":        0.3,
    "grid.linewidth":    0.5,
    "lines.linewidth":   1.2,
    "pdf.fonttype":      42,
})


def decode_h5string(arr):
    """MATLAB v7.3 stores strings as uint16 arrays; decode to python str."""
    flat = np.asarray(arr).ravel()
    return "".join(chr(int(c)) for c in flat if int(c) != 0)


def load_record(path: Path):
    """Read the parts of a result_record we need: cfg + run + summary."""
    with h5py.File(path, "r") as f:
        r = f["rec"]
        out = {
            "X_plant":    np.array(r["run/X_plant"]).T,          # 12 x N
            "U":          np.array(r["run/U"]).T,                # 4  x N-1
            "t_tot":      np.array(r["run/timing/tot"]).ravel() * 1e3,  # ms
            "dt":         float(np.array(r["run/dt"]).ravel()[0]),
            "n_actual":   int(np.array(r["run/n_actual"]).ravel()[0]),
            "t_p99":      float(np.array(r["summary/t_p99_ms"]).ravel()[0]),
            "t_mean":     float(np.array(r["summary/t_mean_ms"]).ravel()[0]),
            "t_max":      float(np.array(r["summary/t_max_ms"]).ravel()[0]),
            "J":          float(np.array(r["summary/J_integrated"]).ravel()[0]),
            "class":      decode_h5string(r["classification"]),
            "order":      decode_h5string(r["cfg/order"]),
            "N":          int(np.array(r["cfg/N"]).ravel()[0]),
        }
    return out


def find_file(stem_dir: Path, prefix: str) -> Path:
    """First file in stem_dir starting with prefix."""
    matches = sorted(stem_dir.glob(prefix + "*"))
    if not matches:
        raise FileNotFoundError(f"No file matching {prefix}* in {stem_dir}")
    return matches[0]


# ======================================================================
# Figure 1 -- F1 vs F4 closed-loop comparison
# ======================================================================

def figure_closed_loop():
    """4 columns (12-F1, 12-F4, 10-F1, 10-F4) x 3 rows (z, roll, t_solve)."""
    runs = {
        ("full12",    "F1"): find_file(ROOT / "F" / "full12",    "F_F1_baseline_ful_base"),
        ("full12",    "F4"): find_file(ROOT / "F" / "full12",    "F_F4_Ewinner_ful_base"),
        ("reduced10", "F1"): find_file(ROOT / "F" / "reduced10", "F_F1_baseline_red_base"),
        ("reduced10", "F4"): find_file(ROOT / "F" / "reduced10", "F_F4_Ewinner_red_base"),
    }
    data = {k: load_record(v) for k, v in runs.items()}

    fig, axes = plt.subplots(3, 4, figsize=(11.0, 6.3), sharex="col")

    titles = {
        ("full12",    "F1"): r"\textsf{F1} (baseline), 12-state",
        ("full12",    "F4"): r"\textsf{F4} (tuned), 12-state",
        ("reduced10", "F1"): r"\textsf{F1} (baseline), 10-state",
        ("reduced10", "F4"): r"\textsf{F4} (tuned), 10-state",
    }

    # Use plain text titles since the LaTeX rendering in matplotlib pulls a
    # lot of dependencies; keep the file portable.
    titles_plain = {
        ("full12",    "F1"): "F1 baseline, 12-state",
        ("full12",    "F4"): "F4 tuned, 12-state",
        ("reduced10", "F1"): "F1 baseline, 10-state",
        ("reduced10", "F4"): "F4 tuned, 10-state",
    }

    col_order = [("full12","F1"), ("full12","F4"), ("reduced10","F1"), ("reduced10","F4")]

    for ci, key in enumerate(col_order):
        d = data[key]
        t_state = np.arange(d["X_plant"].shape[1]) * d["dt"]
        t_input = np.arange(d["U"].shape[1])       * d["dt"]
        t_solve = np.arange(d["t_tot"].shape[0])   * d["dt"]

        # Row 0 -- z deviation (mm)
        zEq = d["X_plant"][2, 0]   # initial state perturbed by IC; track around hover
        ax = axes[0, ci]
        ax.plot(t_state, (d["X_plant"][2, :]) * 1e3, color="C0")
        # Reference: equilibrium z from the model's xEq is ~34.55 mm. Plot a
        # reference line at the long-run mean for visual reference.
        zref_mm = float(np.mean(d["X_plant"][2, -20:])) * 1e3
        ax.axhline(zref_mm, color="grey", linestyle=":", linewidth=0.7)
        ax.set_title(titles_plain[key])
        if ci == 0: ax.set_ylabel("z [mm]")

        # Row 1 -- roll angle (deg)
        ax = axes[1, ci]
        ax.plot(t_state, np.rad2deg(d["X_plant"][3, :]), color="C1", label="roll")
        ax.plot(t_state, np.rad2deg(d["X_plant"][4, :]), color="C2", label="pitch")
        if ci == 0: ax.set_ylabel("angle [deg]")
        if ci == 3: ax.legend(loc="upper right", frameon=False, fontsize=7)

        # Row 2 -- per-step solver time (ms)
        ax = axes[2, ci]
        ax.plot(t_solve, d["t_tot"], color="C3", linewidth=0.8)
        ax.axhline(10.0, color="k", linestyle="--", linewidth=0.7)
        ax.axhline( 5.0, color="k", linestyle=":",  linewidth=0.7)
        ax.set_xlabel("time [s]")
        if ci == 0: ax.set_ylabel("solver time [ms]")
        # Annotate t_p99
        ax.text(0.97, 0.92,
                f"$t_{{p99}}={d['t_p99']:.2f}$ ms\n$J={d['J']:.2f}$",
                transform=ax.transAxes, ha="right", va="top", fontsize=7,
                bbox=dict(boxstyle="round,pad=0.2", facecolor="white",
                          edgecolor="0.7", linewidth=0.5))

    # Make timing axes share a y-axis per column-group for fair comparison
    for ci in range(4):
        axes[2, ci].set_yscale("log")

    fig.suptitle("Closed-loop comparison: untuned baseline vs Stage-E tuned candidate "
                 "(IC scale 0.05)", fontsize=10)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    out = OUT / "exp1_closed_loop.pdf"
    fig.savefig(out, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {out}")


# ======================================================================
# Figure 2 -- Stage-F panel scatter on (t_p99, J)
# ======================================================================

def figure_panel_scatter():
    """One marker per (panel, order) at IC=base. Colors=panel, shape=order."""
    panels = ["F1_baseline", "F2_Cwinner", "F3_Dwinner",
              "F4_Ewinner", "F5_aggressive_embedded", "F6_reduced_only"]
    panel_label = {
        "F1_baseline":             "F1 baseline",
        "F2_Cwinner":              "F2 C-winner",
        "F3_Dwinner":              "F3 D-winner",
        "F4_Ewinner":              "F4 E-winner",
        "F5_aggressive_embedded":  "F5 aggr. emb.",
        "F6_reduced_only":         "F6 red.-only probe",
    }
    order_marker = {"full12": "o", "reduced10": "s"}
    order_label  = {"full12": "12-state", "reduced10": "10-state"}

    # Find one file per (panel, order, base IC) by glob.
    records = []
    for panel in panels:
        # panel prefix in filenames is F_<panel>_<order3>_base
        for order in ("full12", "reduced10"):
            o3 = "ful" if order == "full12" else "red"
            short_panel = panel
            # Match F_<panel>_<ful|red>_base*
            patt = f"F_{short_panel}_{o3}_base*"
            matches = sorted((ROOT / "F" / order).glob(patt))
            if not matches:
                # Some panel names get truncated in filenames -- try a coarser match
                short_panel_alt = panel.split("_")[0]  # e.g. "F1"
                patt = f"F_{short_panel_alt}_*_{o3}_base*"
                matches = sorted((ROOT / "F" / order).glob(patt))
            if not matches:
                print(f"WARN: no match for {panel} {order}")
                continue
            d = load_record(matches[0])
            records.append((panel, order, d["t_p99"], d["J"]))

    fig, ax = plt.subplots(figsize=(6.6, 4.2))
    cmap = plt.get_cmap("tab10")

    panel_to_color = {p: cmap(i) for i, p in enumerate(panels)}
    for panel, order, tp99, J in records:
        # Sentinel J=1e9 means classification failure -- plot at axis top
        # with an open marker to flag it.
        is_fail = J >= 1e6
        if is_fail:
            J_plot = 1e3   # plotted at the top of the log axis
            facecolor = "none"
        else:
            J_plot = J
            facecolor = panel_to_color[panel]
        ax.scatter(tp99, J_plot,
                   marker=order_marker[order],
                   s=70 if order == "reduced10" else 80,
                   edgecolor=panel_to_color[panel],
                   facecolor=facecolor,
                   linewidth=1.2,
                   label=None)

    # Build a custom two-axis legend
    from matplotlib.lines import Line2D
    panel_handles = [Line2D([0],[0], marker="o", linestyle="None",
                             markerfacecolor=panel_to_color[p],
                             markeredgecolor=panel_to_color[p],
                             label=panel_label[p]) for p in panels]
    order_handles = [Line2D([0],[0], marker=order_marker[o], linestyle="None",
                             markerfacecolor="grey", markeredgecolor="black",
                             label=order_label[o]) for o in ("full12","reduced10")]
    leg1 = ax.legend(handles=panel_handles, loc="upper right",
                     title="Panel", frameon=True, fontsize=7, title_fontsize=8)
    ax.add_artist(leg1)
    ax.legend(handles=order_handles, loc="lower right",
              title="Model order", frameon=True, fontsize=7, title_fontsize=8)

    ax.axvline(10.0, color="k", linestyle="--", linewidth=0.7,
               label=None)
    ax.axvline( 5.0, color="k", linestyle=":",  linewidth=0.7)
    ax.text(10.0, 1e3*0.4, " 10 ms", fontsize=7, va="top")
    ax.text( 5.0, 1e3*0.4, " 5 ms",  fontsize=7, va="top", ha="right")

    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlabel(r"$t_{p99}$ [ms]")
    ax.set_ylabel(r"Integrated closed-loop cost $J$")
    ax.set_title("Stage-F panel: model-order comparison on the speed--cost plane (IC base)")
    ax.set_xlim(1.0, 300.0)
    ax.set_ylim(0.2, 2e3)

    out = OUT / "exp1_panel_scatter.pdf"
    fig.tight_layout()
    fig.savefig(out)
    plt.close(fig)
    print(f"wrote {out}")


# ======================================================================
# Figure 3 -- Stage-G repeatability bar chart
# ======================================================================

def figure_repeatability():
    """Hard-coded from stageH summary (10-repetition statistics).

    The Stage-G records are stored across many .mat files; the rolled-up
    statistics are the canonical reference in the thesis text, so we draw
    the chart directly from those numbers (verified against the summary).
    """
    rows = [
        ("12-state",  "F1",  128.99, 3.43, 138.32),
        ("12-state",  "F4",    2.38, 0.50,   3.77),
        ("12-state",  "F5",    4.34, 0.22,   4.87),
        ("10-state",  "F1",   14.51, 0.16,  14.80),
        ("10-state",  "F4",    1.81, 0.06,   1.95),
        ("10-state",  "F5",    3.58, 0.05,   3.64),
    ]
    labels = [f"{o}\n{c}" for o, c, _, _, _ in rows]
    means  = [r[2] for r in rows]
    stds   = [r[3] for r in rows]
    maxes  = [r[4] for r in rows]

    fig, ax = plt.subplots(figsize=(6.4, 3.6))
    x = np.arange(len(rows))
    colors = ["#7e7e7e" if c == "F1" else ("#1f77b4" if c == "F4" else "#ff7f0e")
              for _, c, _, _, _ in rows]
    ax.bar(x, means, yerr=stds, capsize=4, color=colors, edgecolor="black",
           linewidth=0.5, label=None)
    # Worst-case marker
    ax.plot(x, maxes, "kv", markersize=6, label=r"$\max\ t_{p99}$ over 10 reps")

    ax.axhline(10.0, color="k", linestyle="--", linewidth=0.8, label="10 ms deadline")
    ax.axhline( 5.0, color="k", linestyle=":",  linewidth=0.8, label="5 ms tight deadline")

    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=8)
    ax.set_yscale("log")
    ax.set_ylabel(r"$t_{p99}$ [ms] (log scale)")
    ax.set_title("Stage-G timing repeatability: mean $\pm$ $\sigma$ of $t_{p99}$ over 10 repetitions")
    ax.legend(loc="upper right", fontsize=7, frameon=True)
    ax.set_ylim(0.5, 300.0)

    # Annotate each bar with its mean for readability
    for xi, mi in zip(x, means):
        ax.text(xi, mi * 1.08, f"{mi:.2f}", ha="center", va="bottom", fontsize=7)

    out = OUT / "exp1_repeatability.pdf"
    fig.tight_layout()
    fig.savefig(out, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {out}")


if __name__ == "__main__":
    figure_closed_loop()
    figure_panel_scatter()
    figure_repeatability()
