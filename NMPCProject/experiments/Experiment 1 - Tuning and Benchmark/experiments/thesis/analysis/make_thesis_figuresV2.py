"""Generate Experiment-1 thesis figures with thesis-matched fonts.

This is a V2 replacement for make_thesis_figures.py. It keeps the same Stage-F
and Stage-H data sources but renders with Matplotlib's PGF backend by default,
so plot text is typeset by LaTeX with the thesis font stack (T1 + lmodern).

  exp1_closed_loop_F1.pdf   F1 baseline, 12-state vs 10-state
  exp1_closed_loop_F4.pdf   F4 tuned,    12-state vs 10-state
  exp1_panel_scatter.pdf    Stage-F panel on the (t_p99, J) plane
  exp1_repeatability.pdf    Stage-G repeatability summary

Use --no-latex for quick data/debug runs if the local Python environment cannot
find pdflatex.
"""
from __future__ import annotations

import argparse
from pathlib import Path

import h5py
import matplotlib as mpl
import numpy as np


# --- Enter your own paths here ---
# ROOT : directory containing the thesis results
# OUT  : directory where generated figures should be written
ROOT = Path(r"")  # e.g. r"C:\Users\you\isolated_experiment\experiments\thesis\results"
OUT = Path(r"")  # e.g. r"C:\Users\you\master_thesis\figures"

ORDER_LABEL = {
    "full12": "12-state",
    "reduced10": "10-state",
}

ORDER_DIR = {
    "full12": "full12",
    "reduced10": "reduced10",
}

RUN_PREFIX = {
    ("full12", "F1"): "F_F1_baseline_ful_base",
    ("reduced10", "F1"): "F_F1_baseline_red_base",
    ("full12", "F4"): "F_F4_Ewinner_ful_base",
    ("reduced10", "F4"): "F_F4_Ewinner_red_base",
}

PANELS = [
    "F1_baseline",
    "F2_Cwinner",
    "F3_Dwinner",
    "F4_Ewinner",
    "F5_aggressive_embedded",
    "F6_reduced_only",
]

PANEL_LABEL = {
    "F1_baseline": "F1 baseline",
    "F2_Cwinner": "F2 C-winner",
    "F3_Dwinner": "F3 D-winner",
    "F4_Ewinner": "F4 E-winner",
    "F5_aggressive_embedded": "F5 aggressive",
    "F6_reduced_only": "F6 reduced-only",
}

PAGE_CONFIG = {
    "F1": {
        "descriptor": "baseline",
        "suptitle": "Closed-loop comparison: F1 baseline",
        "outfile": "exp1_closed_loop_F1.pdf",
    },
    "F4": {
        "descriptor": "tuned",
        "suptitle": "Closed-loop comparison: F4 tuned",
        "outfile": "exp1_closed_loop_F4.pdf",
    },
}

COLORS = {
    "z": "#2C5F9E",
    "roll": "#D98E2B",
    "pitch": "#2E8B57",
    "time": "#C0392B",
    "ref": "#777777",
}


def configure_matplotlib(use_latex: bool):
    """Configure Matplotlib and return pyplot after the backend is selected."""
    if use_latex:
        mpl.use("pgf")
        latex_preamble = "\n".join(
            [
                r"\usepackage[T1]{fontenc}",
                r"\usepackage{lmodern}",
                r"\usepackage{amsmath}",
            ]
        )
        mpl.rcParams.update(
            {
                "pgf.texsystem": "pdflatex",
                "pgf.rcfonts": False,
                "pgf.preamble": latex_preamble,
                "text.usetex": True,
                "text.latex.preamble": latex_preamble,
                "font.family": "serif",
                "font.serif": ["Latin Modern Roman", "Computer Modern Roman", "DejaVu Serif"],
                "font.size": 9,
            }
        )
    else:
        mpl.use("Agg")
        mpl.rcParams.update(
            {
                "font.family": "serif",
                "font.serif": ["Latin Modern Roman", "Computer Modern Roman", "DejaVu Serif"],
                "mathtext.fontset": "cm",
                "font.size": 9,
                "pdf.fonttype": 42,
            }
        )

    mpl.rcParams.update(
        {
            "axes.titlesize": 10,
            "axes.labelsize": 9,
            "legend.fontsize": 8,
            "xtick.labelsize": 8,
            "ytick.labelsize": 8,
            "axes.grid": True,
            "grid.alpha": 0.28,
            "grid.linewidth": 0.45,
            "lines.linewidth": 1.15,
        }
    )

    import matplotlib.pyplot as plt

    return plt


def decode_h5string(value) -> str:
    """Decode MATLAB v7.3 character arrays stored as uint16 HDF5 datasets."""
    arr = np.asarray(value).ravel()
    if arr.dtype.kind == "S":
        return b"".join(arr.tolist()).decode("utf-8", errors="ignore").replace("\x00", "")
    if arr.dtype.kind == "U":
        return "".join(arr.tolist()).replace("\x00", "")
    return "".join(chr(int(c)) for c in arr if int(c) != 0)


def h5_scalar(group, name: str) -> float:
    """Read a scalar numeric value from an HDF5 group."""
    return float(np.asarray(group[name]).ravel()[0])


def find_file(stem_dir: Path, prefix: str) -> Path:
    """Return the first file in stem_dir whose name starts with prefix."""
    matches = sorted(stem_dir.glob(prefix + "*"))
    if not matches:
        raise FileNotFoundError(f"No file matching {prefix}* in {stem_dir}")
    return matches[0]


def load_record(path: Path) -> dict:
    """Read the parts of a Stage-F result record used by the plot."""
    with h5py.File(path, "r") as f:
        rec = f["rec"]
        out = {
            "path": path,
            "X_plant": np.asarray(rec["run/X_plant"]).T,
            "U": np.asarray(rec["run/U"]).T,
            "t_tot": np.asarray(rec["run/timing/tot"]).ravel() * 1e3,
            "dt": h5_scalar(rec, "run/dt"),
            "n_actual": int(h5_scalar(rec, "run/n_actual")),
            "t_p99": h5_scalar(rec, "summary/t_p99_ms"),
            "t_mean": h5_scalar(rec, "summary/t_mean_ms"),
            "t_max": h5_scalar(rec, "summary/t_max_ms"),
            "J": h5_scalar(rec, "summary/J_integrated"),
            "classification": decode_h5string(rec["classification"]),
            "order": decode_h5string(rec["cfg/order"]),
            "N": int(h5_scalar(rec, "cfg/N")),
        }

    if out["X_plant"].shape[0] != 12:
        raise ValueError(f"Expected 12 plant states in {path}, got {out['X_plant'].shape}")
    return out


def load_stage_f_records(root: Path) -> dict:
    """Load the four baseline-IC records needed for the split figures."""
    records = {}
    for key, prefix in RUN_PREFIX.items():
        order, _candidate = key
        path = find_file(root / "F" / ORDER_DIR[order], prefix)
        records[key] = load_record(path)
    return records


def time_vector(sample_count: int, dt: float) -> np.ndarray:
    return np.arange(sample_count) * dt


def visible_values(t: np.ndarray, y: np.ndarray, xlim: tuple[float, float]) -> np.ndarray:
    mask = (t >= xlim[0]) & (t <= xlim[1])
    return np.asarray(y)[mask]


def padded_ylim(values: list[np.ndarray], pad: float = 0.07) -> tuple[float, float]:
    merged = np.concatenate([np.asarray(v).ravel() for v in values if np.asarray(v).size])
    finite = merged[np.isfinite(merged)]
    if finite.size == 0:
        return (-1.0, 1.0)

    lo = float(np.min(finite))
    hi = float(np.max(finite))
    span = hi - lo
    if span <= 0.0:
        span = max(abs(hi) * 0.1, 1.0)
    return lo - pad * span, hi + pad * span


def solver_ylim(values: list[np.ndarray]) -> tuple[float, float]:
    merged = np.concatenate([np.asarray(v).ravel() for v in values if np.asarray(v).size])
    finite = merged[np.isfinite(merged) & (merged > 0.0)]
    if finite.size == 0:
        return (0.5, 20.0)

    lo = min(float(np.min(finite)), 5.0) * 0.75
    hi = max(float(np.max(finite)), 10.0) * 1.25
    return max(lo, 1e-3), hi


def latex_candidate(candidate: str, descriptor: str, use_latex: bool) -> str:
    if use_latex:
        return rf"\textsf{{{candidate}}} {descriptor}"
    return f"{candidate} {descriptor}"


def annotation_text(record: dict, use_latex: bool) -> str:
    if use_latex:
        return rf"$t_{{p99}}={record['t_p99']:.2f}\,\mathrm{{ms}}$" + "\n" + rf"$J={record['J']:.2f}$"
    return f"t_p99={record['t_p99']:.2f} ms\nJ={record['J']:.2f}"


def make_closed_loop_page(plt, records: dict, candidate: str, out_dir: Path, use_latex: bool) -> Path:
    """Write one 3 x 2 closed-loop page for F1 or F4."""
    cfg = PAGE_CONFIG[candidate]
    orders = ["full12", "reduced10"]
    page_records = {order: records[(order, candidate)] for order in orders}

    ref = page_records["full12"]
    ref_duration = (ref["X_plant"].shape[1] - 1) * ref["dt"]
    xlim = (0.0, ref_duration)

    fig, axes = plt.subplots(
        3,
        2,
        figsize=(7.15, 9.2),
        sharex=True,
        gridspec_kw={"height_ratios": [1.05, 1.05, 1.0], "hspace": 0.18, "wspace": 0.16},
    )

    z_values = []
    angle_values = []
    timing_values = []

    for col, order in enumerate(orders):
        record = page_records[order]
        x = record["X_plant"]
        dt = record["dt"]

        t_state = time_vector(x.shape[1], dt)
        t_solve = time_vector(record["t_tot"].shape[0], dt)

        z_mm = x[2, :] * 1e3
        roll_deg = np.rad2deg(x[3, :])
        pitch_deg = np.rad2deg(x[4, :])
        z_ref_mm = float(np.mean(z_mm[-20:]))

        title = f"{latex_candidate(candidate, cfg['descriptor'], use_latex)}, {ORDER_LABEL[order]}"
        axes[0, col].set_title(title)

        axes[0, col].plot(t_state, z_mm, color=COLORS["z"])
        axes[0, col].axhline(z_ref_mm, color=COLORS["ref"], linestyle=":", linewidth=0.8)

        axes[1, col].plot(t_state, roll_deg, color=COLORS["roll"], label="roll")
        axes[1, col].plot(t_state, pitch_deg, color=COLORS["pitch"], label="pitch")

        axes[2, col].plot(t_solve, record["t_tot"], color=COLORS["time"], linewidth=0.85)
        axes[2, col].axhline(10.0, color="black", linestyle="--", linewidth=0.75)
        axes[2, col].axhline(5.0, color="black", linestyle=":", linewidth=0.75)
        axes[2, col].set_yscale("log")
        axes[2, col].text(
            0.97,
            0.5,
            annotation_text(record, use_latex),
            transform=axes[2, col].transAxes,
            ha="right",
            va="center",
            fontsize=8,
            bbox={
                "boxstyle": "square,pad=0.25",
                "facecolor": "white",
                "edgecolor": "0.65",
                "linewidth": 0.5,
            },
        )

        z_values.append(np.r_[visible_values(t_state, z_mm, xlim), z_ref_mm])
        angle_values.append(visible_values(t_state, roll_deg, xlim))
        angle_values.append(visible_values(t_state, pitch_deg, xlim))
        timing_values.append(visible_values(t_solve, record["t_tot"], xlim))

        for row in range(3):
            axes[row, col].set_xlim(xlim)

    axes[0, 0].set_ylabel(r"$z$ [mm]")
    axes[1, 0].set_ylabel("angle [deg]")
    axes[2, 0].set_ylabel("solver time [ms]")
    axes[2, 0].set_xlabel("time [s]")
    axes[2, 1].set_xlabel("time [s]")
    axes[1, 1].legend(loc="upper right", frameon=False)

    z_lim = padded_ylim(z_values)
    angle_lim = padded_ylim(angle_values)
    time_lim = solver_ylim(timing_values)
    for col in range(2):
        axes[0, col].set_ylim(z_lim)
        axes[1, col].set_ylim(angle_lim)
        axes[2, col].set_ylim(time_lim)

    fig.suptitle(cfg["suptitle"] + " (baseline IC)", fontsize=11)
    fig.subplots_adjust(left=0.09, right=0.985, bottom=0.06, top=0.94, hspace=0.24, wspace=0.18)

    out_path = out_dir / cfg["outfile"]
    fig.savefig(out_path, bbox_inches="tight", pad_inches=0.04)
    plt.close(fig)
    print(f"wrote {out_path} (x-axis from full12 record: 0 to {ref_duration:.3f} s)")
    return out_path


def find_stage_f_panel_file(root: Path, panel: str, order: str) -> Path:
    """Find the baseline-IC Stage-F record for a panel candidate and model order."""
    o3 = "ful" if order == "full12" else "red"
    stem_dir = root / "F" / ORDER_DIR[order]
    prefix = f"F_{panel}_{o3}_base"
    return find_file(stem_dir, prefix)


def load_panel_records(root: Path) -> list[tuple[str, str, dict]]:
    """Load all baseline-IC Stage-F panel records used by the scatter plot."""
    records = []
    for panel in PANELS:
        for order in ("full12", "reduced10"):
            record = load_record(find_stage_f_panel_file(root, panel, order))
            records.append((panel, order, record))
    return records


def make_panel_scatter(plt, root: Path, out_dir: Path) -> Path:
    """Write the Stage-F speed-cost scatter plot."""
    from matplotlib.lines import Line2D

    records = load_panel_records(root)
    fig, ax = plt.subplots(figsize=(7.3, 4.7))
    cmap = plt.get_cmap("tab10")

    order_marker = {"full12": "o", "reduced10": "s"}
    order_label = {"full12": "12-state", "reduced10": "10-state"}
    panel_to_color = {panel: cmap(i) for i, panel in enumerate(PANELS)}

    for panel, order, record in records:
        is_fail = record["J"] >= 1e6
        j_plot = 1e3 if is_fail else record["J"]
        facecolor = "none" if is_fail else panel_to_color[panel]
        ax.scatter(
            record["t_p99"],
            j_plot,
            marker=order_marker[order],
            s=82 if order == "reduced10" else 92,
            edgecolor=panel_to_color[panel],
            facecolor=facecolor,
            linewidth=1.2,
        )

    panel_handles = [
        Line2D(
            [0],
            [0],
            marker="o",
            linestyle="None",
            markerfacecolor=panel_to_color[panel],
            markeredgecolor=panel_to_color[panel],
            label=PANEL_LABEL[panel],
        )
        for panel in PANELS
    ]
    order_handles = [
        Line2D(
            [0],
            [0],
            marker=order_marker[order],
            linestyle="None",
            markerfacecolor="0.65",
            markeredgecolor="black",
            label=order_label[order],
        )
        for order in ("full12", "reduced10")
    ]

    leg1 = ax.legend(
        handles=panel_handles,
        loc="upper right",
        title="Panel",
        frameon=True,
        fontsize=7.5,
        title_fontsize=8,
    )
    ax.add_artist(leg1)
    ax.legend(
        handles=order_handles,
        loc="lower right",
        title="Model order",
        frameon=True,
        fontsize=7.5,
        title_fontsize=8,
    )

    ax.axvline(10.0, color="black", linestyle="--", linewidth=0.75)
    ax.axvline(5.0, color="black", linestyle=":", linewidth=0.75)
    ax.text(10.0, 4e2, "10 ms", fontsize=7.5, va="top", ha="left")
    ax.text(5.0, 4e2, "5 ms", fontsize=7.5, va="top", ha="right")

    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlabel(r"$t_{p99}$ [ms]")
    ax.set_ylabel(r"integrated closed-loop cost $J$")
    ax.set_title(r"Stage-F panel: speed-cost comparison (baseline IC)")
    ax.set_xlim(1.0, 300.0)
    ax.set_ylim(0.2, 2e3)

    fig.subplots_adjust(left=0.105, right=0.975, bottom=0.13, top=0.91)
    out_path = out_dir / "exp1_panel_scatter.pdf"
    fig.savefig(out_path, bbox_inches="tight", pad_inches=0.04)
    plt.close(fig)
    print(f"wrote {out_path}")
    return out_path


def repeatability_label(order: str, candidate: str, use_latex: bool) -> str:
    if use_latex:
        return rf"\shortstack{{{order}\\{candidate}}}"
    return f"{order}\n{candidate}"


def make_repeatability(plt, out_dir: Path, use_latex: bool) -> Path:
    """Write the Stage-G repeatability bar chart from the Stage-H summary numbers."""
    rows = [
        ("12-state", "F1", 128.99, 3.43, 138.32),
        ("12-state", "F4", 2.38, 0.50, 3.77),
        ("12-state", "F5", 4.34, 0.22, 4.87),
        ("10-state", "F1", 14.51, 0.16, 14.80),
        ("10-state", "F4", 1.81, 0.06, 1.95),
        ("10-state", "F5", 3.58, 0.05, 3.64),
    ]
    labels = [repeatability_label(order, candidate, use_latex) for order, candidate, *_ in rows]
    means = [row[2] for row in rows]
    stds = [row[3] for row in rows]
    maxes = [row[4] for row in rows]

    fig, ax = plt.subplots(figsize=(7.8, 4.45))
    x = np.arange(len(rows))
    colors = [
        "#7E7E7E" if candidate == "F1" else ("#2C5F9E" if candidate == "F4" else "#D98E2B")
        for _order, candidate, *_ in rows
    ]
    ax.bar(
        x,
        means,
        yerr=stds,
        capsize=4,
        color=colors,
        edgecolor="black",
        linewidth=0.55,
    )
    ax.plot(x, maxes, "kv", markersize=6, label=r"$\max\,t_{p99}$ over 10 repetitions")

    ax.axhline(10.0, color="black", linestyle="--", linewidth=0.8, label="10 ms deadline")
    ax.axhline(5.0, color="black", linestyle=":", linewidth=0.8, label="5 ms deadline")

    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_yscale("log")
    ax.set_ylabel(r"$t_{p99}$ [ms] (log scale)")
    ax.set_title(r"Stage-G timing repeatability")
    ax.legend(loc="upper right", fontsize=7.5, frameon=True)
    ax.set_ylim(0.5, 300.0)

    for xi, mean, std, max_val in zip(x, means, stds, maxes):
        label_y = max(mean + std, max_val) * 1.12
        ax.text(xi, label_y, f"{mean:.2f}", ha="center", va="bottom", fontsize=7.5)

    fig.subplots_adjust(left=0.105, right=0.985, bottom=0.16, top=0.91)
    out_path = out_dir / "exp1_repeatability.pdf"
    fig.savefig(out_path, bbox_inches="tight", pad_inches=0.04)
    plt.close(fig)
    print(f"wrote {out_path}")
    return out_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=ROOT, help="Experiment thesis results root.")
    parser.add_argument("--out", type=Path, default=OUT, help="Output directory for generated PDFs.")
    parser.add_argument(
        "--no-latex",
        action="store_true",
        help="Use Matplotlib's normal PDF backend instead of PGF/pdflatex.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    use_latex = not args.no_latex
    plt = configure_matplotlib(use_latex)
    records = load_stage_f_records(args.root)

    for key in sorted(records):
        record = records[key]
        print(
            f"loaded {key[0]} {key[1]}: {record['path'].name}, "
            f"class={record['classification']}, "
            f"t_p99={record['t_p99']:.2f} ms, J={record['J']:.2f}"
        )

    make_closed_loop_page(plt, records, "F1", args.out, use_latex)
    make_closed_loop_page(plt, records, "F4", args.out, use_latex)
    make_panel_scatter(plt, args.root, args.out)
    make_repeatability(plt, args.out, use_latex)


if __name__ == "__main__":
    main()
