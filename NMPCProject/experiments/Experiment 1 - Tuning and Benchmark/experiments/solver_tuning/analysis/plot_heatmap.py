"""Stage-2 heatmaps: N x controller_n at fixed Tf, value=t_mean or J.

Usage:
    python plot_heatmap.py [--results-dir PATH]

Reads:  <results-dir>/index.csv
Writes: <results-dir>/figs/heatmap_<model_kind>_<config>_<metric>_Tf<Tf>.pdf
"""
from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def heatmap(
    df: pd.DataFrame,
    title: str,
    metric: str,
    out: Path,
    log_color: bool = False,
) -> None:
    if df.empty:
        return

    pivot = df.pivot_table(index="N", columns="controller_n", values=metric, aggfunc="mean")
    pivot = pivot.sort_index(ascending=True).sort_index(axis=1)

    fig, ax = plt.subplots(figsize=(6.0, 4.0))
    data = pivot.to_numpy()
    if log_color:
        data = np.log10(np.where(data > 0, data, np.nan))
        cbar_label = f"log10({metric})"
    else:
        cbar_label = metric

    im = ax.imshow(
        data,
        aspect="auto",
        origin="lower",
        cmap="viridis",
        interpolation="nearest",
    )
    ax.set_xticks(range(len(pivot.columns)))
    ax.set_xticklabels(pivot.columns)
    ax.set_yticks(range(len(pivot.index)))
    ax.set_yticklabels(pivot.index)
    ax.set_xlabel("controller_n")
    ax.set_ylabel("N (horizon nodes)")
    ax.set_title(title)

    # Annotate each cell
    for r in range(pivot.shape[0]):
        for c in range(pivot.shape[1]):
            val = pivot.iat[r, c]
            if pd.notna(val):
                ax.text(c, r, f"{val:.2g}", ha="center", va="center",
                        color="white" if data[r, c] > np.nanmedian(data) else "black",
                        fontsize=8)

    cbar = fig.colorbar(im, ax=ax)
    cbar.set_label(cbar_label)
    fig.tight_layout()
    fig.savefig(out)
    plt.close(fig)
    print(f"  wrote {out}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--results-dir", type=Path, default=Path("results"))
    args = ap.parse_args()

    csv = args.results_dir / "index.csv"
    if not csv.exists():
        raise SystemExit(f"index.csv not found at {csv}; run dump_index() first")

    df = pd.read_csv(csv)
    figs = args.results_dir / "figs"
    figs.mkdir(parents=True, exist_ok=True)

    sub = df[(df.stage == "2") & (~df.diverged) & (df.all_converged)].copy()
    if sub.empty:
        print("No Stage-2 rows; nothing to plot.")
        return

    # Group by (model_kind, top-config exp_id prefix, Tf)
    # The Stage-2 exp_id is `<top1b_id>_N{N}_T{Tf*1000:03d}_n{controller_n}`,
    # so split on "_N" to recover the top1b id.
    sub["top_id"] = sub.exp_id.str.split("_N", n=1).str[0]

    for (model_kind, top_id, tf), g in sub.groupby(["model_kind", "top_id", "Tf"]):
        if g.empty:
            continue
        tag = f"{model_kind}_{top_id}_Tf{int(round(tf*1000)):03d}"
        heatmap(g, f"{model_kind} | {top_id} | Tf={tf:.3f} s | mean t [ms]",
                "t_mean_ms", figs / f"heatmap_{tag}_t_mean.pdf")
        heatmap(g, f"{model_kind} | {top_id} | Tf={tf:.3f} s | J",
                "J_integrated", figs / f"heatmap_{tag}_J.pdf", log_color=True)


if __name__ == "__main__":
    main()
