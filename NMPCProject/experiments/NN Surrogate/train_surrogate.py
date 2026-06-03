"""
train_surrogate_v2.py — Revised NN training for maglev accelerations.

CHANGES FROM V1:
────────────────────────────────────────────────────────────────────
The V1 network was fine (3x64, 9414 params). The DATA was the problem.
- V1 data had angular accel range +/-2e8 (from extreme domain corners)
- V2 data uses narrower domain + output clipping -> range +/-200,000
- This 1000x reduction in dynamic range lets the same 3x64 network
  achieve much better accuracy where it matters (near equilibrium)

Architecture is kept at 3x64 because:
  - CasADi must build the full symbolic expression graph + Jacobians
  - 3x64 already takes 10-20 min to compile in acados
  - 3x128 would take 1+ hours and produce huge C code
  - The Teensy target needs minimal operations per forward pass

Other changes:
  - Equilibrium penalty: 1000 (was 100)
  - Robust normalization (percentile-based)
  - More epochs: 800 (was 500)
  - GO/NO-GO check in output

PREREQUISITES
    pip install torch numpy scipy scikit-learn matplotlib

INPUT FILE
    nn_training_data.mat   (from generate_nn_data_v2.m)

OUTPUT FILES  (./results/)
    cv_results.txt, training_curves.png, equilibrium_report.txt,
    nn_weights.mat, best_model.pt
"""

import os
import copy
import numpy as np
import scipy.io
import torch
import torch.nn as nn
from torch.utils.data import TensorDataset, DataLoader
from sklearn.model_selection import KFold
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ──────────────────────────────────────────────────────────────────────
# 0.  Configuration
# ──────────────────────────────────────────────────────────────────────
DATA_FILE = "nn_training_data.mat"
OUT_DIR = "results"
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
DTYPE = torch.float64

# Architecture — KEEP 3x64 for CasADi/acados/Teensy compatibility
WIDTH = 64
N_HIDDEN = 3

# Training
K_FOLDS = 3
EPOCHS = 800
BATCH_SIZE = 4096
LR = 1e-3
PATIENCE = 50
MIN_LR = 1e-6

# Equilibrium penalty — 10x higher than v1
EQ_PENALTY_WEIGHT = 5000.0

# Near-eq evaluation radius
EQ_RADIUS_NORMED = 1.0

os.makedirs(OUT_DIR, exist_ok=True)

# ──────────────────────────────────────────────────────────────────────
# 1.  Load data
# ──────────────────────────────────────────────────────────────────────
print("Loading data from", DATA_FILE)
raw = scipy.io.loadmat(DATA_FILE)

X_all = torch.tensor(raw["nn_input"].T, dtype=DTYPE)
Y_all = torch.tensor(raw["nn_output"].T, dtype=DTYPE)

xEq = raw["xEq"].flatten()
uEq = raw["uEq"].flatten()
eq_input_10 = np.concatenate([xEq[:6], uEq])
eq_output_6 = np.zeros(6)

# Remove NaN/Inf
valid = torch.isfinite(Y_all).all(dim=1) & torch.isfinite(X_all).all(dim=1)
if not valid.all():
    n_bad = (~valid).sum().item()
    print(f"  WARNING: Removing {n_bad} NaN/Inf samples")
    X_all, Y_all = X_all[valid], Y_all[valid]

N_total = X_all.shape[0]
print(f"  {N_total} samples.  X: {tuple(X_all.shape)}, Y: {tuple(Y_all.shape)}")
print(f"  Device: {DEVICE}")

print("  Input ranges:")
for i, name in enumerate(
    ["x", "y", "z", "roll", "pitch", "yaw", "u1", "u2", "u3", "u4"]
):
    print(f"    {name:6s}: [{X_all[:,i].min():+.4f}, {X_all[:,i].max():+.4f}]")

print("  Output ranges:")
for i, name in enumerate(["ax", "ay", "az", "alpha_x", "alpha_y", "alpha_z"]):
    lo, hi = Y_all[:,i].min().item(), Y_all[:,i].max().item()
    std = Y_all[:,i].std().item()
    print(f"    {name:8s}: [{lo:+.4e}, {hi:+.4e}], std={std:.4e}")

# ──────────────────────────────────────────────────────────────────────
# 2.  Network — MLP_3x64 (same as v1)
# ──────────────────────────────────────────────────────────────────────

class PlainMLP(nn.Module):
    def __init__(self, width=64, n_hidden=3):
        super().__init__()
        layers = [nn.Linear(10, width), nn.Tanh()]
        for _ in range(n_hidden - 1):
            layers += [nn.Linear(width, width), nn.Tanh()]
        layers.append(nn.Linear(width, 6))
        self.net = nn.Sequential(*layers)

    def forward(self, x):
        return self.net(x)

    def count_params(self):
        return sum(p.numel() for p in self.parameters())


ARCH_NAME = f"MLP_{N_HIDDEN}x{WIDTH}"
make_model = lambda: PlainMLP(width=WIDTH, n_hidden=N_HIDDEN)

tmp = make_model()
print(f"\nArchitecture: {ARCH_NAME}  ({tmp.count_params()} parameters)")
del tmp

# ──────────────────────────────────────────────────────────────────────
# 3.  Robust normalization
# ──────────────────────────────────────────────────────────────────────

def compute_robust_norms(X, Y, percentile=1.0):
    """Compute mean/std with percentile-clipped output statistics.

    Prevents remaining outliers from inflating std, which would
    compress the near-equilibrium region in normalized space.
    """
    X_mean = X.mean(dim=0)
    X_std = X.std(dim=0).clamp(min=1e-8)

    Y_np = Y.numpy()
    lo = np.percentile(Y_np, percentile, axis=0)
    hi = np.percentile(Y_np, 100 - percentile, axis=0)
    Y_clipped = np.clip(Y_np, lo, hi)
    Y_mean = torch.tensor(Y_clipped.mean(axis=0), dtype=DTYPE)
    Y_std = torch.tensor(Y_clipped.std(axis=0), dtype=DTYPE).clamp(min=1e-8)

    return X_mean, X_std, Y_mean, Y_std

# ──────────────────────────────────────────────────────────────────────
# 4.  Training helper
# ──────────────────────────────────────────────────────────────────────

def train_one_fold(model, X_train, Y_train, X_val, Y_val, epochs=EPOCHS):
    model = model.to(DEVICE)

    X_mean, X_std, Y_mean, Y_std = compute_robust_norms(X_train, Y_train)

    Xn_tr = ((X_train - X_mean) / X_std).to(DEVICE)
    Yn_tr = ((Y_train - Y_mean) / Y_std).to(DEVICE)
    Xn_va = ((X_val - X_mean) / X_std).to(DEVICE)
    Yn_va = ((Y_val - Y_mean) / Y_std).to(DEVICE)

    eq_in_normed = (
        (torch.tensor(eq_input_10, dtype=DTYPE) - X_mean) / X_std
    ).to(DEVICE).unsqueeze(0)
    eq_out_normed = (
        (torch.tensor(eq_output_6, dtype=DTYPE) - Y_mean) / Y_std
    ).to(DEVICE).unsqueeze(0)

    loader = DataLoader(
        TensorDataset(Xn_tr, Yn_tr), batch_size=BATCH_SIZE, shuffle=True
    )
    optimizer = torch.optim.Adam(model.parameters(), lr=LR)
    scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
        optimizer, patience=PATIENCE, factor=0.5, min_lr=MIN_LR
    )

    best_val = float("inf")
    best_state = None
    train_hist, val_hist = [], []

    for epoch in range(epochs):
        model.train()
        epoch_loss = 0.0
        n_batches = 0
        for xb, yb in loader:
            pred = model(xb)
            loss = nn.functional.mse_loss(pred, yb)

            eq_pred = model(eq_in_normed)
            eq_loss = nn.functional.mse_loss(eq_pred, eq_out_normed)
            loss = loss + EQ_PENALTY_WEIGHT * eq_loss

            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            epoch_loss += loss.item()
            n_batches += 1
        train_hist.append(epoch_loss / n_batches)

        model.eval()
        with torch.no_grad():
            val_pred = model(Xn_va)
            val_loss = nn.functional.mse_loss(val_pred, Yn_va).item()
        val_hist.append(val_loss)
        scheduler.step(val_loss)

        if val_loss < best_val:
            best_val = val_loss
            best_state = copy.deepcopy(model.state_dict())

        if (epoch + 1) % 100 == 0:
            lr_now = optimizer.param_groups[0]["lr"]
            with torch.no_grad():
                eq_p = model(eq_in_normed)
                eq_p_real = (eq_p.cpu().squeeze() * Y_std + Y_mean).numpy()
            eq_max_lin = np.max(np.abs(eq_p_real[:3]))
            eq_max_ang = np.max(np.abs(eq_p_real[3:]))
            print(
                f"    Epoch {epoch+1:4d}/{epochs}: "
                f"train={train_hist[-1]:.6f}, val={val_loss:.6f}, "
                f"lr={lr_now:.1e}, eq_lin={eq_max_lin:.3e}, eq_ang={eq_max_ang:.3e}"
            )

    model.load_state_dict(best_state)
    norms = (X_mean, X_std, Y_mean, Y_std)
    return model, norms, train_hist, val_hist

# ──────────────────────────────────────────────────────────────────────
# 5.  Evaluation helpers
# ──────────────────────────────────────────────────────────────────────

def evaluate(model, X, Y, norms):
    X_mean, X_std, Y_mean, Y_std = norms
    model.eval()
    with torch.no_grad():
        Xn = ((X - X_mean) / X_std).to(DEVICE)
        pred_n = model(Xn).cpu()
        pred = pred_n * Y_std + Y_mean
    abs_err = (pred - Y).abs()
    return {
        "mae_lin": abs_err[:, :3].mean().item(),
        "mae_ang": abs_err[:, 3:].mean().item(),
        "max_lin": abs_err[:, :3].max().item(),
        "max_ang": abs_err[:, 3:].max().item(),
        "rmse_lin": abs_err[:, :3].pow(2).mean().sqrt().item(),
        "rmse_ang": abs_err[:, 3:].pow(2).mean().sqrt().item(),
    }


def near_equilibrium_mask(X, norms, radius=EQ_RADIUS_NORMED):
    X_mean, X_std, _, _ = norms
    eq_tensor = torch.tensor(eq_input_10, dtype=DTYPE)
    eq_normed = (eq_tensor - X_mean) / X_std
    X_normed = (X - X_mean) / X_std
    return (X_normed - eq_normed).norm(dim=1) < radius

# ──────────────────────────────────────────────────────────────────────
# 6.  Cross-validation (quick: 3 folds, 300 epochs)
# ──────────────────────────────────────────────────────────────────────
CV_EPOCHS = 300
print("\n" + "=" * 70)
print(f"CROSS-VALIDATION  (K={K_FOLDS}, epochs={CV_EPOCHS}, arch={ARCH_NAME})")
print("=" * 70)

kf = KFold(n_splits=K_FOLDS, shuffle=True, random_state=42)
fold_metrics = []

for fold_idx, (train_idx, val_idx) in enumerate(kf.split(X_all)):
    print(f"\n  Fold {fold_idx + 1}/{K_FOLDS}")
    model = make_model().to(dtype=DTYPE)
    X_tr, Y_tr = X_all[train_idx], Y_all[train_idx]
    X_va, Y_va = X_all[val_idx], Y_all[val_idx]

    model, norms, _, _ = train_one_fold(model, X_tr, Y_tr, X_va, Y_va, epochs=CV_EPOCHS)

    full = evaluate(model, X_va, Y_va, norms)
    eq_mask = near_equilibrium_mask(X_va, norms)
    n_eq = eq_mask.sum().item()
    eq = evaluate(model, X_va[eq_mask], Y_va[eq_mask], norms) if n_eq > 0 else {}

    result = {
        "fold": fold_idx,
        "mae_lin": full["mae_lin"], "mae_ang": full["mae_ang"],
        "max_lin": full["max_lin"], "max_ang": full["max_ang"],
        "eq_mae_lin": eq.get("mae_lin", float("nan")),
        "eq_mae_ang": eq.get("mae_ang", float("nan")),
        "n_eq": n_eq,
    }
    fold_metrics.append(result)
    print(f"    MAE_lin={full['mae_lin']:.3e}, MAE_ang={full['mae_ang']:.3e}, "
          f"eq_MAE_lin={result['eq_mae_lin']:.3e}, eq_MAE_ang={result['eq_mae_ang']:.3e}")

# Summary
print("\n" + "=" * 70)
print("CV SUMMARY")
print("=" * 70)
summary_lines = [
    f"Architecture: {ARCH_NAME}  ({make_model().count_params()} params)",
    f"Domain: narrowed (z=[20,45]mm, roll/pitch=[-0.25,0.25]rad, clipped outputs)",
    f"Eq penalty = {EQ_PENALTY_WEIGHT}",
    "",
    f"{'Fold':<6s} {'MAE_lin':>12s} {'MAE_ang':>12s} {'eq_MAE_lin':>12s} {'eq_MAE_ang':>12s}",
    "-" * 60,
]
for f in fold_metrics:
    summary_lines.append(
        f"{f['fold']+1:<6d} {f['mae_lin']:>12.4e} {f['mae_ang']:>12.4e} "
        f"{f['eq_mae_lin']:>12.4e} {f['eq_mae_ang']:>12.4e}")
summary_lines.append("-" * 60)
ml = np.mean([f["mae_lin"] for f in fold_metrics])
ma = np.mean([f["mae_ang"] for f in fold_metrics])
el = np.nanmean([f["eq_mae_lin"] for f in fold_metrics])
ea = np.nanmean([f["eq_mae_ang"] for f in fold_metrics])
summary_lines.append(f"{'Mean':<6s} {ml:>12.4e} {ma:>12.4e} {el:>12.4e} {ea:>12.4e}")

summary_text = "\n".join(summary_lines)
print(summary_text)
with open(os.path.join(OUT_DIR, "cv_results.txt"), "w") as f:
    f.write(summary_text + "\n")

# ──────────────────────────────────────────────────────────────────────
# 7.  Full retrain
# ──────────────────────────────────────────────────────────────────────
print("\n" + "=" * 70)
print(f"FULL RETRAIN ({N_total} samples, {EPOCHS} epochs)")
print("=" * 70)

rng_np = np.random.default_rng(123)
perm = rng_np.permutation(N_total)
n_holdout = max(1000, N_total // 20)
idx_train, idx_test = perm[n_holdout:], perm[:n_holdout]

final_model = make_model().to(dtype=DTYPE)
print(f"  {ARCH_NAME} ({final_model.count_params()} params), {EPOCHS} epochs")

final_model, final_norms, t_hist, v_hist = train_one_fold(
    final_model, X_all[idx_train], Y_all[idx_train],
    X_all[idx_test], Y_all[idx_test], epochs=EPOCHS
)

# Plot
fig, ax = plt.subplots(1, 1, figsize=(8, 4))
ax.semilogy(t_hist, label="Train", alpha=0.7)
ax.semilogy(v_hist, label="Hold-out", alpha=0.7)
ax.set_xlabel("Epoch"); ax.set_ylabel("Normalised MSE")
ax.set_title(f"Training curves -- {ARCH_NAME} (v2 data)")
ax.legend(); ax.grid(True, which="both", ls=":")
fig.tight_layout()
fig.savefig(os.path.join(OUT_DIR, "training_curves.png"), dpi=150)
print(f"  -> saved training_curves.png")

# ──────────────────────────────────────────────────────────────────────
# 8.  Final evaluation
# ──────────────────────────────────────────────────────────────────────
print("\n" + "=" * 70)
print("FINAL EVALUATION")
print("=" * 70)

X_test, Y_test = X_all[idx_test], Y_all[idx_test]
full_eval = evaluate(final_model, X_test, Y_test, final_norms)
eq_mask_test = near_equilibrium_mask(X_test, final_norms)
n_eq_test = eq_mask_test.sum().item()

report = []
report.append(f"Architecture        : {ARCH_NAME}")
report.append(f"Parameters          : {final_model.count_params()}")
report.append(f"Eq penalty weight   : {EQ_PENALTY_WEIGHT}")
report.append(f"Test samples        : {len(X_test)}")
report.append(f"Near-eq samples     : {n_eq_test}")
report.append("")
report.append("--- Full domain ---")
report.append(f"  MAE  linear accel  : {full_eval['mae_lin']:.4e} m/s2")
report.append(f"  MAE  angular accel : {full_eval['mae_ang']:.4e} rad/s2")
report.append(f"  RMSE linear accel  : {full_eval['rmse_lin']:.4e} m/s2")
report.append(f"  RMSE angular accel : {full_eval['rmse_ang']:.4e} rad/s2")
report.append(f"  Max  linear accel  : {full_eval['max_lin']:.4e} m/s2")
report.append(f"  Max  angular accel : {full_eval['max_ang']:.4e} rad/s2")

if n_eq_test > 0:
    eq_eval = evaluate(final_model, X_test[eq_mask_test], Y_test[eq_mask_test], final_norms)
    report.append("")
    report.append("--- Near equilibrium ---")
    report.append(f"  MAE  linear accel  : {eq_eval['mae_lin']:.4e} m/s2")
    report.append(f"  MAE  angular accel : {eq_eval['mae_ang']:.4e} rad/s2")
    report.append(f"  Max  linear accel  : {eq_eval['max_lin']:.4e} m/s2")
    report.append(f"  Max  angular accel : {eq_eval['max_ang']:.4e} rad/s2")

# Exact equilibrium
X_mean, X_std, Y_mean, Y_std = final_norms
eq_in = torch.tensor(eq_input_10, dtype=DTYPE).unsqueeze(0)
final_model.eval()
with torch.no_grad():
    eq_pred_n = final_model(((eq_in - X_mean) / X_std).to(DEVICE)).cpu()
    eq_pred = (eq_pred_n * Y_std + Y_mean).squeeze()

report.append("")
report.append("--- Exact equilibrium prediction ---")
report.append(f"  a_lin  = [{eq_pred[0]:.6e}, {eq_pred[1]:.6e}, {eq_pred[2]:.6e}] m/s2")
report.append(f"  a_ang  = [{eq_pred[3]:.6e}, {eq_pred[4]:.6e}, {eq_pred[5]:.6e}] rad/s2")
report.append(f"  max |a_lin| = {eq_pred[:3].abs().max().item():.4e} m/s2  (target: < 0.1)")
report.append(f"  max |a_ang| = {eq_pred[3:].abs().max().item():.4e} rad/s2  (target: < 10.0)")
report.append("")
report.append("--- GO / NO-GO for NMPC comparison ---")

eq_lin_ok = eq_pred[:3].abs().max().item() < 0.1
eq_ang_ok = eq_pred[3:].abs().max().item() < 10.0

if eq_lin_ok and eq_ang_ok:
    report.append("  >>> GO: Equilibrium accuracy sufficient for NMPC test <<<")
    report.append("  Run: acadosNMPC_NNvsAnalyticComparison.m")
    report.append("  (delete c_generated_code/maglev_nn* first)")
elif eq_lin_ok and eq_pred[3:].abs().max().item() < 50.0:
    report.append("  >>> MARGINAL: May work but expect some steady-state offset <<<")
    report.append("  Worth trying the comparison script.")
else:
    report.append("  >>> NO-GO: Equilibrium accuracy insufficient <<<")
    if not eq_lin_ok:
        report.append(f"    Linear: {eq_pred[:3].abs().max():.4e} > 0.1 m/s2")
    if not eq_ang_ok:
        report.append(f"    Angular: {eq_pred[3:].abs().max():.4e} > 10.0 rad/s2")
    report.append("  Suggestions:")
    report.append("    1. Increase EQ_PENALTY_WEIGHT (try 5000 or 10000)")
    report.append("    2. Narrow domain further (z=[22,40]mm, roll/pitch=[-0.15,0.15])")
    report.append("    3. Tighter output clipping (CLIP_ANG=50000)")

report_text = "\n".join(report)
print(report_text)
with open(os.path.join(OUT_DIR, "equilibrium_report.txt"), "w") as f:
    f.write(report_text + "\n")

# ──────────────────────────────────────────────────────────────────────
# 9.  Export weights
# ──────────────────────────────────────────────────────────────────────
print("\n" + "=" * 70)
print("EXPORTING WEIGHTS")
print("=" * 70)

state = final_model.cpu().state_dict()
export = {
    "input_mean": X_mean.numpy(),
    "input_std": X_std.numpy(),
    "output_mean": Y_mean.numpy(),
    "output_std": Y_std.numpy(),
    "architecture": ARCH_NAME,
    "output_type": "accelerations",
}

weight_keys = sorted(
    [k for k in state if k.endswith(".weight")],
    key=lambda k: list(state.keys()).index(k),
)
bias_keys = sorted(
    [k for k in state if k.endswith(".bias")],
    key=lambda k: list(state.keys()).index(k),
)

n_layers = len(weight_keys)
assert n_layers == N_HIDDEN + 1, f"Expected {N_HIDDEN+1} layers, got {n_layers}"

for i, (wk, bk) in enumerate(zip(weight_keys, bias_keys)):
    W = state[wk].numpy()
    b = state[bk].numpy()
    label = f"W{i+1}" if i < n_layers - 1 else "Wout"
    blabel = f"b{i+1}" if i < n_layers - 1 else "bout"
    export[label] = W
    export[blabel] = b
    print(f"    {label}: {W.shape},  {blabel}: {b.shape}")

export["n_hidden_layers"] = np.array([n_layers - 1])

mat_path = os.path.join(OUT_DIR, "nn_weights.mat")
scipy.io.savemat(mat_path, export)
print(f"\n  -> saved {mat_path}")

pt_path = os.path.join(OUT_DIR, "best_model.pt")
torch.save({
    "state_dict": final_model.state_dict(),
    "architecture": ARCH_NAME,
    "norms": {"X_mean": X_mean, "X_std": X_std, "Y_mean": Y_mean, "Y_std": Y_std},
}, pt_path)
print(f"  -> saved {pt_path}")

print("\n" + "=" * 70)
print("DONE")
print("=" * 70)
print(f"""
  Architecture: {ARCH_NAME} ({final_model.count_params()} params) -- same as v1
  Eq penalty:   {EQ_PENALTY_WEIGHT} (10x higher than v1)
  Data:         Narrowed domain + clipped outputs (the real fix)

  Check equilibrium_report.txt for GO / NO-GO.
  The comparison script (acadosNMPC_NNvsAnalyticComparison.m) works
  unchanged -- same W1(64x10), W2(64x64), W3(64x64), Wout(6x64).
""")