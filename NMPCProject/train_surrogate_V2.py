"""
train_surrogate_v3.py — Linearity decomposition approach.

Trains TWO networks (both 3x64, both with 6 pose inputs):
  NN_perm: pose → 6  (permanent magnet acceleration at u=0)
  NN_sol:  pose → 24 (per-solenoid acceleration per unit amp, 4×6)

CasADi integration:
  accel = NN_perm(pose) + reshape(NN_sol(pose), 6, 4) * u

KEY ADVANTAGES:
  - Input dim 6 instead of 10 → much easier for small MLP
  - Linearity in u is exact by construction
  - df/du = reshape(NN_sol(pose), 6, 4) → no chain rule through NN for u-Jacobian
  - NN_perm maps to small values near eq (the hard part is isolated)
  - NN_sol maps the solenoid sensitivity field (smooth, no clipping needed)

INPUT:  nn_training_data_v3.mat (from generate_nn_data_v3.m)
OUTPUT: results/nn_weights_v3.mat, results/best_model_v3.pt, reports
"""

import os, copy
import numpy as np
import scipy.io
import torch
import torch.nn as nn
from torch.utils.data import TensorDataset, DataLoader
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ──────────────────────────────────────────────────────────────────────
# Config
# ──────────────────────────────────────────────────────────────────────
DATA_FILE = "nn_training_data_v3.mat"
OUT_DIR = "results"
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
DTYPE = torch.float64

WIDTH = 64
N_HIDDEN = 3
EPOCHS = 800
BATCH_SIZE = 4096
LR = 1e-3
PATIENCE = 50
MIN_LR = 1e-6

# Equilibrium penalties
EQ_PENALTY_PERM = 10000.0   # NN_perm must output ~0 at equilibrium
EQ_PENALTY_SOL  = 1000.0   # NN_sol must match analytical sensitivity at eq

os.makedirs(OUT_DIR, exist_ok=True)

# ──────────────────────────────────────────────────────────────────────
# Load data
# ──────────────────────────────────────────────────────────────────────
print("Loading", DATA_FILE)
raw = scipy.io.loadmat(DATA_FILE)

X_pose = torch.tensor(raw["nn_pose"].T, dtype=DTYPE)   # (N, 6)
Y_perm = torch.tensor(raw["nn_perm"].T, dtype=DTYPE)   # (N, 6)
Y_sol  = torch.tensor(raw["nn_sol"].T,  dtype=DTYPE)   # (N, 24)

xEq = raw["xEq"].flatten()
eq_pose_6 = xEq[:6]
eq_perm_target = raw["eq_perm_target"].flatten()  # should be ~0
eq_sol_target  = raw["eq_sol_target"].flatten()   # nonzero solenoid sensitivity

N = X_pose.shape[0]
print(f"  {N} samples, pose dim={X_pose.shape[1]}")

# Remove NaN/Inf
valid = (torch.isfinite(X_pose).all(1) &
         torch.isfinite(Y_perm).all(1) &
         torch.isfinite(Y_sol).all(1))
if not valid.all():
    print(f"  Removing {(~valid).sum()} bad samples")
    X_pose, Y_perm, Y_sol = X_pose[valid], Y_perm[valid], Y_sol[valid]
    N = X_pose.shape[0]

print(f"\n  NN_perm output ranges:")
for i, n in enumerate(["ax","ay","az","αx","αy","αz"]):
    print(f"    {n:4s}: [{Y_perm[:,i].min():+.3e}, {Y_perm[:,i].max():+.3e}], std={Y_perm[:,i].std():.3e}")

print(f"\n  NN_sol output ranges (showing first solenoid angular channels):")
for s in range(4):
    for k, n in enumerate(["αx","αy","αz"]):
        idx = s*6 + 3 + k
        print(f"    sol{s+1}_{n}: [{Y_sol[:,idx].min():+.3e}, {Y_sol[:,idx].max():+.3e}], std={Y_sol[:,idx].std():.3e}")

# ──────────────────────────────────────────────────────────────────────
# Network definition
# ──────────────────────────────────────────────────────────────────────

class MLP(nn.Module):
    def __init__(self, n_in, n_out, width=64, n_hidden=3):
        super().__init__()
        layers = [nn.Linear(n_in, width), nn.Tanh()]
        for _ in range(n_hidden - 1):
            layers += [nn.Linear(width, width), nn.Tanh()]
        layers.append(nn.Linear(width, n_out))
        self.net = nn.Sequential(*layers)

    def forward(self, x):
        return self.net(x)

    def count_params(self):
        return sum(p.numel() for p in self.parameters())

# ──────────────────────────────────────────────────────────────────────
# Robust normalization
# ──────────────────────────────────────────────────────────────────────

def robust_norms(X, Y, pct=1.0):
    X_mean, X_std = X.mean(0), X.std(0).clamp(min=1e-8)
    Y_np = Y.numpy()
    lo = np.percentile(Y_np, pct, axis=0)
    hi = np.percentile(Y_np, 100-pct, axis=0)
    Yc = np.clip(Y_np, lo, hi)
    Y_mean = torch.tensor(Yc.mean(0), dtype=DTYPE)
    Y_std  = torch.tensor(Yc.std(0), dtype=DTYPE).clamp(min=1e-8)
    return X_mean, X_std, Y_mean, Y_std

# ──────────────────────────────────────────────────────────────────────
# Training function
# ──────────────────────────────────────────────────────────────────────

def train_net(model, X_tr, Y_tr, X_va, Y_va,
              eq_input, eq_target, eq_weight,
              epochs=EPOCHS, label=""):

    model = model.to(DEVICE)
    X_mean, X_std, Y_mean, Y_std = robust_norms(X_tr, Y_tr)

    Xn_tr = ((X_tr - X_mean) / X_std).to(DEVICE)
    Yn_tr = ((Y_tr - Y_mean) / Y_std).to(DEVICE)
    Xn_va = ((X_va - X_mean) / X_std).to(DEVICE)
    Yn_va = ((Y_va - Y_mean) / Y_std).to(DEVICE)

    eq_in_n = ((torch.tensor(eq_input, dtype=DTYPE) - X_mean) / X_std).to(DEVICE).unsqueeze(0)
    eq_tgt_n = ((torch.tensor(eq_target, dtype=DTYPE) - Y_mean) / Y_std).to(DEVICE).unsqueeze(0)

    loader = DataLoader(TensorDataset(Xn_tr, Yn_tr), batch_size=BATCH_SIZE, shuffle=True)
    optimizer = torch.optim.Adam(model.parameters(), lr=LR)
    scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
        optimizer, patience=PATIENCE, factor=0.5, min_lr=MIN_LR)

    best_val, best_state = float("inf"), None
    t_hist, v_hist = [], []

    for epoch in range(epochs):
        model.train()
        eloss, nb = 0.0, 0
        for xb, yb in loader:
            pred = model(xb)
            loss = nn.functional.mse_loss(pred, yb)
            eq_pred = model(eq_in_n)
            loss = loss + eq_weight * nn.functional.mse_loss(eq_pred, eq_tgt_n)
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            eloss += loss.item(); nb += 1
        t_hist.append(eloss / nb)

        model.eval()
        with torch.no_grad():
            vl = nn.functional.mse_loss(model(Xn_va), Yn_va).item()
        v_hist.append(vl)
        scheduler.step(vl)

        if vl < best_val:
            best_val = vl
            best_state = copy.deepcopy(model.state_dict())

        if (epoch+1) % 100 == 0:
            with torch.no_grad():
                eq_p = (model(eq_in_n).cpu().squeeze() * Y_std + Y_mean).numpy()
                eq_t = eq_target
            err = np.abs(eq_p - eq_t)
            lr_now = optimizer.param_groups[0]["lr"]
            print(f"    [{label}] Epoch {epoch+1:4d}/{epochs}: "
                  f"train={t_hist[-1]:.6f}, val={vl:.6f}, lr={lr_now:.1e}, "
                  f"eq_max_err={err.max():.3e}")

    model.load_state_dict(best_state)
    return model, (X_mean, X_std, Y_mean, Y_std), t_hist, v_hist

# ──────────────────────────────────────────────────────────────────────
# Train/test split
# ──────────────────────────────────────────────────────────────────────
rng = np.random.default_rng(123)
perm = rng.permutation(N)
n_test = max(1000, N // 20)
idx_tr, idx_te = perm[n_test:], perm[:n_test]

X_tr, X_te = X_pose[idx_tr], X_pose[idx_te]
Yp_tr, Yp_te = Y_perm[idx_tr], Y_perm[idx_te]
Ys_tr, Ys_te = Y_sol[idx_tr], Y_sol[idx_te]

# ──────────────────────────────────────────────────────────────────────
# Train NN_perm
# ──────────────────────────────────────────────────────────────────────
print("\n" + "="*70)
print(f"TRAINING NN_perm (6→6, {WIDTH}×{N_HIDDEN})")
print("="*70)

model_perm = MLP(6, 6, WIDTH, N_HIDDEN).to(dtype=DTYPE)
print(f"  {model_perm.count_params()} params, eq_penalty={EQ_PENALTY_PERM}")

model_perm, norms_perm, th_p, vh_p = train_net(
    model_perm, X_tr, Yp_tr, X_te, Yp_te,
    eq_input=eq_pose_6, eq_target=eq_perm_target,
    eq_weight=EQ_PENALTY_PERM, label="PERM"
)

# ──────────────────────────────────────────────────────────────────────
# Train NN_sol
# ──────────────────────────────────────────────────────────────────────
print("\n" + "="*70)
print(f"TRAINING NN_sol (6→24, {WIDTH}×{N_HIDDEN})")
print("="*70)

model_sol = MLP(6, 24, WIDTH, N_HIDDEN).to(dtype=DTYPE)
print(f"  {model_sol.count_params()} params, eq_penalty={EQ_PENALTY_SOL}")

model_sol, norms_sol, th_s, vh_s = train_net(
    model_sol, X_tr, Ys_tr, X_te, Ys_te,
    eq_input=eq_pose_6, eq_target=eq_sol_target,
    eq_weight=EQ_PENALTY_SOL, label=" SOL"
)

# ──────────────────────────────────────────────────────────────────────
# Evaluation
# ──────────────────────────────────────────────────────────────────────
print("\n" + "="*70)
print("EVALUATION")
print("="*70)

def eval_net(model, X, Y, norms, label):
    Xm, Xs, Ym, Ys = norms
    model.eval()
    with torch.no_grad():
        pred = (model(((X - Xm)/Xs).to(DEVICE)).cpu() * Ys + Ym)
    err = (pred - Y).abs()
    print(f"  {label}:")
    print(f"    MAE  = {err.mean():.4e}")
    print(f"    Max  = {err.max():.4e}")
    print(f"    RMSE = {err.pow(2).mean().sqrt():.4e}")
    return err

err_perm = eval_net(model_perm, X_te, Yp_te, norms_perm, "NN_perm (test set)")
err_sol  = eval_net(model_sol,  X_te, Ys_te, norms_sol,  "NN_sol  (test set)")

# Equilibrium check
Xm_p, Xs_p, Ym_p, Ys_p = norms_perm
Xm_s, Xs_s, Ym_s, Ys_s = norms_sol

eq_in_t = torch.tensor(eq_pose_6, dtype=DTYPE).unsqueeze(0)
model_perm.eval(); model_sol.eval()
with torch.no_grad():
    eq_perm_pred = (model_perm(((eq_in_t - Xm_p)/Xs_p).to(DEVICE)).cpu() * Ys_p + Ym_p).squeeze().numpy()
    eq_sol_pred  = (model_sol(((eq_in_t - Xm_s)/Xs_s).to(DEVICE)).cpu() * Ys_s + Ym_s).squeeze().numpy()

report = []
report.append(f"=== LINEARITY DECOMPOSITION RESULTS ===")
report.append(f"NN_perm: 6→6,  {model_perm.count_params()} params")
report.append(f"NN_sol:  6→24, {model_sol.count_params()} params")
report.append(f"Total:   {model_perm.count_params() + model_sol.count_params()} params")
report.append("")

report.append("--- NN_perm at equilibrium (target: all zeros) ---")
report.append(f"  a_lin = [{eq_perm_pred[0]:.6e}, {eq_perm_pred[1]:.6e}, {eq_perm_pred[2]:.6e}]")
report.append(f"  a_ang = [{eq_perm_pred[3]:.6e}, {eq_perm_pred[4]:.6e}, {eq_perm_pred[5]:.6e}]")
report.append(f"  max |a_lin| = {np.max(np.abs(eq_perm_pred[:3])):.4e}  (target: < 0.01)")
report.append(f"  max |a_ang| = {np.max(np.abs(eq_perm_pred[3:])):.4e}  (target: < 1.0)")

report.append("")
report.append("--- NN_sol at equilibrium (solenoid sensitivity) ---")
eq_sol_err = np.abs(eq_sol_pred - eq_sol_target)
report.append(f"  max error = {eq_sol_err.max():.4e}")
report.append(f"  mean error = {eq_sol_err.mean():.4e}")
for s in range(4):
    idx = slice(s*6, (s+1)*6)
    report.append(f"  Sol{s+1} err: lin={eq_sol_err[idx][:3].max():.3e}, ang={eq_sol_err[idx][3:].max():.3e}")

# Combined check: accel at (xEq, u=0) = NN_perm(xEq) + NN_sol(xEq)*0 = NN_perm(xEq)
report.append("")
report.append("--- Combined accel at (xEq, uEq=0) ---")
report.append(f"  = NN_perm(xEq) = [{eq_perm_pred[0]:.4e}, ..., {eq_perm_pred[5]:.4e}]")
report.append(f"  max |accel| = {np.max(np.abs(eq_perm_pred)):.4e}")

# GO/NO-GO
perm_lin_ok = np.max(np.abs(eq_perm_pred[:3])) < 0.1
perm_ang_ok = np.max(np.abs(eq_perm_pred[3:])) < 10.0
sol_ok = eq_sol_err.max() < 5000  # solenoid sensitivity error manageable

report.append("")
if perm_lin_ok and perm_ang_ok and sol_ok:
    report.append(">>> GO: Run acadosNMPC_NNvsAnalyticComparison_v3.m <<<")
elif perm_lin_ok and np.max(np.abs(eq_perm_pred[3:])) < 50.0:
    report.append(">>> MARGINAL: Worth trying the comparison <<<")
else:
    report.append(">>> NO-GO <<<")
    if not perm_lin_ok: report.append(f"  perm linear: {np.max(np.abs(eq_perm_pred[:3])):.4e}")
    if not perm_ang_ok: report.append(f"  perm angular: {np.max(np.abs(eq_perm_pred[3:])):.4e}")

report_text = "\n".join(report)
print("\n" + report_text)
with open(os.path.join(OUT_DIR, "equilibrium_report.txt"), "w") as f:
    f.write(report_text + "\n")

# ──────────────────────────────────────────────────────────────────────
# Plot
# ──────────────────────────────────────────────────────────────────────
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4))
ax1.semilogy(th_p, label="Train", alpha=0.7)
ax1.semilogy(vh_p, label="Hold-out", alpha=0.7)
ax1.set_title("NN_perm (6→6)"); ax1.set_xlabel("Epoch"); ax1.legend(); ax1.grid(True, ls=":")
ax2.semilogy(th_s, label="Train", alpha=0.7)
ax2.semilogy(vh_s, label="Hold-out", alpha=0.7)
ax2.set_title("NN_sol (6→24)"); ax2.set_xlabel("Epoch"); ax2.legend(); ax2.grid(True, ls=":")
fig.tight_layout()
fig.savefig(os.path.join(OUT_DIR, "training_curves_v3.png"), dpi=150)
print(f"\n  -> saved training_curves_v3.png")

# ──────────────────────────────────────────────────────────────────────
# Export weights
# ──────────────────────────────────────────────────────────────────────
print("\n" + "="*70)
print("EXPORTING WEIGHTS")
print("="*70)

def extract_weights(model, prefix):
    state = model.cpu().state_dict()
    wkeys = sorted([k for k in state if ".weight" in k], key=lambda k: list(state.keys()).index(k))
    bkeys = sorted([k for k in state if ".bias" in k],   key=lambda k: list(state.keys()).index(k))
    out = {}
    for i, (wk, bk) in enumerate(zip(wkeys, bkeys)):
        if i < len(wkeys) - 1:
            wn, bn = f"{prefix}_W{i+1}", f"{prefix}_b{i+1}"
        else:
            wn, bn = f"{prefix}_Wout", f"{prefix}_bout"
        out[wn] = state[wk].numpy()
        out[bn] = state[bk].numpy()
        print(f"    {wn}: {out[wn].shape},  {bn}: {out[bn].shape}")
    return out

export = {}

# Normalization
for tag, norms in [("perm", norms_perm), ("sol", norms_sol)]:
    Xm, Xs, Ym, Ys = norms
    export[f"{tag}_input_mean"]  = Xm.numpy()
    export[f"{tag}_input_std"]   = Xs.numpy()
    export[f"{tag}_output_mean"] = Ym.numpy()
    export[f"{tag}_output_std"]  = Ys.numpy()

# Weights
print("  NN_perm weights:")
export.update(extract_weights(model_perm, "perm"))
print("  NN_sol weights:")
export.update(extract_weights(model_sol, "sol"))

export["architecture"] = f"MLP_{N_HIDDEN}x{WIDTH}"
export["output_type"] = "linearity_decomposition"
export["n_hidden_layers"] = np.array([N_HIDDEN])
export["eq_sol_target"] = eq_sol_target

mat_path = os.path.join(OUT_DIR, "nn_weights_v3.mat")
scipy.io.savemat(mat_path, export)
print(f"\n  -> saved {mat_path}")

pt_path = os.path.join(OUT_DIR, "best_model_v3.pt")
torch.save({
    "perm_state": model_perm.state_dict(),
    "sol_state": model_sol.state_dict(),
    "norms_perm": norms_perm,
    "norms_sol": norms_sol,
}, pt_path)
print(f"  -> saved {pt_path}")

print("\n" + "="*70)
print("DONE")
print("="*70)
print(f"""
  NN_perm: {model_perm.count_params()} params (6→6)
  NN_sol:  {model_sol.count_params()} params (6→24)
  Total:   {model_perm.count_params() + model_sol.count_params()} params

  CasADi integration:
    accel_perm = NN_perm(pose);                       % 6x1
    sol_flat   = NN_sol(pose);                        % 24x1
    sol_matrix = [sol_flat(1:6), sol_flat(7:12), sol_flat(13:18), sol_flat(19:24)];  % 6x4
    accel_nn   = accel_perm + sol_matrix * u;         % 6x1

  Key: df/du = sol_matrix — exact, no NN chain rule!
""")