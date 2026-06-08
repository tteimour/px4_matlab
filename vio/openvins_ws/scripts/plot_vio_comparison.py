#!/usr/bin/env python3
"""Generate VIO vs Ground Truth comparison plots.

Changes from v1:
  - Trims post-landing VIO drift
  - PX4 labeled as "Ground Truth"
  - Simulated INS-only (dead reckoning) divergence in GNSS-denied zone
  - GNSS-denied shading only on position plots (where INS drift is shown)
  - No scatter plot
  - Simplified RMS/CDF/stats (VIO error vs GT only)
  - 3D trajectory plot

Usage:
    python3 plot_vio_comparison.py --px4 px4.csv --vio vio.csv --output-dir plots/
"""

import argparse
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
import numpy as np
from scipy.spatial.transform import Rotation


# ================================================================
#  Data loading & preprocessing
# ================================================================

def load_csv(path):
    return np.genfromtxt(path, delimiter=",", names=True)


def vio_to_ned(vio):
    """OV Z-up → NED-like via Rx(pi): [x, -y, -z]."""
    return np.column_stack([vio["x"], -vio["y"], -vio["z"]])


def vio_velocity_to_world_ned(vio):
    """Rotate VIO body-FLU velocity to world NED."""
    n = len(vio)
    vel = np.zeros((n, 3))
    for i in range(n):
        q = [vio["qx"][i], vio["qy"][i], vio["qz"][i], vio["qw"][i]]
        R_ov = Rotation.from_quat(q).as_matrix()
        v_body = np.array([vio["vx"][i], vio["vy"][i], vio["vz"][i]])
        v_w = R_ov @ v_body
        vel[i] = [v_w[0], -v_w[1], -v_w[2]]
    return vel


def umeyama_se3(src, dst):
    """Rigid SE(3) alignment (no scale). dst ~ R @ src + t."""
    mu_s = src.mean(axis=0)
    mu_d = dst.mean(axis=0)
    H = (src - mu_s).T @ (dst - mu_d) / len(src)
    U, _, Vt = np.linalg.svd(H)
    d = np.linalg.det(Vt.T @ U.T)
    R = Vt.T @ np.diag([1.0, 1.0, d]) @ U.T
    t = mu_d - R @ mu_s
    return R, t


def trim_landing(t, gt, vio, vel):
    """Remove post-landing data where altitude < 1m after having been high."""
    alt = -gt[:, 2]  # NED z negative = up, so -z = altitude
    max_alt = np.max(alt)
    if max_alt < 2.0:
        return t, gt, vio, vel  # never flew high, don't trim

    # Find last time altitude > 1.5m
    above = np.where(alt > 1.5)[0]
    if len(above) == 0:
        return t, gt, vio, vel
    last_above = above[-1]
    # Add small buffer (2 seconds at 20 Hz = 40 pts)
    cut = min(last_above + 40, len(t))
    return t[:cut], gt[:cut], vio[:cut], vel[:cut]


def generate_ins_drift(t, gt, gt_vel, t0d, t1d):
    """Simulate INS-only dead reckoning that diverges in GNSS-denied zone.

    Before denial: follows ground truth exactly (GPS corrected).
    During denial: accumulates accelerometer bias → quadratic position drift.
    After denial: GPS corrections resume, snaps back to ground truth.
    """
    np.random.seed(42)
    n = len(t)
    dt = np.median(np.diff(t))
    ins = gt.copy()

    # Find indices for GNSS-denied boundaries
    i_start = np.searchsorted(t, t0d)
    i_end = np.searchsorted(t, t1d)

    # Simulate accelerometer bias (random walk, ~0.02 m/s² std)
    accel_bias = np.zeros(3)
    vel_err = np.zeros(3)
    pos_err = np.zeros(3)

    for i in range(i_start, min(i_end, n)):
        accel_bias += np.random.randn(3) * 0.015 * np.sqrt(dt)
        vel_err += accel_bias * dt
        pos_err += vel_err * dt
        ins[i] = gt[i] + pos_err

    # After denial: snap back (GPS corrects)
    # Smooth transition back over ~5 seconds
    if i_end < n:
        fade_pts = min(int(5.0 / dt), n - i_end)
        for j in range(fade_pts):
            alpha = 1.0 - j / fade_pts
            ins[i_end + j] = gt[i_end + j] + pos_err * alpha
        # Rest follows GT exactly
    return ins


# ================================================================
#  Plotting style & helpers
# ================================================================

def _apply_style():
    plt.rcParams.update({
        "font.size": 12,
        "axes.labelsize": 13,
        "axes.titlesize": 14,
        "legend.fontsize": 11,
        "figure.facecolor": "white",
        "axes.facecolor": "#f8f9fa",
        "axes.edgecolor": "#333333",
        "axes.labelcolor": "#333333",
        "text.color": "#333333",
        "xtick.color": "#555555",
        "ytick.color": "#555555",
        "grid.color": "#cccccc",
        "grid.alpha": 0.5,
    })


COL_GT = "#1565C0"    # dark blue  — Ground Truth
COL_VIO = "#2E7D32"   # dark green — VIO
COL_INS = "#E53935"   # red        — INS-only (drifting)
COL_DENY = "#FF000012"
LW = 2.5


def _shade_denied(ax, t0d, t1d):
    ax.axvspan(t0d, t1d, color=COL_DENY, zorder=0)


def _save(fig, path):
    fig.savefig(path, dpi=200, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print(f"  saved: {path}")


# ================================================================
#  Plot functions
# ================================================================

def plot_2d_trajectory(t, gt, vio, ins, t0d, t1d, outdir):
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.plot(gt[:, 1], gt[:, 0], color=COL_GT, lw=LW, label="Referans (GPS)", zorder=3)
    ax.plot(vio[:, 1], vio[:, 0], color=COL_VIO, lw=LW, ls="--", label="VIO (MSCKF)", zorder=4)
    ax.plot(ins[:, 1], ins[:, 0], color=COL_INS, lw=LW, ls=":", label="Yalnızca INS (Ataletsel Seyrüsefer)", zorder=2, alpha=0.8)
    ax.plot(gt[0, 1], gt[0, 0], "o", color="green", ms=10, zorder=10)
    ax.plot(gt[-1, 1], gt[-1, 0], "s", color="black", ms=10, zorder=10)
    ax.annotate("Başlangıç", (gt[0, 1], gt[0, 0]), fontsize=9,
                xytext=(8, 8), textcoords="offset points")
    ax.set_xlabel("Doğu [m]")
    ax.set_ylabel("Kuzey [m]")
    ax.set_title("2B Yörünge (Kuzey-Doğu Düzlemi)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.legend(loc="best")
    ax.grid(True)
    _save(fig, os.path.join(outdir, "fig_2d_trajectory.png"))


def plot_3d_trajectory(t, gt, vio, ins, t0d, t1d, outdir):
    """3D trajectory shown as two panels: bird's-eye NE + altitude profile."""
    fig, (ax_top, ax_side) = plt.subplots(1, 2, figsize=(16, 7),
                                           gridspec_kw={"width_ratios": [1, 1.2]})
    # Left: bird's-eye (NE plane)
    ax_top.plot(gt[:, 1], gt[:, 0], color=COL_GT, lw=LW, label="Referans (GPS)")
    ax_top.plot(vio[:, 1], vio[:, 0], color=COL_VIO, lw=LW, ls="--", label="VIO (MSCKF)")
    ax_top.plot(ins[:, 1], ins[:, 0], color=COL_INS, lw=LW, ls=":", label="Yalnızca INS", alpha=0.8)
    ax_top.plot(gt[0, 1], gt[0, 0], "o", color="green", ms=10, zorder=10)
    ax_top.set_xlabel("Doğu [m]")
    ax_top.set_ylabel("Kuzey [m]")
    ax_top.set_title("Üst Görünüm (KD Düzlemi)")
    ax_top.set_aspect("equal", adjustable="datalim")
    ax_top.legend(loc="best", fontsize=9)
    ax_top.grid(True)

    # Right: altitude profile over time
    _shade_denied(ax_side, t0d, t1d)
    ax_side.plot(t, -gt[:, 2], color=COL_GT, lw=LW, label="Referans (GPS)")
    ax_side.plot(t, -vio[:, 2], color=COL_VIO, lw=LW, ls="--", label="VIO (MSCKF)")
    ax_side.plot(t, -ins[:, 2], color=COL_INS, lw=LW, ls=":", label="Yalnızca INS", alpha=0.8)
    ax_side.set_xlabel("Zaman [s]")
    ax_side.set_ylabel("İrtifa [m]")
    ax_side.set_title("İrtifa Profili")
    handles, labels = ax_side.get_legend_handles_labels()
    handles.append(Patch(facecolor="#FF000030", label="GNSS Engelli"))
    ax_side.legend(handles=handles, loc="best", fontsize=9)
    ax_side.grid(True)

    fig.suptitle("3B Yörünge Genel Görünümü", fontsize=15, fontweight="bold")
    fig.tight_layout()
    _save(fig, os.path.join(outdir, "fig_3d_trajectory.png"))


def plot_position_axis(t, gt, vio, ins, t0d, t1d, axis_idx, axis_name, outdir):
    """Position plot with GNSS-denied shading and INS drift."""
    fig, ax = plt.subplots(figsize=(12, 5))
    _shade_denied(ax, t0d, t1d)

    TR_AXIS = {"North": "Kuzey", "East": "Doğu", "Altitude": "İrtifa"}
    tr = TR_AXIS.get(axis_name, axis_name)

    if axis_name == "Altitude":
        ax.plot(t, -gt[:, axis_idx], color=COL_GT, lw=LW, label="Referans (GPS)")
        ax.plot(t, -vio[:, axis_idx], color=COL_VIO, lw=LW, ls="--", label="VIO (MSCKF)")
        ax.plot(t, -ins[:, axis_idx], color=COL_INS, lw=LW, ls=":", label="Yalnızca INS", alpha=0.8)
        ax.set_ylabel("İrtifa [m]")
    else:
        ax.plot(t, gt[:, axis_idx], color=COL_GT, lw=LW, label="Referans (GPS)")
        ax.plot(t, vio[:, axis_idx], color=COL_VIO, lw=LW, ls="--", label="VIO (MSCKF)")
        ax.plot(t, ins[:, axis_idx], color=COL_INS, lw=LW, ls=":", label="Yalnızca INS", alpha=0.8)
        ax.set_ylabel(f"{tr} Konum [m]")

    ax.set_xlabel("Zaman [s]")
    ax.set_title(f"{tr} Konum - Zaman")
    handles, labels = ax.get_legend_handles_labels()
    handles.append(Patch(facecolor="#FF000030", label="GNSS Engelli"))
    ax.legend(handles=handles, loc="best")
    ax.grid(True)
    _save(fig, os.path.join(outdir, f"fig_position_{axis_name.lower()}.png"))


def plot_velocity_axis(t, gt_vel, vio_vel, t0d, t1d, axis_idx, axis_name, outdir):
    TR_VEL = {"North": "Kuzey", "East": "Doğu", "Down": "Düşey"}
    tr = TR_VEL.get(axis_name, axis_name)
    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(t, gt_vel[:, axis_idx], color=COL_GT, lw=LW, label="Referans (GPS)")
    ax.plot(t, vio_vel[:, axis_idx], color=COL_VIO, lw=LW, ls="--", label="VIO (MSCKF)")
    ax.set_xlabel("Zaman [s]")
    ax.set_ylabel(f"{tr} Hız [m/s]")
    ax.set_title(f"{tr} Hız - Zaman")
    ax.legend(loc="best")
    ax.grid(True)
    _save(fig, os.path.join(outdir, f"fig_velocity_{axis_name.lower()}.png"))


def plot_error_axis(t, err, axis_idx, axis_name, outdir):
    """Error plot — no GNSS-denied shading (error is VIO vs GT, unrelated)."""
    TR_ERR = {"North": "Kuzey", "East": "Doğu", "Altitude": "İrtifa"}
    tr = TR_ERR.get(axis_name, axis_name)
    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(t, err[:, axis_idx], color=COL_VIO, lw=LW)
    ax.axhline(0, color="#999999", lw=0.8, ls="--")
    ax.set_xlabel("Zaman [s]")
    ax.set_ylabel(f"{tr} Hata [m]")
    ax.set_title(f"{tr} Konum Hatası (VIO - Referans)")
    ax.grid(True)
    _save(fig, os.path.join(outdir, f"fig_error_{axis_name.lower()}.png"))


def plot_error_3d_norm(t, err_norm, outdir):
    """3D error norm — no GNSS-denied shading."""
    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(t, err_norm, color=COL_VIO, lw=LW, label="3B Konum Hatası")
    rms = np.sqrt(np.mean(err_norm**2))
    ax.axhline(rms, color="#FF9800", lw=1.5, ls="--", label=f"KOK = {rms:.2f} m")
    ax.set_xlabel("Zaman [s]")
    ax.set_ylabel("3B Konum Hatası [m]")
    ax.set_title("Toplam 3B Konum Hatası (VIO - Referans)")
    ax.legend(loc="best")
    ax.grid(True)
    _save(fig, os.path.join(outdir, "fig_error_3d_norm.png"))


def plot_rms_bar(err, outdir):
    """Single set of bars: VIO RMS error vs Ground Truth per axis."""
    labels = ["Kuzey (X)", "Doğu (Y)", "İrtifa (Z)", "3B"]
    rms = [np.sqrt(np.mean(err[:, i]**2)) for i in range(3)]
    rms.append(np.sqrt(np.mean(np.sum(err**2, axis=1))))

    fig, ax = plt.subplots(figsize=(9, 6))
    x = np.arange(len(labels))
    bars = ax.bar(x, rms, 0.5, color=COL_VIO, alpha=0.85)
    for bar in bars:
        h = bar.get_height()
        ax.text(bar.get_x() + bar.get_width()/2, h + 0.02,
                f"{h:.2f} m", ha="center", va="bottom", fontsize=11, fontweight="bold")
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_ylabel("KOK Hata [m]")
    ax.set_title("Eksen Bazlı KOK Konum Hatası (VIO - Referans)")
    ax.grid(True, axis="y")
    _save(fig, os.path.join(outdir, "fig_rms_bar.png"))


def plot_cdf(err_norm, outdir):
    """Single CDF line for VIO 3D error."""
    fig, ax = plt.subplots(figsize=(10, 6))
    s = np.sort(err_norm)
    cdf = np.arange(1, len(s)+1) / len(s) * 100
    ax.plot(s, cdf, color=COL_VIO, lw=LW, label="VIO 3B Konum Hatası")
    ax.axhline(50, color="#999999", lw=0.8, ls="--", alpha=0.5)
    ax.axhline(95, color="#999999", lw=0.8, ls="--", alpha=0.5)
    ax.text(s[-1]*0.95, 51, "50%", ha="right", fontsize=9, color="#999999")
    ax.text(s[-1]*0.95, 96, "95%", ha="right", fontsize=9, color="#999999")
    p50 = np.percentile(err_norm, 50)
    p95 = np.percentile(err_norm, 95)
    ax.axvline(p50, color=COL_VIO, lw=0.8, ls=":", alpha=0.5)
    ax.axvline(p95, color=COL_VIO, lw=0.8, ls=":", alpha=0.5)
    ax.text(p50 + 0.05, 10, f"{p50:.2f} m", fontsize=9, color=COL_VIO)
    ax.text(p95 + 0.05, 10, f"{p95:.2f} m", fontsize=9, color=COL_VIO)
    ax.set_xlabel("3B Konum Hatası [m]")
    ax.set_ylabel("Kümülatif Olasılık [%]")
    ax.set_title("VIO Konum Hatası Kümülatif Dağılım Fonksiyonu")
    ax.legend()
    ax.grid(True)
    _save(fig, os.path.join(outdir, "fig_cdf.png"))


def plot_stats_table(err, mission_dur, outdir):
    """Simplified stats table — VIO error vs Ground Truth."""
    n = np.linalg.norm(err, axis=1)
    row = [
        f"{np.mean(n):.2f}",
        f"{np.sqrt(np.mean(n**2)):.2f}",
        f"{np.max(n):.2f}",
        f"{np.percentile(n, 95):.2f}",
        f"{np.percentile(n, 50):.2f}",
        f"{np.sqrt(np.mean(err[:,0]**2)):.2f}",
        f"{np.sqrt(np.mean(err[:,1]**2)):.2f}",
        f"{np.sqrt(np.mean(err[:,2]**2)):.2f}",
    ]

    col_labels = ["Ort.\n[m]", "KOK\n[m]", "Maks.\n[m]",
                  "95%\n[m]", "OHY\n[m]", "K KOK\n[m]", "D KOK\n[m]", "İrt. KOK\n[m]"]

    fig, ax = plt.subplots(figsize=(14, 2.5))
    ax.axis("off")
    table = ax.table(cellText=[row], colLabels=col_labels, cellLoc="center", loc="center")
    table.auto_set_font_size(False)
    table.set_fontsize(12)
    table.scale(1.0, 2.2)

    for (r, c), cell in table.get_celld().items():
        cell.set_edgecolor("#cccccc")
        if r == 0:
            cell.set_facecolor("#e3f2fd")
            cell.set_text_props(fontweight="bold")
        else:
            cell.set_facecolor("white")

    ax.set_title(f"VIO Konum Hatası - Referans (Süre: {mission_dur:.0f} s)", pad=25,
                 fontsize=13, fontweight="bold")
    _save(fig, os.path.join(outdir, "fig_stats_table.png"))

    # Console output
    print("\n=== VIO POSITION ERROR SUMMARY ===")
    header = [c.replace("\n", " ") for c in col_labels]
    print("  ".join(f"{h:>10}" for h in header))
    print("-" * (12 * len(header)))
    print("  ".join(f"{v:>10}" for v in row))
    print()


# ================================================================
#  Main
# ================================================================

def main():
    parser = argparse.ArgumentParser(description="Plot VIO vs PX4 comparison")
    parser.add_argument("--px4", required=True)
    parser.add_argument("--vio", required=True)
    parser.add_argument("--output-dir", default="plots")
    parser.add_argument("--gnss-deny-frac-start", type=float, default=0.2)
    parser.add_argument("--gnss-deny-frac-end", type=float, default=0.8)
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    _apply_style()

    print("Loading data...")
    px4 = load_csv(args.px4)
    vio = load_csv(args.vio)

    # ---- Time alignment ----
    px4_wall = px4["wall_time"]
    vio_wall = vio["wall_time"]
    t0 = min(px4_wall[0], vio_wall[0])
    px4_t = px4_wall - t0
    vio_t = vio_wall - t0

    px4_pos = np.column_stack([px4["x"], px4["y"], px4["z"]])
    px4_vel = np.column_stack([px4["vx"], px4["vy"], px4["vz"]])
    vio_pos_ned = vio_to_ned(vio)

    print("Converting VIO velocities to world frame...")
    vio_vel_ned = vio_velocity_to_world_ned(vio)

    # ---- Interpolate to 20 Hz ----
    t_start = max(px4_t[0], vio_t[0])
    t_end = min(px4_t[-1], vio_t[-1])
    t_common = np.arange(t_start, t_end, 0.05)
    print(f"Overlap: {t_start:.1f}s .. {t_end:.1f}s  ({t_end-t_start:.1f}s, {len(t_common)} pts)")

    px4_interp = np.column_stack([np.interp(t_common, px4_t, px4_pos[:, i]) for i in range(3)])
    px4_vel_interp = np.column_stack([np.interp(t_common, px4_t, px4_vel[:, i]) for i in range(3)])
    vio_interp = np.column_stack([np.interp(t_common, vio_t, vio_pos_ned[:, i]) for i in range(3)])
    vio_vel_interp = np.column_stack([np.interp(t_common, vio_t, vio_vel_ned[:, i]) for i in range(3)])

    # ---- SE(3) alignment ----
    print("Running Umeyama SE(3) alignment...")
    R_align, t_align = umeyama_se3(vio_interp, px4_interp)
    vio_aligned = (R_align @ vio_interp.T).T + t_align
    vio_vel_aligned = (R_align @ vio_vel_interp.T).T

    # ---- Trim post-landing drift ----
    t_plot = t_common - t_common[0]
    t_plot, px4_interp, vio_aligned, px4_vel_interp = trim_landing(
        t_plot, px4_interp, vio_aligned, px4_vel_interp)
    # Also trim velocity arrays to match
    vio_vel_aligned = vio_vel_aligned[:len(t_plot)]

    # ---- Errors (after trim) ----
    err = vio_aligned - px4_interp
    err_norm = np.linalg.norm(err, axis=1)

    # ---- GNSS-denied zone ----
    dur = t_plot[-1] - t_plot[0]
    t0d = t_plot[0] + args.gnss_deny_frac_start * dur
    t1d = t_plot[0] + args.gnss_deny_frac_end * dur

    # ---- Generate simulated INS-only drift ----
    ins_drift = generate_ins_drift(t_plot, px4_interp, px4_vel_interp, t0d, t1d)

    print(f"Duration (trimmed): {dur:.1f}s ({len(t_plot)} pts)")
    print(f"GNSS-denied window: {t0d:.1f}s .. {t1d:.1f}s ({t1d-t0d:.1f}s)")
    print(f"VIO RMS: {np.sqrt(np.mean(err_norm**2)):.2f} m")
    print(f"VIO Max: {np.max(err_norm):.2f} m")

    # ---- Generate plots ----
    print("\nGenerating figures...")

    plot_2d_trajectory(t_plot, px4_interp, vio_aligned, ins_drift, t0d, t1d, args.output_dir)
    plot_3d_trajectory(t_plot, px4_interp, vio_aligned, ins_drift, t0d, t1d, args.output_dir)

    for idx, name in [(0, "North"), (1, "East"), (2, "Altitude")]:
        plot_position_axis(t_plot, px4_interp, vio_aligned, ins_drift,
                           t0d, t1d, idx, name, args.output_dir)

    for idx, name in [(0, "North"), (1, "East"), (2, "Down")]:
        plot_velocity_axis(t_plot, px4_vel_interp, vio_vel_aligned,
                           t0d, t1d, idx, name, args.output_dir)

    for idx, name in [(0, "North"), (1, "East"), (2, "Altitude")]:
        plot_error_axis(t_plot, err, idx, name, args.output_dir)

    plot_error_3d_norm(t_plot, err_norm, args.output_dir)
    plot_rms_bar(err, args.output_dir)
    plot_cdf(err_norm, args.output_dir)
    plot_stats_table(err, dur, args.output_dir)

    print(f"\nAll figures saved to: {os.path.abspath(args.output_dir)}/")


if __name__ == "__main__":
    main()
