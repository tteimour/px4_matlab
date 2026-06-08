#!/usr/bin/env python3
"""Compute ATE and RPE statistics for OpenVINS vs PX4 ground truth.

Reads the latest px4_*.csv / vio_*.csv pair recorded by record_vio_comparison.py
and writes a plain-text statistics file (and a LaTeX table) suitable for a
conference paper.

Methodology (see also methodology section in the output file):

  1. Time alignment:
       - Both streams are timestamped with wall_time. We compute the overlapping
         interval, build a common 20 Hz grid, and linearly interpolate position
         (and SLERP for VIO orientation) onto it.

  2. Frames:
       - VIO publishes position in an ENU world frame and orientation as a
         body-FLU -> world-ENU rotation.
       - PX4 ground truth is body-FRD in NED world.

  3. SE(3) trajectory alignment (Umeyama, no scale):
       - We solve (R*, t*) = argmin sum ||R* p_vio_ENU + t - p_gt_NED||^2.
       - R* therefore captures the full world-frame ENU -> NED rotation
         (plus any residual heading offset between VIO's arbitrary world
         frame and PX4's NED).
       - For orientation, R* is applied on the world side and the FLU->FRD
         body fix R_x(pi) on the body side:
            R_FRD_to_NED = R* @ R_vio @ R_FLU_to_FRD .
       - A residual constant yaw offset (initial-heading bias from VIO's
         arbitrary world frame) is removed using the median of the first 10 s
         of yaw error.

  4. Post-landing trim:
       - Samples after the drone descends below 1.5 m altitude are dropped to
         avoid double-counting drift accumulated when stationary on the ground.

  5. Absolute Trajectory Error (ATE):
       - Position:  e_p(t_i) = R*p_vio(t_i) + t* - p_gt(t_i)
       - Orientation (yaw): e_y(t_i) = wrap_pi( yaw_vio_NED(t_i) - yaw_gt(t_i) )
       - Reported as RMSE / mean(|.|) / median(|.|) / 95th percentile / max.

  6. Relative Pose Error (RPE) at horizon Delta:
       - For each pair (i, i+k) with k = round(Delta/dt):
           dp_gt  = p_gt(t_{i+k}) - p_gt(t_i)
           dp_vio = aligned VIO equivalent
           dy_gt  = wrap_pi( yaw_gt(t_{i+k})  - yaw_gt(t_i)  )
           dy_vio = wrap_pi( yaw_vio(t_{i+k}) - yaw_vio(t_i) )
       - Translation error norm is RMS'd to give RPE_t [m / Delta].
       - Yaw error is RMS'd to give RPE_yaw [deg / Delta].
       - We also report % of distance travelled.
"""

import argparse
import glob
import os
import sys
from datetime import datetime

import numpy as np
from scipy.spatial.transform import Rotation, Slerp


# ---------------------------------------------------------------------------
#  Helpers
# ---------------------------------------------------------------------------

def load_csv(path):
    return np.genfromtxt(path, delimiter=",", names=True)


def umeyama_se3(src, dst):
    """Rigid SE(3) alignment, no scale.  dst ~ R @ src + t."""
    mu_s = src.mean(axis=0)
    mu_d = dst.mean(axis=0)
    H = (src - mu_s).T @ (dst - mu_d) / len(src)
    U, _, Vt = np.linalg.svd(H)
    d = np.linalg.det(Vt.T @ U.T)
    R_a = Vt.T @ np.diag([1.0, 1.0, d]) @ U.T
    t_a = mu_d - R_a @ mu_s
    return R_a, t_a


def wrap_pi(x):
    return (x + np.pi) % (2.0 * np.pi) - np.pi


def stats_block(arr):
    a = np.asarray(arr, dtype=float)
    abs_a = np.abs(a)
    return {
        "n": int(a.size),
        "rmse": float(np.sqrt(np.mean(a * a))),
        "mean_abs": float(np.mean(abs_a)),
        "std": float(np.std(a)),
        "median_abs": float(np.median(abs_a)),
        "p95": float(np.percentile(abs_a, 95)),
        "max": float(np.max(abs_a)),
    }


# ---------------------------------------------------------------------------
#  Main
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description="ATE/RPE for VIO vs PX4 ground truth")
    ap.add_argument("--px4")
    ap.add_argument("--vio")
    ap.add_argument(
        "--rec-dir",
        default=os.path.expanduser("~/ytu_thesis/simulation/openvins_ws/recorded_data"),
    )
    ap.add_argument("--output")
    ap.add_argument("--rpe-deltas", type=float, nargs="+",
                    default=[1.0, 5.0, 10.0],
                    help="RPE horizons in seconds")
    ap.add_argument("--dt", type=float, default=0.05,
                    help="Common grid step in seconds (default 0.05 = 20 Hz)")
    args = ap.parse_args()

    if not args.px4 or not args.vio:
        px4_files = sorted(glob.glob(os.path.join(args.rec_dir, "px4_*.csv")))
        vio_files = sorted(glob.glob(os.path.join(args.rec_dir, "vio_*.csv")))
        if not px4_files or not vio_files:
            sys.exit(f"No recorded CSVs found in {args.rec_dir}")
        args.px4 = args.px4 or px4_files[-1]
        args.vio = args.vio or vio_files[-1]

    if not args.output:
        ts = os.path.splitext(os.path.basename(args.vio))[0].replace("vio_", "")
        args.output = os.path.join(args.rec_dir, f"errors_{ts}.txt")
    tex_output = os.path.splitext(args.output)[0] + ".tex"

    print(f"PX4: {args.px4}")
    print(f"VIO: {args.vio}")
    print(f"Output: {args.output}")
    print(f"LaTeX:  {tex_output}")

    px4 = load_csv(args.px4)
    vio = load_csv(args.vio)

    # -- Time base ----------------------------------------------------------
    t0 = min(px4["wall_time"][0], vio["wall_time"][0])
    px4_t = px4["wall_time"] - t0
    vio_t = vio["wall_time"] - t0

    # -- Ground truth (NED) -------------------------------------------------
    p_gt_raw = np.column_stack([px4["x"], px4["y"], px4["z"]])
    yaw_gt_raw = np.unwrap(px4["heading"])

    # -- VIO raw ENU position; orientation as quaternion -------------------
    p_vio_raw = np.column_stack([vio["x"], vio["y"], vio["z"]])
    q_vio_raw = np.column_stack([vio["qx"], vio["qy"], vio["qz"], vio["qw"]])

    # -- Common time grid ---------------------------------------------------
    t_start = max(px4_t[0], vio_t[0])
    t_end = min(px4_t[-1], vio_t[-1])
    if t_end - t_start < 5.0:
        sys.exit(f"Overlap too short: {t_end - t_start:.1f} s")
    t = np.arange(t_start, t_end, args.dt)

    p_gt = np.column_stack([np.interp(t, px4_t, p_gt_raw[:, k]) for k in range(3)])
    yaw_gt = np.interp(t, px4_t, yaw_gt_raw)
    p_vio = np.column_stack([np.interp(t, vio_t, p_vio_raw[:, k]) for k in range(3)])

    # SLERP for VIO quaternions
    rots = Rotation.from_quat(q_vio_raw)
    slerp = Slerp(vio_t, rots)
    t_clip = np.clip(t, vio_t[0], vio_t[-1])
    rots_i = slerp(t_clip)

    # -- SE(3) alignment ----------------------------------------------------
    R_align, t_align = umeyama_se3(p_vio, p_gt)
    p_vio_a = (R_align @ p_vio.T).T + t_align

    # Bring VIO orientation into NED world: R_world_NED_body_FRD
    R_FLU2FRD = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], dtype=float)
    yaw_vio = np.zeros(len(t))
    for i in range(len(t)):
        Rm = R_align @ rots_i[i].as_matrix() @ R_FLU2FRD
        yaw_vio[i] = np.arctan2(Rm[1, 0], Rm[0, 0])
    yaw_vio = np.unwrap(yaw_vio)

    # -- Trim post-landing drift -------------------------------------------
    alt = -p_gt[:, 2]
    if alt.max() > 2.0:
        above = np.where(alt > 1.5)[0]
        if len(above):
            cut = min(above[-1] + int(2.0 / args.dt), len(t))
            t, p_gt, p_vio_a = t[:cut], p_gt[:cut], p_vio_a[:cut]
            yaw_gt, yaw_vio = yaw_gt[:cut], yaw_vio[:cut]

    duration = float(t[-1] - t[0])

    # -- ATE ----------------------------------------------------------------
    e_pos = p_vio_a - p_gt
    e_pos_norm = np.linalg.norm(e_pos, axis=1)

    # Remove constant yaw offset (VIO world frame heading is arbitrary)
    raw_yaw_err = wrap_pi(yaw_vio - yaw_gt)
    init_n = max(1, min(int(10.0 / args.dt), len(raw_yaw_err) // 4))
    yaw_bias = float(np.median(raw_yaw_err[:init_n]))
    e_yaw = wrap_pi(raw_yaw_err - yaw_bias)
    e_yaw_deg = np.degrees(e_yaw)

    ate_pos_3d = stats_block(e_pos_norm)
    ate_pos_n = stats_block(e_pos[:, 0])
    ate_pos_e = stats_block(e_pos[:, 1])
    ate_pos_d = stats_block(e_pos[:, 2])
    ate_yaw = stats_block(e_yaw_deg)

    # Trajectory length (GT) for normalised metrics
    gt_path_len = float(np.sum(np.linalg.norm(np.diff(p_gt, axis=0), axis=1)))

    # -- RPE ----------------------------------------------------------------
    rpe = []
    for delta_s in args.rpe_deltas:
        k = max(1, int(round(delta_s / args.dt)))
        if k >= len(t):
            continue
        dp_gt = p_gt[k:] - p_gt[:-k]
        dp_vio = p_vio_a[k:] - p_vio_a[:-k]
        e_dp = dp_vio - dp_gt
        e_dp_norm = np.linalg.norm(e_dp, axis=1)
        dist_gt = np.linalg.norm(dp_gt, axis=1)
        # Avoid zero-distance rows
        valid = dist_gt > 1e-3
        pct = float(np.sqrt(np.mean(e_dp_norm[valid] ** 2)) /
                    max(np.mean(dist_gt[valid]), 1e-6) * 100.0)

        dy_gt = wrap_pi(yaw_gt[k:] - yaw_gt[:-k])
        dy_vio = wrap_pi(yaw_vio[k:] - yaw_vio[:-k])
        e_dy = np.degrees(wrap_pi(dy_vio - dy_gt))

        rpe.append({
            "delta_s": delta_s,
            "trans": stats_block(e_dp_norm),
            "trans_pct": pct,
            "yaw_deg": stats_block(e_dy),
        })

    # -- Write text report --------------------------------------------------
    lines = []
    push = lines.append
    push("=" * 72)
    push("  OpenVINS vs PX4 Ground Truth — ATE and RPE Statistics")
    push("=" * 72)
    push(f"Generated      : {datetime.now().isoformat(timespec='seconds')}")
    push(f"PX4 CSV        : {args.px4}")
    push(f"VIO CSV        : {args.vio}")
    push(f"Common grid    : {args.dt*1000:.0f} ms ({1.0/args.dt:.0f} Hz)")
    push(f"Duration       : {duration:.1f} s ({len(t)} samples)")
    push(f"GT path length : {gt_path_len:.2f} m")
    push(f"Yaw bias removed: {np.degrees(yaw_bias):+.3f} deg "
         f"(initial-heading offset between VIO and PX4 world frames)")
    push("")
    push("Notes:")
    push("  - Position errors are after Umeyama SE(3) alignment of VIO -> PX4.")
    push("  - PX4 only publishes yaw (heading); orientation error is yaw-only.")
    push("  - 'rmse' = sqrt(mean(e^2)); abs/median/p95/max use |e|.")
    push("")

    push("-" * 72)
    push("  ABSOLUTE TRAJECTORY ERROR (ATE)")
    push("-" * 72)
    push(f"{'Metric':<28}{'RMSE':>10}{'Mean|.|':>10}{'Median':>10}"
         f"{'95%':>10}{'Max':>10}")
    def row(name, s, unit):
        return (f"{name+' ['+unit+']':<28}"
                f"{s['rmse']:>10.4f}{s['mean_abs']:>10.4f}"
                f"{s['median_abs']:>10.4f}{s['p95']:>10.4f}{s['max']:>10.4f}")
    push(row("ATE position 3D",       ate_pos_3d, "m"))
    push(row("ATE position North",    ate_pos_n,  "m"))
    push(row("ATE position East",     ate_pos_e,  "m"))
    push(row("ATE position Down",     ate_pos_d,  "m"))
    push(row("ATE yaw",               ate_yaw,    "deg"))
    push("")

    push("-" * 72)
    push("  RELATIVE POSE ERROR (RPE)")
    push("-" * 72)
    push(f"{'Horizon':<10}{'Trans RMSE':>14}{'Trans %':>10}"
         f"{'Trans Max':>12}{'Yaw RMSE':>12}{'Yaw Max':>10}")
    push(f"{'[s]':<10}{'[m]':>14}{'[%]':>10}{'[m]':>12}"
         f"{'[deg]':>12}{'[deg]':>10}")
    for r in rpe:
        push(f"{r['delta_s']:<10.1f}"
             f"{r['trans']['rmse']:>14.4f}"
             f"{r['trans_pct']:>10.2f}"
             f"{r['trans']['max']:>12.4f}"
             f"{r['yaw_deg']['rmse']:>12.4f}"
             f"{r['yaw_deg']['max']:>10.4f}")
    push("")

    with open(args.output, "w") as f:
        f.write("\n".join(lines) + "\n")

    # -- LaTeX table --------------------------------------------------------
    tex = []
    tex.append("% Auto-generated by analyze_errors.py")
    tex.append("% Source: " + os.path.basename(args.vio))
    tex.append("\\begin{table}[t]")
    tex.append("  \\centering")
    tex.append(f"  \\caption{{OpenVINS hata istatistikleri "
               f"(uçuş süresi {duration:.0f}~s, kat edilen mesafe "
               f"{gt_path_len:.1f}~m).}}")
    tex.append("  \\label{tab:vio-errors}")
    tex.append("  \\begin{tabular}{lrrrrr}")
    tex.append("    \\hline")
    tex.append("    Metrik & KOK & Ort.$|\\cdot|$ & Medyan & 95\\% & Maks. \\\\")
    tex.append("    \\hline")
    def trow(label, s, fmt="%.3f"):
        return ("    " + label + " & " +
                " & ".join(fmt % v for v in
                           [s["rmse"], s["mean_abs"], s["median_abs"],
                            s["p95"], s["max"]]) +
                " \\\\")
    tex.append("    \\multicolumn{6}{l}{\\textit{Mutlak Yörünge Hatası (ATE)}} \\\\")
    tex.append(trow("Konum 3B [m]",     ate_pos_3d))
    tex.append(trow("Konum Kuzey [m]",  ate_pos_n))
    tex.append(trow("Konum Doğu [m]",   ate_pos_e))
    tex.append(trow("Konum Aşağı [m]",  ate_pos_d))
    tex.append(trow("Yönelim (sapma) [deg]", ate_yaw))
    tex.append("    \\hline")
    tex.append("    \\multicolumn{6}{l}{\\textit{Bağıl Poz Hatası (RPE)}} \\\\")
    for r in rpe:
        d = r["delta_s"]
        tex.append(trow(f"Konum $\\Delta={d:.0f}$~s [m]", r["trans"]))
        tex.append(trow(f"Yönelim $\\Delta={d:.0f}$~s [deg]", r["yaw_deg"]))
    tex.append("    \\hline")
    tex.append("  \\end{tabular}")
    tex.append("\\end{table}")
    with open(tex_output, "w") as f:
        f.write("\n".join(tex) + "\n")

    print("\n" + "\n".join(lines))


if __name__ == "__main__":
    main()
