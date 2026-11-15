"""Live demo: the same trial flown by several controllers, animated side by side.

Every controller gets the same seed, so the target path, the detector noise and
the wind are identical (paired comparison). The trials are simulated first and
then replayed as an animation; no internet connection is needed.

Usage examples:
  python demo/live_demo.py
  python demo/live_demo.py --scenario C --wind 9.6
  python demo/live_demo.py --scenario A --target-speed 1.8 --seed 4
  python demo/live_demo.py --sigma-px 20 --p-miss 0.3
  python demo/live_demo.py --controllers pid_tuned it2_paper_tuned
  python demo/live_demo.py --save results/figures/demo.gif     # no window, write a GIF

Keys in the window: space = pause / resume, q = quit.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import matplotlib  # noqa: E402

if "--save" in sys.argv:
    matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.animation import FuncAnimation, PillowWriter  # noqa: E402

from src.config import load_config, load_controller, load_mf_setting  # noqa: E402
from src.metrics import trial_metrics  # noqa: E402
from src.sim.environment import run_trial  # noqa: E402

LABELS = {
    "pid_tuned": "PID",
    "it2_paper_tuned": "IT2-FLC (paper)",
    "it2_pd_tuned": "IT2-FPD (proposed)",
}


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", default="C", choices=["A", "B", "C"])
    ap.add_argument("--controllers", nargs="+",
                    default=["pid_tuned", "it2_paper_tuned", "it2_pd_tuned"],
                    help="file names in configs/controllers/")
    ap.add_argument("--mf", default="benchmark", help="file name in configs/mf_settings/")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--wind", type=float, default=None, help="wind speed W in m/s (paper: 4.9, 9.6, 12.1)")
    ap.add_argument("--target-speed", type=float, default=None, help="fixed target speed in m/s (UAV max is 2)")
    ap.add_argument("--sigma-px", type=float, default=None, help="detector pixel noise (px)")
    ap.add_argument("--p-miss", type=float, default=None, help="probability of a missed detection")
    ap.add_argument("--no-pf", action="store_true", help="use raw detections instead of the particle filter")
    ap.add_argument("--duration", type=float, default=None, help="trial length in s (default 40)")
    ap.add_argument("--speedup", type=float, default=2.0, help="playback speed (1 = real time)")
    ap.add_argument("--save", default=None, help="write the animation to this .gif instead of showing it")
    return ap.parse_args()


def build_overrides(args: argparse.Namespace) -> dict:
    ov: dict = {"particle_filter": {"enabled": not args.no_pf}}
    if args.wind is not None:
        ov["wind"] = {"speed": args.wind}
    if args.target_speed is not None:
        ov["target"] = {"speed_range": [args.target_speed, args.target_speed]}
    det = {}
    if args.sigma_px is not None:
        det["sigma_px"] = args.sigma_px
    if args.p_miss is not None:
        det["p_miss"] = args.p_miss
    if det:
        ov["detector"] = det
    if args.duration is not None:
        ov["sim"] = {"duration": args.duration}
    return ov


def main() -> None:
    args = parse_args()
    cfg = load_config(args.scenario, build_overrides(args))
    # the B scenario's illumination change overrides sigma_px; keep the user's value
    if args.sigma_px is not None and "illumination" in cfg["detector"]:
        cfg["detector"].pop("illumination")
    mf = load_mf_setting(args.mf)

    runs = []
    for name in args.controllers:
        rec = run_trial(cfg, load_controller(name), mf, args.seed)
        m = trial_metrics(rec, cfg)
        runs.append((LABELS.get(name, name), rec, m))
        cap = f"{m['capture_time']:.1f} s" if m["captured"] else "not captured"
        after = f"{m['tracking_error']:.2f} m" if m["captured"] else "-"
        print(f"{LABELS.get(name, name):<20} capture: {cap:<14} "
              f"mean error: {m['mean_error_all']:.2f} m  error after capture: {after}")

    t = runs[0][1]["t"]
    dt = cfg["sim"]["dt"]
    stride = max(1, int(round(args.speedup)))       # frames skipped per animation step
    frames = range(0, len(t), stride)
    radius = cfg["metrics"]["capture_radius"]

    fig, (ax_map, ax_err) = plt.subplots(1, 2, figsize=(13, 6), gridspec_kw={"width_ratios": [1.1, 1]})
    title = f"Scenario {args.scenario} | seed {args.seed}"
    if args.wind:
        title += f" | wind {args.wind} m/s"
    if args.target_speed:
        title += f" | target {args.target_speed} m/s"
    if args.no_pf:
        title += " | no particle filter"
    fig.suptitle(title)

    # top view: fixed limits that contain every path
    allxy = np.vstack([r[1]["target_pos"] for r in runs] + [r[1]["uav_pos"] for r in runs])
    lo, hi = allxy.min(0) - 2, allxy.max(0) + 2
    ax_map.set_xlim(lo[0], hi[0])
    ax_map.set_ylim(lo[1], hi[1])
    ax_map.set_aspect("equal")
    ax_map.set_xlabel("x (m)")
    ax_map.set_ylabel("y (m)")
    ax_map.grid(alpha=0.3)
    for x, y, r in cfg["detector"].get("occlusion_zones", []):
        ax_map.add_patch(plt.Circle((x, y), r, color="grey", alpha=0.3))
    tgt = runs[0][1]["target_pos"]
    (tgt_line,) = ax_map.plot([], [], "k-", lw=1.5, label="target")
    (tgt_dot,) = ax_map.plot([], [], "k*", ms=12)

    ax_err.set_xlim(0, t[-1])
    ax_err.set_ylim(0, max(np.nanmax(r[1]["error"]) for r in runs) * 1.05)
    ax_err.axhline(radius, color="grey", ls="--", lw=1, label=f"capture radius {radius} m")
    ax_err.set_xlabel("time (s)")
    ax_err.set_ylabel("UAV–target distance (m)")
    ax_err.grid(alpha=0.3)

    artists = []
    for i, (label, rec, _) in enumerate(runs):
        c = f"C{i}"
        (path,) = ax_map.plot([], [], c + "-", lw=1.2, label=label)
        (dot,) = ax_map.plot([], [], c + "o", ms=7)
        (head,) = ax_map.plot([], [], c + "-", lw=2)
        (err,) = ax_err.plot([], [], c + "-", lw=1.4, label=label)
        artists.append((rec, path, dot, head, err))
    ax_map.legend(loc="upper left", fontsize=8)
    ax_err.legend(loc="upper right", fontsize=8)
    time_txt = ax_map.text(0.98, 0.02, "", transform=ax_map.transAxes, ha="right", fontsize=10)

    def update(k: int):
        tgt_line.set_data(tgt[: k + 1, 0], tgt[: k + 1, 1])
        tgt_dot.set_data([tgt[k, 0]], [tgt[k, 1]])
        out = [tgt_line, tgt_dot, time_txt]
        for rec, path, dot, head, err in artists:
            p = rec["uav_pos"]
            path.set_data(p[: k + 1, 0], p[: k + 1, 1])
            dot.set_data([p[k, 0]], [p[k, 1]])
            yaw = rec["uav_yaw"][k]          # short line = heading of the UAV
            head.set_data([p[k, 0], p[k, 0] + 1.2 * np.cos(yaw)], [p[k, 1], p[k, 1] + 1.2 * np.sin(yaw)])
            err.set_data(rec["t"][: k + 1], rec["error"][: k + 1])
            out += [path, dot, head, err]
        time_txt.set_text(f"t = {t[k]:.1f} s")
        return out

    interval_ms = 1000 * dt * stride / max(args.speedup, 1e-6)   # simulated time / playback speed
    anim = FuncAnimation(fig, update, frames=frames, interval=interval_ms, blit=False, repeat=False)

    if args.save:
        out = Path(args.save)
        out.parent.mkdir(parents=True, exist_ok=True)
        anim.save(out, writer=PillowWriter(fps=max(1, int(1000 / interval_ms))))
        print(f"animation written to {out}")
        return

    paused = {"on": False}

    def on_key(event):
        if event.key == " ":
            (anim.resume if paused["on"] else anim.pause)()
            paused["on"] = not paused["on"]
        elif event.key == "q":
            plt.close(fig)

    fig.canvas.mpl_connect("key_press_event", on_key)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
