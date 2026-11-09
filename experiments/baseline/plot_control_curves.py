"""Figure R4: IT2 membership functions and controller input-output curves.

Usage:  python experiments/baseline/plot_control_curves.py
Output: results/figures/mf_settings.png, results/figures/control_curves.png
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from src.config import ROOT, load_controller, load_mf_setting  # noqa: E402
from src.control.factory import build_controller  # noqa: E402
from src.fuzzy.membership import build_it2_terms  # noqa: E402
from src.viz import X, plot_it2_terms  # noqa: E402

SETTINGS = ["benchmark", "setting1", "setting2"]
OUT = ROOT / "results" / "figures"


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    flc_cfg = load_controller("it2_paper")
    pid_cfg = load_controller("pid")

    # 1) membership functions of the three paper settings
    fig, axes = plt.subplots(1, 3, figsize=(15, 3.8), sharey=True)
    for ax, name in zip(axes, SETTINGS):
        names, terms = build_it2_terms(load_mf_setting(name), flc_cfg["fou_delta"],
                                       flc_cfg["lower_height"], flc_cfg["mf_center"])
        plot_it2_terms(ax, names, terms, title=f"{name}  (FOU δ={flc_cfg['fou_delta']})")
    fig.tight_layout()
    fig.savefig(OUT / "mf_settings.png", dpi=130)
    plt.close(fig)

    # 2) input-output (control) curves
    fig, ax = plt.subplots(figsize=(7, 5))
    for name in SETTINGS:
        flc = build_controller(flc_cfg, load_mf_setting(name), dt=0.05)
        y = [flc.compute(e) for e in X]
        ax.plot(X, y, lw=1.6, label=f"IT2 FLC – {name}  (y(0) = {flc.compute(0.0):+.3f})")
    ax.plot(X, np.clip(pid_cfg["kp"] * X, -1, 1), "k--", lw=1.0,
            label=f"P part of PID (kp = {pid_cfg['kp']})")
    ax.axhline(0, color="grey", lw=0.6)
    ax.axvline(0, color="grey", lw=0.6)
    ax.set_xlabel("normalised error  k_x = Δx/x_c  (or k_y = Δy/y_c)")
    ax.set_ylabel("normalised command  k_yaw  (or k_v)")
    ax.set_title("Controller input–output curves")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(OUT / "control_curves.png", dpi=130)
    plt.close(fig)
    print(f"saved figures to {OUT}")


if __name__ == "__main__":
    main()
