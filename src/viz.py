"""Plotting helpers (membership functions, control curves, trajectories)."""
from __future__ import annotations

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

X = np.linspace(-1.0, 1.0, 801)


def plot_it2_terms(ax, names, terms, title=""):
    """Draw each IT2 MF as its FOU (shaded) between the lower and upper MF."""
    colors = plt.cm.viridis(np.linspace(0.05, 0.9, len(terms)))
    for name, mf, color in zip(names, terms, colors):
        lo, up = mf.lower(X), mf.upper(X)
        ax.fill_between(X, lo, up, color=color, alpha=0.3)
        ax.plot(X, up, color=color, lw=1.2)
        ax.plot(X, lo, color=color, lw=1.2, ls="--")
        ax.text(mf.upper.b, 1.04, name, ha="center", fontsize=8)
    ax.set_xlim(-1, 1)
    ax.set_ylim(0, 1.12)
    ax.set_xlabel("normalised error  k = Δ / half image size")
    ax.set_ylabel("membership")
    ax.set_title(title)
    ax.grid(alpha=0.3)


def plot_trial(result: dict, path: str, title: str = "") -> None:
    """Top view of one trial + tracking error over time."""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
    tgt, uav, est = result["target_pos"], result["uav_pos"], result["est_pos"]
    ax1.plot(tgt[:, 0], tgt[:, 1], "k-", lw=1.5, label="target (true)")
    ax1.plot(uav[:, 0], uav[:, 1], "C0-", lw=1.2, label="UAV")
    valid = ~np.isnan(est[:, 0])
    ax1.plot(est[valid, 0], est[valid, 1], "C1:", lw=1.0, label="PF estimate")
    ax1.plot(*tgt[0], "ko", ms=5)
    ax1.plot(*uav[0], "C0s", ms=5)
    for x, y, r in result.get("occlusion_zones", []):
        ax1.add_patch(plt.Circle((x, y), r, color="grey", alpha=0.3))
    ax1.set_aspect("equal")
    ax1.set_xlabel("x (m)")
    ax1.set_ylabel("y (m)")
    ax1.legend(fontsize=8)
    ax1.grid(alpha=0.3)
    ax1.set_title("top view")

    t = result["t"]
    ax2.plot(t, result["error"], "C0-", lw=1.0, label="UAV-target distance")
    ax2.plot(t, result["est_error"], "C1-", lw=0.8, alpha=0.7, label="PF estimation error")
    miss = ~result["measured"]
    ax2.fill_between(t, 0, 1, where=miss, transform=ax2.get_xaxis_transform(),
                     color="grey", alpha=0.15, lw=0, label="no detection")
    ax2.set_xlabel("time (s)")
    ax2.set_ylabel("m")
    ax2.legend(fontsize=8)
    ax2.grid(alpha=0.3)
    ax2.set_title("errors")
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def method_boxplots(df, x, order, methods, path, xlabel):
    """Box plots of the headline metrics: one box per (group ``x``, method)."""
    metrics = [("capture_time", "capture time (s)"), ("tracking_error", "tracking error after capture (m)"),
               ("mean_error_all", "mean error, whole trial (m)")]
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.2))
    width = 0.8 / len(methods)
    for ax, (col, label) in zip(axes, metrics):
        for i, m in enumerate(methods):
            data = [df[(df[x] == o) & (df["method"] == m)][col].dropna().values for o in order]
            pos = np.arange(len(order)) + (i - (len(methods) - 1) / 2) * width
            bp = ax.boxplot(data, positions=pos, widths=width * 0.9, patch_artist=True, showfliers=True,
                            flierprops={"markersize": 3})
            for patch in bp["boxes"]:
                patch.set_facecolor(f"C{i}")
                patch.set_alpha(0.6)
            ax.plot([], [], "s", color=f"C{i}", alpha=0.6, label=m)
        ax.set_xticks(range(len(order)), [str(o) for o in order])
        ax.set_xlabel(xlabel)
        ax.set_ylabel(label)
        ax.grid(alpha=0.3, axis="y")
    axes[0].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def sweep_plot(df, x, methods, path, xlabel, xscale="linear"):
    """Mean ± 95 % CI of the headline metrics versus a swept parameter ``x``."""
    from src.stats import mean_ci

    metrics = [("capture_time", "capture time (s)"), ("tracking_error", "tracking error after capture (m)"),
               ("mean_error_all", "mean error, whole trial (m)"), ("captured", "capture rate")]
    fig, axes = plt.subplots(1, 4, figsize=(18, 4))
    for ax, (col, label) in zip(axes, metrics):
        for i, m in enumerate(methods):
            sub = df[df["method"] == m]
            xs = sorted(sub[x].unique())
            stats = [mean_ci(sub[sub[x] == v][col].astype(float)) for v in xs]
            mean = np.array([s[0] for s in stats])
            lo = np.array([s[1] for s in stats])
            hi = np.array([s[2] for s in stats])
            ax.plot(xs, mean, "o-", color=f"C{i}", lw=1.4, ms=4, label=m)
            ax.fill_between(xs, lo, hi, color=f"C{i}", alpha=0.15)
        ax.set_xscale(xscale)
        ax.set_xlabel(xlabel)
        ax.set_ylabel(label)
        ax.grid(alpha=0.3)
    axes[0].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)
