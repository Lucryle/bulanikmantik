"""Build a controller object from its YAML config."""
from __future__ import annotations

from src.control.base import Controller
from src.control.pid import PID
from src.fuzzy.it2_flc import paper_it2_flc, pd_it2_flc


def build_controller(ctrl_cfg: dict, mf_setting: dict | None, dt: float) -> Controller:
    kind = ctrl_cfg["type"]
    if kind == "pid":
        return PID(ctrl_cfg["kp"], ctrl_cfg["ki"], ctrl_cfg["kd"], dt,
                   d_filter_tau=ctrl_cfg.get("d_filter_tau", 0.1))
    if kind == "it2_flc":
        if mf_setting is None:
            raise ValueError("it2_flc needs an mf_setting")
        return paper_it2_flc(ctrl_cfg, mf_setting)
    if kind == "it2_pd":
        if mf_setting is None:
            raise ValueError("it2_pd needs an mf_setting")
        return pd_it2_flc(ctrl_cfg, mf_setting, dt)
    raise ValueError(f"unknown controller type: {kind}")
