"""Load YAML configs and merge scenario overrides into the defaults."""
from __future__ import annotations

import copy
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
CONFIGS = ROOT / "configs"


def load_yaml(path: str | Path) -> dict:
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def deep_merge(base: dict, override: dict) -> dict:
    """Return a copy of ``base`` with ``override`` applied recursively."""
    out = copy.deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = deep_merge(out[key], value)
        else:
            out[key] = copy.deepcopy(value)
    return out


def load_config(scenario: str | None = None, overrides: dict | None = None) -> dict:
    """defaults <- scenario file (e.g. "A") <- explicit overrides."""
    cfg = load_yaml(CONFIGS / "default.yaml")
    if scenario:
        cfg = deep_merge(cfg, load_yaml(CONFIGS / "scenarios" / f"scenario_{scenario}.yaml"))
    if overrides:
        cfg = deep_merge(cfg, overrides)
    return cfg


def load_controller(name: str) -> dict:
    return load_yaml(CONFIGS / "controllers" / f"{name}.yaml")


def load_mf_setting(name: str) -> dict:
    return load_yaml(CONFIGS / "mf_settings" / f"{name}.yaml")
