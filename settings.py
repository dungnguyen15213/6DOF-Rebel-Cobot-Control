"""Loads centralized analytics tuning parameters from config.yaml.

Usage:
    from settings import get_settings
    settings = get_settings()
    settings.ff_rls.forgetting_factor
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Any

import yaml

CONFIG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config.yaml")


@dataclass
class FFRLSSettings:
    forgetting_factor: float = 0.98
    initial_covariance: float = 1000.0
    n_params: int = 3


@dataclass
class CUSUMSettings:
    k: float = 0.5
    h: float = 5.0


@dataclass
class PayloadSettings:
    filter_alpha: float = 0.05
    mass_sensitivity_gain: float = 200.0
    attach_threshold_g: float = 50.0


@dataclass
class LoggingSettings:
    output_dir: str = "experiments"
    run_prefix: str = "run"
    flush_every: int = 1


@dataclass
class Settings:
    ff_rls: FFRLSSettings = field(default_factory=FFRLSSettings)
    cusum: CUSUMSettings = field(default_factory=CUSUMSettings)
    payload: PayloadSettings = field(default_factory=PayloadSettings)
    logging: LoggingSettings = field(default_factory=LoggingSettings)


def _load_yaml(path: str) -> dict[str, Any]:
    if not os.path.exists(path):
        return {}
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def load_settings(path: str = CONFIG_PATH) -> Settings:
    """Parses config.yaml into a typed Settings object, falling back to
    dataclass defaults for any missing keys."""
    raw = _load_yaml(path)
    return Settings(
        ff_rls=FFRLSSettings(**raw.get("ff_rls", {})),
        cusum=CUSUMSettings(**raw.get("cusum", {})),
        payload=PayloadSettings(**raw.get("payload", {})),
        logging=LoggingSettings(**raw.get("logging", {})),
    )


_settings_singleton: Settings | None = None


def get_settings(reload: bool = False) -> Settings:
    """Returns the process-wide Settings singleton, loading it on first use.

    Args:
        reload: If True, re-reads config.yaml from disk (useful when tuning
            parameters between lab experiment runs without restarting).
    """
    global _settings_singleton
    if _settings_singleton is None or reload:
        _settings_singleton = load_settings()
    return _settings_singleton


def set_settings(new_settings: Settings) -> None:
    """Replaces the process-wide Settings singleton (e.g. after the GUI
    applies edited tuning values) without touching config.yaml on disk."""
    global _settings_singleton
    _settings_singleton = new_settings


def save_settings(new_settings: Settings, path: str = CONFIG_PATH) -> None:
    """Persists the given Settings to config.yaml and updates the singleton
    so subsequently created estimators pick up the saved values."""
    raw = {
        "ff_rls": {
            "forgetting_factor": new_settings.ff_rls.forgetting_factor,
            "initial_covariance": new_settings.ff_rls.initial_covariance,
            "n_params": new_settings.ff_rls.n_params,
        },
        "cusum": {
            "k": new_settings.cusum.k,
            "h": new_settings.cusum.h,
        },
        "payload": {
            "filter_alpha": new_settings.payload.filter_alpha,
            "mass_sensitivity_gain": new_settings.payload.mass_sensitivity_gain,
            "attach_threshold_g": new_settings.payload.attach_threshold_g,
        },
        "logging": {
            "output_dir": new_settings.logging.output_dir,
            "run_prefix": new_settings.logging.run_prefix,
            "flush_every": new_settings.logging.flush_every,
        },
    }
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(raw, f, sort_keys=False)
    set_settings(new_settings)
