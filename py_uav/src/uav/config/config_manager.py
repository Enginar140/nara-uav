"""YAML konfigurasyon okuyucu. Noktali yol (dotted-path) erisimi.

C++ ConfigManager ile ayni arayuz: get(key, default), nested erisim "a.b.c".
"""
from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml


class ConfigManager:
    def __init__(self, config_path: str | Path):
        self.path = Path(config_path)
        if not self.path.exists():
            raise FileNotFoundError(f"Config bulunamadi: {self.path}")
        with self.path.open("r", encoding="utf-8") as f:
            self._root: dict = yaml.safe_load(f) or {}

    def _navigate(self, dotted: str) -> Any:
        node: Any = self._root
        for part in dotted.split("."):
            if isinstance(node, dict) and part in node:
                node = node[part]
            else:
                return None
        return node

    def has_key(self, key: str) -> bool:
        return self._navigate(key) is not None

    def get(self, key: str, default: Any = None) -> Any:
        val = self._navigate(key)
        return default if val is None else val

    def get_str(self, key: str, default: str = "") -> str:
        val = self._navigate(key)
        return default if val is None else str(val)

    def get_int(self, key: str, default: int = 0) -> int:
        val = self._navigate(key)
        try:
            return default if val is None else int(val)
        except (TypeError, ValueError):
            return default

    def get_float(self, key: str, default: float = 0.0) -> float:
        val = self._navigate(key)
        try:
            return default if val is None else float(val)
        except (TypeError, ValueError):
            return default

    def get_bool(self, key: str, default: bool = False) -> bool:
        val = self._navigate(key)
        if val is None:
            return default
        if isinstance(val, bool):
            return val
        if isinstance(val, str):
            return val.strip().lower() in ("1", "true", "yes", "on")
        return bool(val)

    def get_node(self, key: str) -> Any:
        return self._navigate(key)

    # Kolaylik
    def pixhawk_connection(self) -> str:
        return self.get_str("mavlink.pixhawk.connection", "udpin:127.0.0.1:14550")

    def gcs_connection(self) -> str:
        return self.get_str("mavlink.gcs.connection", "")
