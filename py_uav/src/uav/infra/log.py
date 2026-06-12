"""Loglama kurulumu. stdlib logging; dosya rotasyonu + konsol."""
from __future__ import annotations

import logging
from logging.handlers import RotatingFileHandler
from pathlib import Path

_LEVELS = {
    "debug": logging.DEBUG,
    "info": logging.INFO,
    "warn": logging.WARNING,
    "warning": logging.WARNING,
    "error": logging.ERROR,
}

_initialized = False


def setup_logging(
    level: str = "info",
    log_dir: str = "logs",
    max_size_mb: int = 10,
    backup_count: int = 5,
    to_console: bool = True,
) -> None:
    global _initialized
    root = logging.getLogger("uav")
    root.setLevel(_LEVELS.get(level.lower(), logging.INFO))
    root.handlers.clear()

    fmt = logging.Formatter(
        "%(asctime)s.%(msecs)03d [%(levelname)s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    if to_console:
        ch = logging.StreamHandler()
        ch.setFormatter(fmt)
        root.addHandler(ch)

    try:
        Path(log_dir).mkdir(parents=True, exist_ok=True)
        fh = RotatingFileHandler(
            Path(log_dir) / "uav.log",
            maxBytes=max_size_mb * 1024 * 1024,
            backupCount=backup_count,
            encoding="utf-8",
        )
        fh.setFormatter(fmt)
        root.addHandler(fh)
    except OSError:
        root.warning("Log dosyasi acilamadi, sadece konsol kullanilacak")

    root.propagate = False
    _initialized = True


def get_logger(name: str) -> logging.Logger:
    """uav.<name> alt-logger'i dondur."""
    if not _initialized:
        setup_logging()
    return logging.getLogger(f"uav.{name}")
