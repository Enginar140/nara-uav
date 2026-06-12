"""Giris noktasi: python -m uav.main --config config/sitl.yaml"""
from __future__ import annotations

import argparse
import asyncio
import signal

# Hem `python -m uav.main` hem de dosyayi dogrudan calistirmayi (IDE "run")
# destekle. Dogrudan calistirildiginda paket bilinmez; src'yi sys.path'e ekle.
if __package__ in (None, ""):
    import pathlib
    import sys
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
    from uav.app.application import Application
else:
    from .app.application import Application


def main() -> None:
    parser = argparse.ArgumentParser(description="Nara v10 IHA otonomi (Python)")
    parser.add_argument("--config", "-c", default="config/default.yaml",
                        help="YAML konfigurasyon yolu")
    args = parser.parse_args()

    app = Application(args.config)

    async def runner():
        loop = asyncio.get_running_loop()
        stop_task = None

        def request_stop():
            nonlocal stop_task
            if stop_task is None:
                stop_task = asyncio.create_task(app.stop())

        try:
            loop.add_signal_handler(signal.SIGINT, request_stop)
            loop.add_signal_handler(signal.SIGTERM, request_stop)
        except NotImplementedError:
            pass  # Windows: sinyal handler kisitli

        try:
            await app.run()
        except KeyboardInterrupt:
            await app.stop()

    asyncio.run(runner())


if __name__ == "__main__":
    main()
