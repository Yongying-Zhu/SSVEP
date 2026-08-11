"""Small BLE discovery utility used before starting the receiver."""

from __future__ import annotations

import asyncio


async def _scan() -> None:
    try:
        from bleak import BleakScanner
    except ImportError as exc:
        raise SystemExit("Install bleak before scanning BLE devices") from exc

    try:
        devices = await BleakScanner.discover(timeout=8.0)
    except PermissionError as exc:
        raise SystemExit(
            "BlueZ/D-Bus access was denied. Check that the bluetooth service is running "
            "and that this terminal has access to the system D-Bus."
        ) from exc
    if not devices:
        print("No BLE devices found")
        return
    for device in devices:
        print(f"{device.name or '<unnamed>'}\t{device.address}")


def main() -> None:
    asyncio.run(_scan())


if __name__ == "__main__":
    main()
