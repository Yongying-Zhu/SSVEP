"""Console entry point for the BLE-to-LSL receiver."""

from __future__ import annotations

import argparse
import asyncio
import logging

from .ble_receiver import BleLslReceiver


def main() -> None:
    parser = argparse.ArgumentParser(description="Publish the EEG headband as an LSL stream")
    parser.add_argument("--device-name", default="VIS_BCI_DFED857C")
    parser.add_argument("--sample-rate", type=int, default=250, choices=[250, 500, 1000])
    parser.add_argument("--include-soft-label", action="store_true")
    parser.add_argument("--no-set-device-rate", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    receiver = BleLslReceiver(
        device_name=args.device_name,
        sample_rate=args.sample_rate,
        include_soft_label=args.include_soft_label,
        set_device_rate=not args.no_set_device_rate,
    )
    try:
        asyncio.run(receiver.run())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
