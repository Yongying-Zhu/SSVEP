"""BLE receiver that republishes the headband data as an LSL EEG stream."""

from __future__ import annotations

import asyncio
import binascii
import logging
from typing import Optional

from .ble_protocol import CHANNELS, parse_notification

try:
    from bleak import BleakClient, BleakScanner
    from pylsl import StreamInfo, StreamOutlet
except ImportError as exc:  # pragma: no cover - exercised only on unprepared hosts
    BleakClient = BleakScanner = StreamInfo = StreamOutlet = None
    _IMPORT_ERROR = exc
else:
    _IMPORT_ERROR = None


LOG = logging.getLogger(__name__)

SERVICE_UUID = "f0001680-0451-4000-b000-000000000000"
CONFIG_UUID = "f0001681-0451-4000-b000-000000000000"
EEG_NOTIFY_UUID = "f0001682-0451-4000-b000-000000000000"

SUPPORTED_RATES = {250: 0x00FA, 500: 0x01F4, 1000: 0x03E8}


class BleLslReceiver:
    """Connect to the headband and publish one LSL sample per EEG sample."""

    def __init__(
        self,
        device_name: str,
        sample_rate: int = 250,
        include_soft_label: bool = False,
        set_device_rate: bool = True,
        scan_timeout: float = 15.0,
    ) -> None:
        if _IMPORT_ERROR is not None:
            raise RuntimeError(
                "BLE/LSL dependencies are missing. Install requirements.txt first."
            ) from _IMPORT_ERROR
        if sample_rate not in SUPPORTED_RATES:
            raise ValueError(f"sample_rate must be one of {sorted(SUPPORTED_RATES)}")

        self.device_name = device_name
        self.sample_rate = sample_rate
        self.include_soft_label = include_soft_label
        self.set_device_rate = set_device_rate
        self.scan_timeout = scan_timeout
        self.soft_label = 0.0
        self.notification_count = 0
        self._last_frame_counter: Optional[int] = None

        channel_count = CHANNELS + int(include_soft_label)
        self.info = StreamInfo(
            "BCIPro",
            "EEG",
            channel_count,
            sample_rate,
            "float32",
            "bci-pro-ble-eeg",
        )
        channels = self.info.desc().append_child("channels")
        for index in range(CHANNELS):
            channels.append_child("channel").append_child_value(
                "label", f"EEG_{index + 1}"
            ).append_child_value("unit", "microvolts")
        if include_soft_label:
            channels.append_child("channel").append_child_value(
                "label", "SoftLabel"
            ).append_child_value("unit", "label")
        self.outlet = StreamOutlet(self.info)

    @staticmethod
    def _rate_hex(sample_rate: int) -> str:
        return f"{SUPPORTED_RATES[sample_rate]:04x}"

    async def configure_device(self, client: BleakClient) -> None:
        """Set data mode/sample rate using the protocol found in local code."""
        raw = await client.read_gatt_char(CONFIG_UUID)
        hex_data = bytes(raw).hex()
        # The local protocol is a 42-byte frame: CF ... G.
        if len(hex_data) < 84 or hex_data[:4] != "4346" or hex_data[82:84] != "47":
            LOG.warning("Device config frame did not match the expected CF...G format")
            return

        config = list(hex_data)
        config[4:8] = list("0000")  # data mode
        config[8:12] = list(self._rate_hex(self.sample_rate))
        payload = binascii.unhexlify("".join(config))
        try:
            await client.write_gatt_char(CONFIG_UUID, payload, response=True)
        except TypeError:
            # Older bleak versions do not accept response=.
            await client.write_gatt_char(CONFIG_UUID, payload)
        LOG.info("Configured device for %d Hz data mode", self.sample_rate)

    def _notification_handler(self, _sender: int, data: bytearray) -> None:
        try:
            packet = parse_notification(bytes(data))
        except ValueError as exc:
            LOG.warning("Dropped malformed EEG packet: %s", exc)
            return

        # Some firmware revisions leave byte 141 at zero instead of exposing a
        # packet counter. Do not report that reserved/fixed value as loss.
        counter_is_active = (
            packet.frame_counter is not None
            and (packet.frame_counter != 0 or self._last_frame_counter not in (None, 0))
        )
        if counter_is_active and self._last_frame_counter is not None:
            expected = (self._last_frame_counter + 1) & 0xFF
            if packet.frame_counter != expected:
                LOG.warning(
                    "BLE frame counter jump: previous=%d current=%d",
                    self._last_frame_counter,
                    packet.frame_counter,
                )
        self._last_frame_counter = packet.frame_counter

        for sample in packet.samples:
            values = sample.tolist()
            if self.include_soft_label:
                values.append(float(self.soft_label))
            self.outlet.push_sample(values)
        self.notification_count += 1
        if self.notification_count % 50 == 0:
            LOG.info("Received %d BLE packets", self.notification_count)

    async def run(self) -> None:
        while True:
            LOG.info("Scanning for BLE device named %s", self.device_name)
            device = await BleakScanner.find_device_by_name(
                self.device_name, timeout=self.scan_timeout
            )
            if device is None:
                LOG.error("BLE device %s was not found", self.device_name)
                await asyncio.sleep(3.0)
                continue

            try:
                LOG.info("Connecting to %s", self.device_name)
                async with BleakClient(device) as client:
                    LOG.info("BLE connected")
                    if self.set_device_rate:
                        try:
                            await self.configure_device(client)
                        except Exception:
                            LOG.exception("Could not configure the device; continuing")
                    await client.start_notify(EEG_NOTIFY_UUID, self._notification_handler)
                    LOG.info(
                        "Publishing LSL stream BCIPro (%d channels, %d Hz)",
                        self.info.channel_count(),
                        self.sample_rate,
                    )
                    while client.is_connected:
                        await asyncio.sleep(1.0)
            except asyncio.CancelledError:
                raise
            except Exception:
                LOG.exception("BLE connection failed; retrying")
            await asyncio.sleep(2.0)
