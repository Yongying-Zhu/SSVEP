"""Packet parsing for the 8-channel BLE EEG headband.

The local reference implementation uses bytes 2..121 as five consecutive
samples, with eight signed 24-bit big-endian values per sample.  The remaining
packet bytes contain device/protocol metadata and are not EEG samples.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Sequence

import numpy as np


CHANNELS = 8
SAMPLES_PER_PACKET = 5
BYTES_PER_VALUE = 3
EEG_PAYLOAD_OFFSET = 2
EEG_PAYLOAD_BYTES = SAMPLES_PER_PACKET * CHANNELS * BYTES_PER_VALUE
DEFAULT_SCALE_UV = 0.02235


@dataclass(frozen=True)
class ParsedPacket:
    samples: np.ndarray
    frame_counter: int | None


def _signed_24_be(raw: Sequence[int]) -> int:
    if len(raw) != 3:
        raise ValueError("a signed 24-bit value must contain exactly 3 bytes")
    value = (int(raw[0]) << 16) | (int(raw[1]) << 8) | int(raw[2])
    if value & 0x800000:
        value -= 1 << 24
    return value


def parse_samples(payload: bytes, scale_uv: float = DEFAULT_SCALE_UV) -> np.ndarray:
    """Parse the 120-byte EEG payload into ``(5, 8)`` float32 samples."""
    if len(payload) < EEG_PAYLOAD_BYTES:
        raise ValueError(
            f"EEG payload is {len(payload)} bytes; expected at least {EEG_PAYLOAD_BYTES}"
        )

    values: List[float] = []
    for offset in range(0, EEG_PAYLOAD_BYTES, BYTES_PER_VALUE):
        values.append(_signed_24_be(payload[offset : offset + BYTES_PER_VALUE]) * scale_uv)
    return np.asarray(values, dtype=np.float32).reshape(SAMPLES_PER_PACKET, CHANNELS)


def parse_notification(data: bytes, scale_uv: float = DEFAULT_SCALE_UV) -> ParsedPacket:
    """Parse a complete BLE notification using the observed packet layout."""
    if len(data) < EEG_PAYLOAD_OFFSET + EEG_PAYLOAD_BYTES:
        raise ValueError(f"BLE notification is too short: {len(data)} bytes")

    frame_counter = int(data[141]) if len(data) > 141 else None
    payload = data[EEG_PAYLOAD_OFFSET : EEG_PAYLOAD_OFFSET + EEG_PAYLOAD_BYTES]
    return ParsedPacket(parse_samples(payload, scale_uv=scale_uv), frame_counter)
