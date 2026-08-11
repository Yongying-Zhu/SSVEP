"""Pure data helpers for the LSL-to-ROS bridge."""

from __future__ import annotations

from typing import Optional

import numpy as np


def normalize_eeg_sample(
    raw_sample, expected_channels: int, accept_soft_label: bool
) -> Optional[tuple[np.ndarray, Optional[float]]]:
    """Validate one LSL sample and split EEG channels from an optional label."""
    values = np.asarray(raw_sample, dtype=np.float32).reshape(-1)
    if values.size < expected_channels:
        return None
    eeg_values = values[:expected_channels]
    soft_label: Optional[float] = None
    if accept_soft_label and values.size > expected_channels:
        soft_label = float(values[expected_channels])
    return eeg_values, soft_label
