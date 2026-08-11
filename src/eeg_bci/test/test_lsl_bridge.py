import numpy as np

from eeg_bci.lsl_bridge_utils import normalize_eeg_sample


def test_normalize_eeg_sample_keeps_channels_and_soft_label():
    result = normalize_eeg_sample(range(9), expected_channels=8, accept_soft_label=True)

    assert result is not None
    eeg_values, soft_label = result
    assert np.array_equal(eeg_values, np.arange(8, dtype=np.float32))
    assert soft_label == 8.0


def test_normalize_eeg_sample_rejects_short_sample():
    assert normalize_eeg_sample([1.0] * 7, 8, True) is None


def test_normalize_eeg_sample_ignores_extra_channels_without_label_mode():
    result = normalize_eeg_sample([1.0] * 10, expected_channels=8, accept_soft_label=False)

    assert result is not None
    eeg_values, soft_label = result
    assert eeg_values.shape == (8,)
    assert soft_label is None
