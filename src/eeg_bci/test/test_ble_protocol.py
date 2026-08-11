import numpy as np

from eeg_bci.ble_protocol import parse_notification, parse_samples


def test_parse_samples_shape_and_sign():
    payload = bytes([0, 0, 1]) * 40
    samples = parse_samples(payload)
    assert samples.shape == (5, 8)
    assert samples.dtype == np.float32
    assert np.all(samples > 0)


def test_parse_notification_reads_frame_counter():
    packet = bytearray(142)
    packet[2:122] = bytes([0, 0, 1]) * 40
    packet[141] = 42
    parsed = parse_notification(bytes(packet))
    assert parsed.frame_counter == 42
    assert parsed.samples.shape == (5, 8)
