"""Bound the video backlog and preserve responsiveness during slow detection."""

from threading import Event
import time

import numpy as np

from handeye_calibration.preview import CornerPreview, LatestFrame


def test_renderer_receives_latest_frame_without_backlog():
    frames = LatestFrame()
    for index in range(1000):
        frames.put(index)
    assert frames.take() == 999
    assert frames.take() is None


def test_slow_detector_does_not_block_or_accumulate_jobs(monkeypatch):
    entered, release = Event(), Event()
    expected = np.ones((88, 1, 2), np.float32)
    calls = []

    def detect(image, pattern):
        calls.append(pattern)
        entered.set()
        assert release.wait(2)
        return expected

    monkeypatch.setattr('handeye_calibration.preview.detect_corners', detect)
    detector = CornerPreview((11, 8))
    image = np.zeros((240, 320, 3), np.uint8)
    try:
        assert detector.update(image, now=0.0) is None
        assert entered.wait(1)
        pending = detector.future
        start = time.monotonic()
        for index in range(1, 10):
            assert detector.update(image, now=index * 0.03) is None
        assert time.monotonic() - start < 0.2
        assert detector.future is pending
        assert len(calls) == 1
        release.set()
        pending.result(timeout=1)
        np.testing.assert_array_equal(detector.update(image, now=0.3), expected)
        detector.future.result(timeout=1)
        # The second result belongs to t=0.3, not the current frame at t=1.0.
        assert detector.update(image, now=1.0) is None
        assert detector.update(np.zeros((120, 160, 3), np.uint8), now=1.01) is None
    finally:
        release.set()
        detector.close()
