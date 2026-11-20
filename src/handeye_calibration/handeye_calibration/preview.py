"""Bounded preview transport and asynchronous, throttled corner detection."""

from collections import deque
from concurrent.futures import ThreadPoolExecutor
from threading import Lock
import time

import cv2


class LatestFrame:
    """A single slot: slow renderers replace old frames instead of queuing them."""

    def __init__(self):
        self._lock = Lock()
        self._value = None

    def put(self, value):
        with self._lock:
            self._value = value

    def take(self):
        with self._lock:
            value, self._value = self._value, None
        return value


def detect_corners(image, pattern):
    """Use a smaller image for advisory overlays; never for calibration poses."""
    scale = min(1.0, 480 / image.shape[1])
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    if scale < 1:
        gray = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    found, corners = cv2.findChessboardCorners(
        gray, pattern, cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
        | cv2.CALIB_CB_FAST_CHECK)
    return corners / scale if found else None


class CornerPreview:
    """One running detection, no pending queue; expire overlays on moving video."""

    def __init__(self, pattern, interval=0.2, max_age=0.4):
        self.pattern = pattern
        self.interval = interval
        self.max_age = max_age
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='handeye_corners')
        self.future = None
        self.next_detection = 0.0
        self.corners = None
        self.shape = None
        self.submitted_at = 0.0
        self._pending_shape = None
        self._pending_at = 0.0
        self.completed_at = deque(maxlen=10)
        self.error = ''

    def update(self, image, now=None):
        now = time.monotonic() if now is None else now
        if self.future is not None and self.future.done():
            try:
                self.corners = self.future.result()
                self.error = ''
            except Exception as exc:
                self.corners = None
                self.error = str(exc)
            self.shape = self._pending_shape
            self.submitted_at = self._pending_at
            self.completed_at.append(now)
            self.future = None
        # Remember which frame the cached overlay belongs to, not the new job.
        corners = (self.corners if self.shape == image.shape[:2]
                   and now - self.submitted_at <= self.max_age else None)
        if self.future is None and now >= self.next_detection:
            self._pending_shape = image.shape[:2]
            self._pending_at = now
            self.next_detection = now + self.interval
            self.future = self.executor.submit(detect_corners, image.copy(), self.pattern)
        return corners

    @property
    def fps(self):
        if len(self.completed_at) < 2:
            return 0.0
        return (len(self.completed_at) - 1) / (self.completed_at[-1] - self.completed_at[0])

    def close(self):
        self.executor.shutdown(wait=True, cancel_futures=True)
