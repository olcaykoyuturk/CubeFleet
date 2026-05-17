"""
Threaded MJPEG stream reader.
Surekli arka planda en taze frame'i tutar; UI bunu okur.
"""

import threading
import time
import cv2


class StreamReader:
    def __init__(self, url):
        self.url = url
        self.cap = None
        self.frame = None
        self.frame_count = 0
        self.running = False
        self.lock = threading.Lock()
        self.thread = None

    def start(self) -> bool:
        self.cap = cv2.VideoCapture(self.url)
        if not self.cap.isOpened():
            return False
        try:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()
        return True

    def _loop(self):
        while self.running:
            ok, frame = self.cap.read()
            if not ok:
                time.sleep(0.05)
                continue
            with self.lock:
                self.frame = frame
                self.frame_count += 1

    def read(self):
        """En taze frame ve toplam frame sayisi (frame, count). frame=None ise henuz gelmedi."""
        with self.lock:
            if self.frame is None:
                return None, 0
            return self.frame.copy(), self.frame_count

    def stop(self):
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=1.0)
        if self.cap is not None:
            self.cap.release()
        self.frame = None
