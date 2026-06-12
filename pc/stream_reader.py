"""
Threaded MJPEG stream reader.
Surekli arka planda en taze frame'i tutar; UI bunu okur.

ONEMLI: cv2.VideoCapture(url) AGI uzerinden bir akisa baglanirken UI thread'inde
cagrilirsa kamera erisilemezse SANIYELERCE bloklar (FFmpeg connect timeout) ve
tum arayuz donar. Bu yuzden acilis + okuma TAMAMEN arka thread'de yapilir; start()
aninda doner, kamera yoksa UI bloklanmaz, thread arka planda yeniden baglanmayi
dener. Durum `status` ile UI'ya bildirilir.
"""

import threading
import time
import cv2


class StreamReader:
    def __init__(self, url, open_timeout_ms=4000, read_timeout_ms=4000,
                 retry_delay_s=2.0):
        self.url = url
        self.cap = None
        self.frame = None
        self.frame_count = 0
        self.running = False
        self.lock = threading.Lock()
        self.thread = None
        # idle | connecting | open | failed  — UI bunu okuyup kullaniciya gosterir
        self.status = "idle"
        self._open_timeout_ms = open_timeout_ms
        self._read_timeout_ms = read_timeout_ms
        self._retry_delay_s   = retry_delay_s

    def start(self) -> bool:
        """NON-BLOCKING: VideoCapture acilisi thread icinde yapilir → UI bloklanmaz.
        Her zaman True doner (gercek baglanti durumu `status`'tan okunur)."""
        if self.running:
            return True
        self.running = True
        self.status = "connecting"
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()
        return True

    def _open(self):
        """VideoCapture'i open/read timeout ile ac (kamera yokken thread sonsuz
        bloklanmasin). Eski OpenCV parametre formunu desteklemezse sade acar."""
        try:
            params = [
                int(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC), self._open_timeout_ms,
                int(cv2.CAP_PROP_READ_TIMEOUT_MSEC), self._read_timeout_ms,
            ]
            cap = cv2.VideoCapture(self.url, cv2.CAP_FFMPEG, params)
        except Exception:
            # Eski OpenCV: timeout parametre formu yok — yine thread'de, UI bloklanmaz
            cap = cv2.VideoCapture(self.url)
        try:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass
        return cap

    def _sleep_interruptible(self, seconds: float):
        """running=False olunca erken cikan bekleme (stop() hizli olsun)."""
        end = seconds
        step = 0.1
        while end > 0 and self.running:
            time.sleep(step)
            end -= step

    def _loop(self):
        while self.running:
            # (Yeniden) baglan
            if self.cap is None or not self.cap.isOpened():
                if self.cap is not None:
                    try:
                        self.cap.release()
                    except Exception:
                        pass
                    self.cap = None
                self.status = "connecting"
                cap = self._open()
                if not cap.isOpened():
                    try:
                        cap.release()
                    except Exception:
                        pass
                    self.status = "failed"
                    self._sleep_interruptible(self._retry_delay_s)
                    continue
                self.cap = cap
                self.status = "open"

            ok, frame = self.cap.read()
            if not ok:
                # Akis koptu — kapat, dongu basinda yeniden baglanir
                try:
                    self.cap.release()
                except Exception:
                    pass
                self.cap = None
                self.status = "failed"
                self._sleep_interruptible(0.3)
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
            try:
                self.cap.release()
            except Exception:
                pass
            self.cap = None
        self.frame = None
        self.status = "idle"
