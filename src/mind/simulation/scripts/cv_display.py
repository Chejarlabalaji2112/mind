# cv_display.py (New modular fallback for non-MuJoCo video display)
import cv2
import threading
import time
#TODO:may be we need to put this outside of simulation
class CvDisplay:
    """
    Fallback video display using OpenCV window.
    Thread-safe for put_frame calls.
    """
    def __init__(self, window_name="Video Playback"):
        self.window_name = window_name
        self._pending_frame = None
        self._lock = threading.Lock()
        self.running = True

    def put_frame(self, frame_bgr):
        """Put frame into buffer for display."""
        with self._lock:
            self._pending_frame = frame_bgr.copy()  # Avoid ref issues

    def put_text(self, text, bg_color=(0, 0, 0), text_color=(255, 255, 255), scale=1.0):
        """Overlay text on a blank frame (or current pending)."""
        h, w = 480, 640  # Default size if no frame
        canvas = cv2.imread("black.png") if hasattr(self, 'canvas') else np.zeros((h, w, 3), dtype=np.uint8)
        canvas[:] = bg_color
        font = cv2.FONT_HERSHEY_SIMPLEX
        text_size = cv2.getTextSize(text, font, scale, 2)[0]
        text_x = (w - text_size[0]) // 2
        text_y = (h + text_size[1]) // 2
        cv2.putText(canvas, text, (text_x, text_y), font, scale, text_color, 2)
        self.put_frame(canvas)

    def update(self):
        """Call in main loop to display pending frame."""
        with self._lock:
            if self._pending_frame is None:
                return
            frame_to_show = self._pending_frame.copy()
            self._pending_frame = None

        cv2.imshow(self.window_name, frame_to_show)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.running = False

    def close(self):
        """Cleanup window."""
        cv2.destroyAllWindows()
        self.running = False