import cv2 

class CameraSource:
    """
    Standalone camera handler.
    """
    def __init__(self, device_index=0):
        self.cap = cv2.VideoCapture(device_index)
        
    def get_frame(self):
        if self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                return frame
        return None
        
    def release(self):
        self.cap.release()
