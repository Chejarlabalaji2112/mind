import cv2 
from mind.ports.percieve_port import Vision

class CameraSource(Vision()):
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
    
    def see(self):
        return self.get_frame()
        
    def release(self):
        self.cap.release()
