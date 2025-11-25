import cv2

class USBCamera:
    def __init__(self, cam_id=0, width=640, height=480):
        self.cap = cv2.VideoCapture(cam_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    def read(self):
        ret, frame = self.cap.read()
        if not ret:
            raise RuntimeError("Failed to read frame from USB camera.")
        return frame  # BGR format

    def release(self):
        self.cap.release()
