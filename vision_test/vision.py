import cv2
import numpy as np

# HSV 阈值
HSV_RANGES = {
    "red": [
        (np.array([0, 100, 80]),  np.array([10, 255, 255])),
        (np.array([160, 100, 80]), np.array([179, 255, 255]))
    ],
    "yellow": [
        (np.array([20, 100, 100]), np.array([35, 255, 255]))
    ],
    "blue": [
        (np.array([90, 80, 80]), np.array([130, 255, 255]))
    ]
}

MIN_AREA_BIG = 8000  # 你需要在现场微调

def detect_large_color_region(frame_bgr, color_name):
    """
    检测大面积颜色区域（地上的大纸）
    返回: found, cx, cy, bbox, area
    """
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    ranges = HSV_RANGES[color_name]

    mask = None
    for (lower, upper) in ranges:
        m = cv2.inRange(hsv, lower, upper)
        mask = m if mask is None else cv2.bitwise_or(mask, m)

    # 去噪
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return False, None, None, None, 0

    max_cnt = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(max_cnt)
    if area < MIN_AREA_BIG:
        return False, None, None, None, area

    x, y, w, h = cv2.boundingRect(max_cnt)
    cx = x + w//2
    cy = y + h//2

    return True, cx, cy, (x,y,w,h), area, mask
