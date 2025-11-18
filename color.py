#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
在 Raspberry Pi 5 上使用 PiCam + OpenCV 识别红、黄、蓝色方块，
在画面中标出方块轮廓和中心点，并在终端打印坐标信息。

依赖:
  sudo apt install python3-opencv python3-picamera2
"""

import time
import cv2
import numpy as np
from picamera2 import Picamera2

# ========== 颜色 HSV 阈值（初始版本，后续可调）==========
# 红色需要两段
LOWER_RED1 = np.array([0, 100, 80])
UPPER_RED1 = np.array([10, 255, 255])
LOWER_RED2 = np.array([160, 100, 80])
UPPER_RED2 = np.array([179, 255, 255])

LOWER_YELLOW = np.array([20, 100, 100])
UPPER_YELLOW = np.array([35, 255, 255])

LOWER_BLUE = np.array([90, 80, 80])
UPPER_BLUE = np.array([130, 255, 255])

# 轮廓面积阈值，避免把噪点当成方块
MIN_AREA = 500  # 根据实际画面可适当调大/调小

# ========== 初始化 PiCamera2 ==========
picam2 = Picamera2()
config = picam2.create_preview_configuration(
    main={
        "size": (640, 480),   # 分辨率，够用即可
        "format": "RGB888"
    }
)
picam2.configure(config)
picam2.start()
time.sleep(0.5)  # 给相机一点时间稳定曝光

print("Camera started. Press 'q' in the window to quit.")

def find_largest_contour(mask):
    """在二值掩膜中找到面积最大的轮廓及其外接矩形中心点"""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, None, None

    max_cnt = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(max_cnt)
    if area < MIN_AREA:
        return None, None, None

    x, y, w, h = cv2.boundingRect(max_cnt)
    cx = x + w // 2
    cy = y + h // 2
    return max_cnt, (x, y, w, h), (cx, cy)

while True:
    # ========== 1. 采集一帧图像 ==========
    frame = picam2.capture_array()       # RGB
    frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

    # ========== 2. 构建颜色掩膜 ==========
    # 红色：两个区间 + 起来
    mask_red1 = cv2.inRange(hsv, LOWER_RED1, UPPER_RED1)
    mask_red2 = cv2.inRange(hsv, LOWER_RED2, UPPER_RED2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)

    mask_yellow = cv2.inRange(hsv, LOWER_YELLOW, UPPER_YELLOW)
    mask_blue = cv2.inRange(hsv, LOWER_BLUE, UPPER_BLUE)

    # ========== 3. 去噪：形态学操作 ==========
    kernel = np.ones((5, 5), np.uint8)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
    mask_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_OPEN, kernel)
    mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel)

    # ========== 4. 查找并绘制每种颜色中最大的方块 ==========
    detections = []

    # 红色
    cnt_r, rect_r, center_r = find_largest_contour(mask_red)
    if cnt_r is not None:
        x, y, w, h = rect_r
        cx, cy = center_r
        cv2.rectangle(frame_bgr, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cv2.circle(frame_bgr, (cx, cy), 5, (0, 0, 255), -1)
        cv2.putText(frame_bgr, f"RED ({cx},{cy})", (x, y - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        detections.append(("red", cx, cy))

    # 黄色
    cnt_y, rect_y, center_y = find_largest_contour(mask_yellow)
    if cnt_y is not None:
        x, y, w, h = rect_y
        cx, cy = center_y
        cv2.rectangle(frame_bgr, (x, y), (x + w, y + h), (0, 255, 255), 2)
        cv2.circle(frame_bgr, (cx, cy), 5, (0, 255, 255), -1)
        cv2.putText(frame_bgr, f"YEL ({cx},{cy})", (x, y - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        detections.append(("yellow", cx, cy))

    # 蓝色
    cnt_b, rect_b, center_b = find_largest_contour(mask_blue)
    if cnt_b is not None:
        x, y, w, h = rect_b
        cx, cy = center_b
        cv2.rectangle(frame_bgr, (x, y), (x + w, y + h), (255, 0, 0), 2)
        cv2.circle(frame_bgr, (cx, cy), 5, (255, 0, 0), -1)
        cv2.putText(frame_bgr, f"BLU ({cx},{cy})", (x, y - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        detections.append(("blue", cx, cy))

    # 在终端打印当前检测结果
    if detections:
        print("Detected blocks:")
        for color, cx, cy in detections:
            print(f"  {color:6s} at (u={cx}, v={cy})")
    else:
        print("No blocks detected.")

    # ========== 5. 显示结果画面 ==========
    cv2.imshow("Color Blocks", frame_bgr)

    # 如果需要调试掩膜，也可以顺带显示其中一个:
    # cv2.imshow("Red Mask", mask_red)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

# ========== 清理资源 ==========
picam2.stop()
cv2.destroyAllWindows()
