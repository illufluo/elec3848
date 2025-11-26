import cv2
from vision import detect_large_color_region

FRAME_W = 640
CX_TARGET = FRAME_W // 2

TOL_X = 40         # 水平方向允许的偏差（像素）
NEAR_HEIGHT = 220  # bounding box 高度达到这个值就认为“足够近”

def align_to_region(camera, car, target_color="red"):
    stable_count = 0

    while True:
        frame = camera.read()
        found, cx, cy, (x,y,w,h), area, mask = detect_large_color_region(frame, target_color)

        disp = frame.copy()

        if found:
            cv2.rectangle(disp, (x,y), (x+w, y+h), (0,255,0), 2)
            cv2.circle(disp, (cx,cy), 5, (0,0,255), -1)
            cv2.putText(disp, f"{target_color} h={h}", (x, y-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)

            dx = cx - CX_TARGET

            # 水平偏差
            if dx > TOL_X:
                car.right(0.13)
                stable_count = 0
            elif dx < -TOL_X:
                car.left(0.13)
                stable_count = 0
            else:
                # 距离判断
                if h < NEAR_HEIGHT:
                    car.forward(0.18)
                    stable_count = 0
                else:
                    stable_count += 1
                    car.stop()
                    if stable_count >= 5:
                        print(f"[ALIGN] aligned to {target_color}")
                        return  # 完成

        else:
            # 找不到目标区域，就旋转搜索
            car.rot_cw(0.12)
            stable_count = 0

        cv2.imshow("align", disp)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            car.stop()
            break
