# 快速入门 / Quick Start Guide

## 一、硬件连接

1. **Arduino** → Raspberry Pi (USB连接)
2. **USB摄像头** → Raspberry Pi
3. 确保Arduino已烧录 `Car_Volt_Feedback_24A_with_hand_gesture_and_robo_arm.ino`

## 二、安装

```bash
cd /Users/chenyanning/Desktop/vision
pip3 install -r requirements.txt
```

## 三、硬件测试

### 测试串口通信
```bash
python3 movement.py
```

### 测试视觉伺服
```bash
python3 vision_servo.py
# 按 r/y/b/g 切换目标颜色
```

### 测试方块检测
```bash
python3 color_detector.py
```

## 四、运行主程序

```bash
python3 main.py
```

或指定串口：
```bash
python3 main.py /dev/ttyACM0
```

## 五、工作区域设置

在地面铺设：
- **绿色垫子** - START区域（起点）
- **红色垫子** - 红色方块目标区域
- **黄色垫子** - 黄色方块目标区域
- **蓝色垫子** - 蓝色方块目标区域

在START区域放置彩色小方块。

## 六、控制按键

- **Q** - 退出程序
- **S** - 跳过当前状态（调试）

## 七、常见问题

### 找不到Arduino
```bash
ls /dev/tty*  # 查看可用端口
python3 main.py /dev/ttyACM0  # 尝试不同端口
```

### 颜色检测不准
编辑 `vision_servo.py` 和 `color_detector.py` 中的HSV范围

### 运动速度调整
```python
# 在 main.py __init__ 中
self.robot.set_speed(50)  # 改为 30 或 80
```

## 八、参数调整

### 对齐阈值（vision_servo.py）
```python
self.x_tolerance = 50  # 增大 = 更宽松对齐
self.approach_area_threshold = 50000  # 增大 = 需要更接近
```

### 运动时间（main.py）
```python
self.robot.forward()
time.sleep(0.2)  # 增大 = 移动更长
self.robot.stop()
```

---

**就这些！开始测试吧！**

