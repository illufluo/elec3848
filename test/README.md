# 树莓派颜色方块搬运机器人 / Raspberry Pi Color Block Transport Robot

<div align="center">

![Python](https://img.shields.io/badge/Python-3.9+-blue.svg)
![OpenCV](https://img.shields.io/badge/OpenCV-4.8+-green.svg)
![Raspberry Pi](https://img.shields.io/badge/Raspberry%20Pi-5-red.svg)

自主导航的颜色方块搬运机器人，集成视觉识别、路径规划和机械臂控制。

*Autonomous color block transport robot with integrated vision, path planning and robotic arm control.*

</div>

---

## 📋 目录 / Table of Contents

- [项目简介 / Project Overview](#项目简介--project-overview)
- [硬件要求 / Hardware Requirements](#硬件要求--hardware-requirements)
- [快速开始 / Quick Start](#快速开始--quick-start)
- [使用方法 / Usage](#使用方法--usage)
- [代码结构 / Code Structure](#代码结构--code-structure)
- [参数调整 / Parameter Tuning](#参数调整--parameter-tuning)
- [常见问题 / Troubleshooting](#常见问题--troubleshooting)

---

## 项目简介 / Project Overview

完整的自主颜色方块搬运机器人系统。机器人能够：
1. 在START区域识别并抓取彩色方块
2. 根据方块颜色自动导航到对应目标区域
3. 使用视觉伺服精准对齐
4. 放下方块后自动返回START区域
5. 循环执行直到所有方块搬运完毕

*Complete autonomous color block transport robot. The robot can identify and pick colored blocks from START area, navigate to corresponding target regions based on block color, precisely align using visual servoing, drop blocks and return to START automatically.*

### 核心功能 / Core Features

✅ **自主导航** - 基于视觉的自动导航和对齐  
✅ **颜色识别** - 识别红、黄、蓝色方块和区域  
✅ **状态机控制** - 完整的任务流程管理  
✅ **视觉伺服** - 精准的目标区域对齐  
✅ **机械臂控制** - 自动抓取和释放方块

---

## 硬件要求 / Hardware Requirements

### 必需硬件 / Required Hardware

| 组件 | 规格 | 说明 |
|------|------|------|
| 🍓 **Raspberry Pi** | Raspberry Pi 5 | 视觉处理和决策控制 |
| 🤖 **Arduino** | Arduino Mega 2560 | 电机和机械臂底层控制 |
| 📷 **Camera** | USB Camera | USB摄像头 (640x480) |
| 🚗 **Chassis** | 4-wheel Mecanum | 麦克纳姆轮小车底盘 |
| 🦾 **Arm** | 4-DOF Robotic Arm | 4自由度机械臂带夹爪 |
| 💾 **Storage** | 16GB+ microSD | 系统存储 |
| 🔌 **Power** | 12V Battery | 机器人电源 |

---

## 快速开始 / Quick Start

### 1. 硬件连接

- Arduino通过USB连接到树莓派（通常是 `/dev/ttyUSB0` 或 `/dev/ttyACM0`）
- USB摄像头连接到树莓派
- 确保Arduino已烧录 `Car_Volt_Feedback_24A_with_hand_gesture_and_robo_arm.ino`

### 2. 安装依赖

```bash
cd /Users/chenyanning/Desktop/vision
pip3 install -r requirements.txt
```

### 3. 测试硬件

```bash
# Test serial connection
python3 movement.py

# Test camera and visual servoing
python3 vision_servo.py

# Test block detection
python3 color_detector.py
```

### 4. 运行主程序

```bash
python3 main.py
```

或指定串口和摄像头：

```bash
python3 main.py /dev/ttyACM0 0
```

---

## 使用方法 / Usage

### 主程序运行

```bash
python3 main.py [serial_port] [camera_id]
```

参数说明：
- `serial_port`: Arduino串口路径（默认 `/dev/ttyUSB0`）
- `camera_id`: 摄像头设备ID（默认 `0`）

### 操作流程

1. **准备工作区**：
   - 在地面铺设START区域（绿色垫子）
   - 放置目标区域垫子（红色、黄色、蓝色）
   - 在START区域放置彩色方块

2. **启动机器人**：
   ```bash
   python3 main.py
   ```

3. **自动运行**：
   - 机器人自动对齐START区域
   - 识别并抓取方块
   - 导航到对应颜色的目标区域
   - 放下方块
   - 返回START区域
   - 重复循环

### 控制按键

| 按键 | 功能 |
|------|------|
| **Q** | 退出程序 |
| **S** | 跳过当前状态（调试用） |

### 状态说明

机器人运行时会经历以下状态：
- `INIT` - 初始化
- `START_ALIGN` - 对齐START区域
- `SEARCH_BLOCK` - 搜索方块
- `PICK` - 抓取方块
- `GOTO_REGION` - 粗略导航到目标区域
- `ALIGN_REGION` - 精准对齐目标区域
- `DROP` - 放下方块
- `RETURN_START` - 返回START区域
- `COMPLETE` - 任务完成

---

## 参数调整 / Parameter Tuning

### 颜色范围调整

如果颜色检测不准确，修改以下文件中的HSV范围：

**vision_servo.py** - 大区域检测（目标区域和START区域）：
```python
self.color_ranges = {
    'red': [...],
    'yellow': [...],
    'blue': [...],
    'green': [...]  # START area
}
```

**color_detector.py** - 小方块检测：
```python
self.color_ranges = {
    'red': [...],
    'yellow': [...],
    'blue': [...]
}
```

### 视觉伺服参数

在 `vision_servo.py` 中调整对齐阈值：

```python
self.x_tolerance = 50  # 水平对齐容差（像素）
self.y_tolerance = 40  # 垂直对齐容差
self.min_area_threshold = 3000  # 最小区域面积
self.approach_area_threshold = 50000  # "足够近"的面积阈值
```

### 运动参数

在 `main.py` 中调整运动时长：

```python
self.robot.forward()
time.sleep(0.2)  # 前进持续时间
self.robot.stop()
```

在 `movement.py` 中调整速度：

```python
self.robot.set_speed(50)  # 30, 50, or 80
```

---

## 代码结构 / Code Structure

```
vision/
│
├── main.py                     # 主程序：状态机控制
│   └── ColorBlockRobot         # 机器人主控制类
│       ├── state_init()
│       ├── state_start_align()
│       ├── state_search_block()
│       ├── state_pick()
│       ├── state_goto_region()
│       ├── state_align_region()
│       ├── state_drop()
│       └── state_return_start()
│
├── movement.py                 # 串口通信：Arduino控制
│   └── RobotController         # 运动控制类
│       ├── forward(), backward()
│       ├── left(), right()
│       ├── rotate_clockwise(), rotate_counterclockwise()
│       ├── pick(), release()
│       └── stop()
│
├── vision_servo.py             # 视觉伺服：区域对齐
│   └── VisualServo             # 视觉伺服类
│       ├── detect_largest_block()
│       ├── calculate_alignment_error()
│       ├── get_movement_command()
│       └── is_close_enough()
│
├── color_detector.py           # 颜色检测：小方块识别
│   └── SmallBlockDetector      # 方块检测类
│       ├── detect_blocks()
│       └── find_closest_block()
│
├── requirements.txt            # Python依赖
└── README.md                   # 文档
```

### 模块说明 / Module Description

- **main.py**: 状态机，协调整个任务流程
- **movement.py**: 封装Arduino串口通信协议
- **vision_servo.py**: 视觉伺服算法，实现区域对齐
- **color_detector.py**: 小方块检测和识别

---

## 添加新区域 / Adding New Regions

要添加新的颜色区域（如紫色），按以下步骤操作：

### 1. 更新vision_servo.py

添加新颜色的HSV范围：
```python
self.color_ranges = {
    # ... existing colors ...
    'purple': [
        (np.array([130, 100, 100]), np.array([160, 255, 255]))
    ]
}
```

### 2. 更新main.py

在颜色映射中添加：
```python
self.color_map = {
    'red': 'red',
    'yellow': 'yellow',
    'blue': 'blue',
    'purple': 'purple'  # New mapping
}
```

### 3. 更新color_detector.py（如果小方块也有新颜色）

```python
self.color_ranges = {
    # ... existing colors ...
    'purple': [
        (np.array([130, 120, 100]), np.array([160, 255, 255]))
    ]
}
```

就这么简单！

---

## 常见问题 / Troubleshooting

### ❌ 问题 1: 无法连接Arduino

**错误**: `Failed to connect to /dev/ttyUSB0`

**解决方案**:
```bash
# 查找Arduino端口
ls /dev/tty*

# 尝试不同端口
python3 main.py /dev/ttyACM0

# 添加用户到dialout组（需要重启）
sudo usermod -a -G dialout $USER
```

### ❌ 问题 2: 摄像头无法打开

**解决方案**:
```bash
# 测试摄像头
ls /dev/video*

# 尝试不同ID
python3 main.py /dev/ttyUSB0 1

# 检查摄像头权限
sudo chmod 666 /dev/video0
```

### ❌ 问题 3: 颜色检测不准确

**解决方案**:
1. 改善光照条件（均匀照明）
2. 调整 `vision_servo.py` 和 `color_detector.py` 中的HSV范围
3. 使用 `calibration_tool.py` 找到最佳HSV值

### ❌ 问题 4: 机器人找不到目标

**可能原因**:
- 区域垫子不在视野内
- 颜色范围不匹配
- 面积阈值设置不当

**解决方案**:
- 确保START和目标区域在摄像头可见范围内
- 调整 `min_area_threshold` 参数
- 检查区域颜色是否与代码中定义的匹配

### ❌ 问题 5: 机器人状态超时

**解决方案**:
```python
# 在 main.py 中增加超时时间
self.timeout = 60.0  # 增加到60秒
```

---

## 串口通信协议 / Serial Protocol

Arduino接收的命令格式（每条命令以换行符结束）：

| 命令 | 功能 | Arduino函数 |
|------|------|------------|
| `A\n` | 前进 | ADVANCE() |
| `B\n` | 后退 | BACK() |
| `L\n` | 左移 | LEFT_2() |
| `R\n` | 右移 | RIGHT_2() |
| `rC\n` | 顺时针旋转 | rotate_1() |
| `rA\n` | 逆时针旋转 | rotate_2() |
| `S\n` | 停止 | STOP() |
| `30\n` | 设置速度30 | Motor_PWM=30 |
| `50\n` | 设置速度50 | Motor_PWM=50 |
| `80\n` | 设置速度80 | Motor_PWM=80 |
| `go\n` | 抓取序列 | approach, clip, rise |
| `rel\n` | 释放夹爪 | release() |

---

<div align="center">

**🤖 Autonomous Robot System Ready! 自主机器人系统就绪！**

</div>

