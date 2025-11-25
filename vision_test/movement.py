import serial
import time

class CarController:
    def __init__(self, port="/dev/ttyACM0", baud=9600):
        self.ser = serial.Serial(port, baud, timeout=0.1)
        time.sleep(2.0)  # 给 Arduino 重启时间

    def send(self, cmd):
        line = (cmd + '\n').encode("utf-8")
        self.ser.write(line)
        time.sleep(0.01)

    def stop(self):
        self.send("S")

    def forward(self, dt=0.2):
        self.send("A")
        time.sleep(dt)
        self.stop()

    def back(self, dt=0.2):
        self.send("B")
        time.sleep(dt)
        self.stop()

    def left(self, dt=0.2):
        self.send("L")
        time.sleep(dt)
        self.stop()

    def right(self, dt=0.2):
        self.send("R")
        time.sleep(dt)
        self.stop()

    def rot_cw(self, dt=0.15):
        self.send("rC")
        time.sleep(dt)
        self.stop()

    def rot_ccw(self, dt=0.15):
        self.send("rA")
        time.sleep(dt)
        self.stop()

    def set_speed(self, level=50):
        # level ∈ {30, 50, 80}
        self.send(str(level))
