from camera import USBCamera
from movement import CarController
from align_region import align_to_region

def main():
    cam = USBCamera(cam_id=0)
    car = CarController("/dev/ttyACM0", 9600)

    car.set_speed(50)

    # 示例：让小车找到红色区域并靠近
    print("Start aligning to RED region...")
    align_to_region(cam, car, "red")

    print("Done.")

if __name__ == "__main__":
    main()
