#!/usr/bin/env python3
"""
Serial Communication Module for Arduino Control
Handles movement commands and gripper control via USB serial port
"""

import serial
import time
from typing import Optional


class RobotController:
    """Serial controller for Arduino-based robot car with gripper"""
    
    def __init__(self, port: str = '/dev/ttyUSB0', baudrate: int = 9600, timeout: float = 1.0):
        """
        Initialize serial connection to Arduino
        
        Args:
            port: Serial port path (usually /dev/ttyUSB0 or /dev/ttyACM0)
            baudrate: Communication speed
            timeout: Read timeout in seconds
        """
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.connect()
        
    def connect(self):
        """Establish serial connection"""
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=1.0)
            time.sleep(2)  # Wait for Arduino to reset
            print(f"Connected to Arduino on {self.port}")
        except Exception as e:
            print(f"Failed to connect to {self.port}: {e}")
            print("Trying alternative port /dev/ttyACM0...")
            try:
                self.serial = serial.Serial('/dev/ttyACM0', self.baudrate, timeout=1.0)
                time.sleep(2)
                self.port = '/dev/ttyACM0'
                print(f"Connected to Arduino on /dev/ttyACM0")
            except Exception as e2:
                print(f"Failed to connect: {e2}")
                raise
    
    def _send_command(self, cmd: str):
        """Send command to Arduino via serial"""
        if self.serial and self.serial.is_open:
            self.serial.write(f"{cmd}\n".encode())
            self.serial.flush()
            
    def forward(self, duration: float = 0):
        """Move forward (A command)"""
        self._send_command("A")
        if duration > 0:
            time.sleep(duration)
            self.stop()
    
    def backward(self, duration: float = 0):
        """Move backward (B command)"""
        self._send_command("B")
        if duration > 0:
            time.sleep(duration)
            self.stop()
    
    def left(self, duration: float = 0):
        """Move left (L command)"""
        self._send_command("L")
        if duration > 0:
            time.sleep(duration)
            self.stop()
    
    def right(self, duration: float = 0):
        """Move right (R command)"""
        self._send_command("R")
        if duration > 0:
            time.sleep(duration)
            self.stop()
    
    def rotate_clockwise(self, duration: float = 0):
        """Rotate clockwise (rC command)"""
        self._send_command("rC")
        if duration > 0:
            time.sleep(duration)
            self.stop()
    
    def rotate_counterclockwise(self, duration: float = 0):
        """Rotate counter-clockwise (rA command)"""
        self._send_command("rA")
        if duration > 0:
            time.sleep(duration)
            self.stop()
    
    def stop(self):
        """Stop all movement (S command)"""
        self._send_command("S")
    
    def set_speed(self, speed: int):
        """
        Set motor speed
        Args:
            speed: 30, 50, or 80
        """
        if speed in [30, 50, 80]:
            self._send_command(str(speed))
        else:
            print(f"Invalid speed {speed}, use 30, 50, or 80")
    
    def pick(self):
        """Execute pick sequence: approach, clip, rise"""
        print("Executing pick sequence...")
        self._send_command("go")
        time.sleep(4)  # Wait for sequence to complete
    
    def release(self):
        """Release gripper"""
        print("Releasing gripper...")
        self._send_command("rel")
        time.sleep(2)  # Wait for release
    
    def close(self):
        """Close serial connection"""
        if self.serial and self.serial.is_open:
            self.stop()
            self.serial.close()
            print("Serial connection closed")


# Test function
if __name__ == "__main__":
    print("=== Robot Controller Test ===")
    
    try:
        robot = RobotController()
        robot.set_speed(50)
        
        print("Testing movements (each 1 second)...")
        print("Forward...")
        robot.forward(1.0)
        
        print("Backward...")
        robot.backward(1.0)
        
        print("Left...")
        robot.left(1.0)
        
        print("Right...")
        robot.right(1.0)
        
        print("Rotate CW...")
        robot.rotate_clockwise(0.5)
        
        print("Rotate CCW...")
        robot.rotate_counterclockwise(0.5)
        
        print("Test complete!")
        
    except Exception as e:
        print(f"Error: {e}")
    finally:
        robot.close()

