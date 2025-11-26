#!/usr/bin/env python3
"""
Visual Servoing Module for Region Alignment
Uses color block detection to align robot with target regions
"""

import cv2
import numpy as np
from typing import Tuple, Optional, Dict


class VisualServo:
    """Visual servoing controller for aligning with colored regions"""
    
    def __init__(self, frame_width: int = 640, frame_height: int = 480):
        """
        Initialize visual servo controller
        
        Args:
            frame_width: Camera frame width
            frame_height: Camera frame height
        """
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.center_x = frame_width // 2
        self.center_y = frame_height // 2
        
        # Alignment thresholds
        self.x_tolerance = 50  # pixels - horizontal alignment tolerance
        self.y_tolerance = 40  # pixels - vertical alignment tolerance
        self.min_area_threshold = 3000  # minimum area to consider block as target
        self.approach_area_threshold = 50000  # area threshold for "close enough"
        
        # Define HSV color ranges for regions
        self.color_ranges = {
            'red': [
                (np.array([0, 100, 100]), np.array([10, 255, 255])),
                (np.array([160, 100, 100]), np.array([180, 255, 255]))
            ],
            'yellow': [
                (np.array([20, 100, 100]), np.array([35, 255, 255]))
            ],
            'blue': [
                (np.array([100, 100, 100]), np.array([130, 255, 255]))
            ],
            'green': [  # For START area
                (np.array([40, 50, 50]), np.array([80, 255, 255]))
            ]
        }
        
        self.kernel = np.ones((5, 5), np.uint8)
    
    def detect_largest_block(self, frame: np.ndarray, color: str) -> Optional[Dict]:
        """
        Detect the largest colored block in frame
        
        Args:
            frame: BGR image from camera
            color: Color name ('red', 'yellow', 'blue', 'green')
            
        Returns:
            Dictionary with block info or None if not found
        """
        if color not in self.color_ranges:
            return None
        
        # Preprocess
        blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        
        # Create mask for target color
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower, upper in self.color_ranges[color]:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))
        
        # Morphological operations
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None
        
        # Get largest contour
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        
        if area < self.min_area_threshold:
            return None
        
        # Calculate center
        M = cv2.moments(largest_contour)
        if M["m00"] == 0:
            return None
        
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        
        # Get bounding box
        x, y, w, h = cv2.boundingRect(largest_contour)
        
        return {
            'center': (cx, cy),
            'area': area,
            'bbox': (x, y, w, h),
            'contour': largest_contour,
            'mask': mask
        }
    
    def calculate_alignment_error(self, block_info: Dict) -> Tuple[int, int]:
        """
        Calculate alignment error from frame center
        
        Args:
            block_info: Block detection result
            
        Returns:
            (x_error, y_error) in pixels
        """
        cx, cy = block_info['center']
        x_error = cx - self.center_x
        y_error = cy - self.center_y
        return x_error, y_error
    
    def get_movement_command(self, block_info: Dict) -> str:
        """
        Determine movement command based on block position
        
        Args:
            block_info: Block detection result
            
        Returns:
            Movement command: 'forward', 'left', 'right', 'rotate_cw', 'rotate_ccw', 'aligned', 'close'
        """
        if block_info is None:
            return 'search'  # Need to search for target
        
        x_error, y_error = self.calculate_alignment_error(block_info)
        area = block_info['area']
        
        # Check if close enough
        if area > self.approach_area_threshold:
            return 'close'
        
        # Check horizontal alignment
        if abs(x_error) > self.x_tolerance:
            # Need to turn or strafe
            if abs(x_error) > 150:  # Large error - rotate
                return 'rotate_cw' if x_error > 0 else 'rotate_ccw'
            else:  # Small error - strafe
                return 'right' if x_error > 0 else 'left'
        
        # If horizontally aligned, move forward
        return 'forward'
    
    def is_aligned(self, block_info: Dict) -> bool:
        """Check if robot is aligned with target"""
        if block_info is None:
            return False
        
        x_error, y_error = self.calculate_alignment_error(block_info)
        return abs(x_error) <= self.x_tolerance
    
    def is_close_enough(self, block_info: Dict) -> bool:
        """Check if robot is close enough to target"""
        if block_info is None:
            return False
        
        return block_info['area'] > self.approach_area_threshold
    
    def draw_debug_info(self, frame: np.ndarray, block_info: Optional[Dict], 
                        target_color: str) -> np.ndarray:
        """
        Draw debug visualization on frame
        
        Args:
            frame: Input frame
            block_info: Block detection result
            target_color: Target color name
            
        Returns:
            Annotated frame
        """
        annotated = frame.copy()
        
        # Draw center crosshair
        cv2.line(annotated, (self.center_x - 30, self.center_y), 
                (self.center_x + 30, self.center_y), (0, 255, 0), 2)
        cv2.line(annotated, (self.center_x, self.center_y - 30), 
                (self.center_x, self.center_y + 30), (0, 255, 0), 2)
        
        # Draw tolerance box
        cv2.rectangle(annotated, 
                     (self.center_x - self.x_tolerance, self.center_y - self.y_tolerance),
                     (self.center_x + self.x_tolerance, self.center_y + self.y_tolerance),
                     (0, 255, 255), 2)
        
        if block_info is not None:
            # Draw detected block
            x, y, w, h = block_info['bbox']
            cv2.rectangle(annotated, (x, y), (x + w, y + h), (0, 0, 255), 3)
            
            # Draw center
            cx, cy = block_info['center']
            cv2.circle(annotated, (cx, cy), 8, (255, 0, 0), -1)
            
            # Draw line from center to block
            cv2.line(annotated, (self.center_x, self.center_y), (cx, cy), (255, 0, 255), 2)
            
            # Display info
            x_error, y_error = self.calculate_alignment_error(block_info)
            area = block_info['area']
            
            info_text = [
                f"Target: {target_color.upper()}",
                f"X Error: {x_error:+d} px",
                f"Y Error: {y_error:+d} px",
                f"Area: {int(area)}",
                f"Status: {'ALIGNED' if self.is_aligned(block_info) else 'ADJUSTING'}",
                f"Distance: {'CLOSE' if self.is_close_enough(block_info) else 'FAR'}"
            ]
            
            y_offset = 30
            for text in info_text:
                cv2.putText(annotated, text, (10, y_offset),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                y_offset += 30
        else:
            cv2.putText(annotated, f"Searching for {target_color.upper()}...", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        return annotated


# Test function
if __name__ == "__main__":
    print("=== Visual Servo Test ===")
    
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    servo = VisualServo(640, 480)
    target_color = 'red'  # Change to test different colors
    
    print(f"Testing visual servo with target color: {target_color}")
    print("Press 'r' for red, 'y' for yellow, 'b' for blue, 'g' for green")
    print("Press 'q' to quit")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # Detect target
        block_info = servo.detect_largest_block(frame, target_color)
        
        # Get movement command
        if block_info:
            command = servo.get_movement_command(block_info)
            print(f"\rCommand: {command:15s}", end='', flush=True)
        
        # Draw debug info
        debug_frame = servo.draw_debug_info(frame, block_info, target_color)
        
        cv2.imshow('Visual Servo Test', debug_frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('r'):
            target_color = 'red'
        elif key == ord('y'):
            target_color = 'yellow'
        elif key == ord('b'):
            target_color = 'blue'
        elif key == ord('g'):
            target_color = 'green'
    
    cap.release()
    cv2.destroyAllWindows()

