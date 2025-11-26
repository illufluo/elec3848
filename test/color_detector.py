#!/usr/bin/env python3
"""
Color Block Detector for Small Blocks (pickup targets)
Simplified version using USB camera
"""

import cv2
import numpy as np
from typing import List, Optional, Dict


class SmallBlockDetector:
    """Detector for small colored blocks (pickup targets)"""
    
    def __init__(self):
        """Initialize detector"""
        # HSV color ranges for small blocks
        self.color_ranges = {
            'red': [
                (np.array([0, 120, 100]), np.array([10, 255, 255])),
                (np.array([160, 120, 100]), np.array([180, 255, 255]))
            ],
            'yellow': [
                (np.array([20, 120, 100]), np.array([35, 255, 255]))
            ],
            'blue': [
                (np.array([100, 120, 100]), np.array([130, 255, 255]))
            ]
        }
        
        # Small block detection parameters
        self.min_area = 500  # Minimum area for small blocks
        self.max_area = 8000  # Maximum area for small blocks
        self.kernel = np.ones((5, 5), np.uint8)
    
    def detect_blocks(self, frame: np.ndarray) -> List[Dict]:
        """
        Detect all small colored blocks in frame
        
        Args:
            frame: BGR image from camera
            
        Returns:
            List of detected blocks with color, center, area info
        """
        blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        
        all_blocks = []
        
        # Check each color
        for color_name, ranges in self.color_ranges.items():
            # Create mask
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lower, upper in ranges:
                mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))
            
            # Morphological operations
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                area = cv2.contourArea(contour)
                
                # Filter by area
                if self.min_area <= area <= self.max_area:
                    # Calculate center
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        
                        x, y, w, h = cv2.boundingRect(contour)
                        
                        all_blocks.append({
                            'color': color_name,
                            'center': (cx, cy),
                            'area': area,
                            'bbox': (x, y, w, h),
                            'contour': contour
                        })
        
        # Sort by area (largest first)
        all_blocks.sort(key=lambda b: b['area'], reverse=True)
        
        return all_blocks
    
    def find_closest_block(self, frame: np.ndarray, 
                          center_x: int, center_y: int) -> Optional[Dict]:
        """
        Find the closest block to specified position
        
        Args:
            frame: BGR image
            center_x, center_y: Target position
            
        Returns:
            Closest block info or None
        """
        blocks = self.detect_blocks(frame)
        
        if not blocks:
            return None
        
        # Find closest to center
        def distance(block):
            bx, by = block['center']
            return (bx - center_x) ** 2 + (by - center_y) ** 2
        
        return min(blocks, key=distance)
    
    def draw_blocks(self, frame: np.ndarray, blocks: List[Dict]) -> np.ndarray:
        """
        Draw detected blocks on frame
        
        Args:
            frame: Input frame
            blocks: List of detected blocks
            
        Returns:
            Annotated frame
        """
        annotated = frame.copy()
        
        color_map = {
            'red': (0, 0, 255),
            'yellow': (0, 255, 255),
            'blue': (255, 0, 0)
        }
        
        for i, block in enumerate(blocks):
            color = color_map.get(block['color'], (255, 255, 255))
            
            # Draw bounding box
            x, y, w, h = block['bbox']
            cv2.rectangle(annotated, (x, y), (x + w, y + h), color, 2)
            
            # Draw center
            cx, cy = block['center']
            cv2.circle(annotated, (cx, cy), 5, color, -1)
            
            # Label
            label = f"{block['color'].upper()}#{i+1}"
            cv2.putText(annotated, label, (x, y - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        return annotated


# Test function
if __name__ == "__main__":
    print("=== Small Block Detector Test ===")
    
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    detector = SmallBlockDetector()
    
    print("Detecting small colored blocks...")
    print("Press 'q' to quit")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # Detect blocks
        blocks = detector.detect_blocks(frame)
        
        # Draw results
        result = detector.draw_blocks(frame, blocks)
        
        # Display count
        cv2.putText(result, f"Blocks found: {len(blocks)}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Print to console
        if blocks:
            print(f"\rDetected {len(blocks)} blocks: ", end='')
            for b in blocks[:3]:  # Show first 3
                print(f"{b['color']}({int(b['area'])}) ", end='')
            print("   ", end='', flush=True)
        
        cv2.imshow('Block Detection', result)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()

