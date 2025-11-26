#!/usr/bin/env python3
"""
Main State Machine for Autonomous Color Block Transport Robot
Handles full workflow: pickup from START, transport to target region, return
"""

import cv2
import time
from enum import Enum
from typing import Optional, Dict

from movement import RobotController
from vision_servo import VisualServo
from color_detector import SmallBlockDetector


class State(Enum):
    """Robot states"""
    INIT = "INIT"
    START_ALIGN = "START_ALIGN"
    SEARCH_BLOCK = "SEARCH_BLOCK"
    PICK = "PICK"
    GOTO_REGION = "GOTO_REGION"
    ALIGN_REGION = "ALIGN_REGION"
    DROP = "DROP"
    RETURN_START = "RETURN_START"
    COMPLETE = "COMPLETE"
    ERROR = "ERROR"


class ColorBlockRobot:
    """Main robot controller with state machine"""
    
    def __init__(self, serial_port: str = '/dev/ttyUSB0', camera_id: int = 0):
        """
        Initialize robot system
        
        Args:
            serial_port: Arduino serial port
            camera_id: USB camera device ID
        """
        print("=== Color Block Transport Robot ===")
        print("Initializing systems...")
        
        # Initialize hardware
        self.robot = RobotController(port=serial_port)
        self.robot.set_speed(50)  # Set moderate speed
        
        # Initialize camera
        self.camera = cv2.VideoCapture(camera_id)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        time.sleep(1)
        
        # Initialize vision modules
        self.visual_servo = VisualServo(640, 480)
        self.block_detector = SmallBlockDetector()
        
        # State machine
        self.state = State.INIT
        self.previous_state = None
        
        # Mission data
        self.current_block_color = None  # Color of block being transported
        self.target_region_color = None  # Target region color
        self.blocks_transported = 0
        
        # Color mapping: block color -> region color (can be customized)
        self.color_map = {
            'red': 'red',
            'yellow': 'yellow',
            'blue': 'blue'
        }
        
        # Timing
        self.state_start_time = time.time()
        self.timeout = 30.0  # State timeout in seconds
        
        # Debug display
        self.show_debug = True
        
        print("Initialization complete!")
        print(f"Blocks transported: {self.blocks_transported}")
    
    def get_frame(self) -> Optional[cv2.Mat]:
        """Capture frame from camera"""
        ret, frame = self.camera.read()
        return frame if ret else None
    
    def change_state(self, new_state: State):
        """Change to new state"""
        self.previous_state = self.state
        self.state = new_state
        self.state_start_time = time.time()
        print(f"\n>>> State: {self.previous_state.value} -> {new_state.value}")
    
    def check_timeout(self) -> bool:
        """Check if current state has timed out"""
        elapsed = time.time() - self.state_start_time
        if elapsed > self.timeout:
            print(f"!!! State timeout after {elapsed:.1f}s")
            return True
        return False
    
    def state_init(self):
        """Initial state - prepare for operation"""
        print("Robot ready. Starting mission...")
        self.robot.stop()
        time.sleep(0.5)
        self.change_state(State.START_ALIGN)
    
    def state_start_align(self):
        """Align with START region"""
        frame = self.get_frame()
        if frame is None:
            return
        
        # Detect START region (green)
        start_region = self.visual_servo.detect_largest_block(frame, 'green')
        
        if start_region is None:
            # Can't see START - search by rotating
            print("Searching for START region...")
            self.robot.rotate_clockwise()
            time.sleep(0.1)
            self.robot.stop()
            
            if self.check_timeout():
                print("Cannot find START region!")
                self.change_state(State.ERROR)
            return
        
        # Get movement command
        command = self.visual_servo.get_movement_command(start_region)
        
        # Execute command
        if command == 'close':
            print("Aligned with START region!")
            self.robot.stop()
            self.change_state(State.SEARCH_BLOCK)
        elif command == 'forward':
            self.robot.forward()
            time.sleep(0.2)
            self.robot.stop()
        elif command == 'left':
            self.robot.left()
            time.sleep(0.15)
            self.robot.stop()
        elif command == 'right':
            self.robot.right()
            time.sleep(0.15)
            self.robot.stop()
        elif command == 'rotate_cw':
            self.robot.rotate_clockwise()
            time.sleep(0.1)
            self.robot.stop()
        elif command == 'rotate_ccw':
            self.robot.rotate_counterclockwise()
            time.sleep(0.1)
            self.robot.stop()
        
        # Debug display
        if self.show_debug:
            debug_frame = self.visual_servo.draw_debug_info(frame, start_region, 'green')
            cv2.imshow('Robot Vision', debug_frame)
            cv2.waitKey(1)
    
    def state_search_block(self):
        """Search for small colored block to pick up"""
        frame = self.get_frame()
        if frame is None:
            return
        
        # Detect small blocks
        blocks = self.block_detector.detect_blocks(frame)
        
        if not blocks:
            print("No blocks found. Searching...")
            # Try small rotation to search
            self.robot.rotate_clockwise()
            time.sleep(0.15)
            self.robot.stop()
            
            if self.check_timeout():
                print("No blocks found in START area. Completing mission.")
                self.change_state(State.COMPLETE)
            
            if self.show_debug:
                cv2.imshow('Robot Vision', frame)
                cv2.waitKey(1)
            return
        
        # Found blocks - select the first one (largest)
        target_block = blocks[0]
        self.current_block_color = target_block['color']
        self.target_region_color = self.color_map[self.current_block_color]
        
        print(f"Found {self.current_block_color.upper()} block!")
        print(f"Target region: {self.target_region_color.upper()}")
        
        # Check if block is centered
        cx, cy = target_block['center']
        frame_center_x = frame.shape[1] // 2
        
        if abs(cx - frame_center_x) > 60:
            # Need to align with block
            if cx > frame_center_x:
                self.robot.right()
            else:
                self.robot.left()
            time.sleep(0.1)
            self.robot.stop()
        else:
            # Block is centered - pick it up
            self.change_state(State.PICK)
        
        # Debug display
        if self.show_debug:
            debug_frame = self.block_detector.draw_blocks(frame, blocks)
            cv2.putText(debug_frame, f"Target: {self.current_block_color.upper()}", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow('Robot Vision', debug_frame)
            cv2.waitKey(1)
    
    def state_pick(self):
        """Execute pick sequence"""
        print(f"Picking up {self.current_block_color.upper()} block...")
        self.robot.pick()
        print("Block picked!")
        self.change_state(State.GOTO_REGION)
    
    def state_goto_region(self):
        """Rough navigation towards target region"""
        frame = self.get_frame()
        if frame is None:
            return
        
        # Detect target region
        target_region = self.visual_servo.detect_largest_block(frame, self.target_region_color)
        
        if target_region is None:
            # Can't see target - rotate to search
            print(f"Searching for {self.target_region_color.upper()} region...")
            self.robot.rotate_clockwise()
            time.sleep(0.2)
            self.robot.stop()
            
            if self.check_timeout():
                print(f"Cannot find {self.target_region_color.upper()} region!")
                self.change_state(State.ERROR)
            
            if self.show_debug:
                cv2.imshow('Robot Vision', frame)
                cv2.waitKey(1)
            return
        
        # Found target region - switch to precise alignment
        print(f"Found {self.target_region_color.upper()} region!")
        self.robot.stop()
        self.change_state(State.ALIGN_REGION)
        
        if self.show_debug:
            debug_frame = self.visual_servo.draw_debug_info(
                frame, target_region, self.target_region_color)
            cv2.imshow('Robot Vision', debug_frame)
            cv2.waitKey(1)
    
    def state_align_region(self):
        """Precise visual servoing to align with target region"""
        frame = self.get_frame()
        if frame is None:
            return
        
        # Detect target region
        target_region = self.visual_servo.detect_largest_block(frame, self.target_region_color)
        
        if target_region is None:
            print("Lost target region!")
            self.change_state(State.GOTO_REGION)
            return
        
        # Get movement command
        command = self.visual_servo.get_movement_command(target_region)
        
        # Execute command
        if command == 'close':
            print(f"Reached {self.target_region_color.upper()} region!")
            self.robot.stop()
            self.change_state(State.DROP)
        elif command == 'forward':
            self.robot.forward()
            time.sleep(0.2)
            self.robot.stop()
        elif command == 'left':
            self.robot.left()
            time.sleep(0.15)
            self.robot.stop()
        elif command == 'right':
            self.robot.right()
            time.sleep(0.15)
            self.robot.stop()
        elif command == 'rotate_cw':
            self.robot.rotate_clockwise()
            time.sleep(0.1)
            self.robot.stop()
        elif command == 'rotate_ccw':
            self.robot.rotate_counterclockwise()
            time.sleep(0.1)
            self.robot.stop()
        
        # Debug display
        if self.show_debug:
            debug_frame = self.visual_servo.draw_debug_info(
                frame, target_region, self.target_region_color)
            cv2.imshow('Robot Vision', debug_frame)
            cv2.waitKey(1)
    
    def state_drop(self):
        """Drop the block"""
        print(f"Dropping {self.current_block_color.upper()} block...")
        self.robot.release()
        print("Block dropped!")
        
        self.blocks_transported += 1
        print(f"Blocks transported: {self.blocks_transported}")
        
        # Move back a bit
        self.robot.backward()
        time.sleep(0.5)
        self.robot.stop()
        
        # Reset mission data
        self.current_block_color = None
        self.target_region_color = None
        
        self.change_state(State.RETURN_START)
    
    def state_return_start(self):
        """Return to START region"""
        frame = self.get_frame()
        if frame is None:
            return
        
        # Detect START region (green)
        start_region = self.visual_servo.detect_largest_block(frame, 'green')
        
        if start_region is None:
            # Can't see START - search
            print("Searching for START region to return...")
            self.robot.rotate_counterclockwise()
            time.sleep(0.2)
            self.robot.stop()
            
            if self.check_timeout():
                print("Cannot find START region!")
                self.change_state(State.ERROR)
            
            if self.show_debug:
                cv2.imshow('Robot Vision', frame)
                cv2.waitKey(1)
            return
        
        # Navigate back to START
        command = self.visual_servo.get_movement_command(start_region)
        
        if command == 'close':
            print("Returned to START region!")
            self.robot.stop()
            time.sleep(0.5)
            self.change_state(State.START_ALIGN)  # Start next cycle
        elif command == 'forward':
            self.robot.forward()
            time.sleep(0.2)
            self.robot.stop()
        elif command == 'left':
            self.robot.left()
            time.sleep(0.15)
            self.robot.stop()
        elif command == 'right':
            self.robot.right()
            time.sleep(0.15)
            self.robot.stop()
        elif command == 'rotate_cw':
            self.robot.rotate_clockwise()
            time.sleep(0.1)
            self.robot.stop()
        elif command == 'rotate_ccw':
            self.robot.rotate_counterclockwise()
            time.sleep(0.1)
            self.robot.stop()
        
        # Debug display
        if self.show_debug:
            debug_frame = self.visual_servo.draw_debug_info(frame, start_region, 'green')
            cv2.imshow('Robot Vision', debug_frame)
            cv2.waitKey(1)
    
    def state_complete(self):
        """Mission complete"""
        print("\n" + "="*50)
        print("MISSION COMPLETE!")
        print(f"Total blocks transported: {self.blocks_transported}")
        print("="*50)
        self.robot.stop()
    
    def state_error(self):
        """Error state"""
        print("\n!!! ERROR STATE !!!")
        print("Stopping robot...")
        self.robot.stop()
    
    def run(self):
        """Main control loop"""
        print("\nStarting autonomous operation...")
        print("Press 'q' to quit, 's' to skip to next state")
        
        # State handler mapping
        state_handlers = {
            State.INIT: self.state_init,
            State.START_ALIGN: self.state_start_align,
            State.SEARCH_BLOCK: self.state_search_block,
            State.PICK: self.state_pick,
            State.GOTO_REGION: self.state_goto_region,
            State.ALIGN_REGION: self.state_align_region,
            State.DROP: self.state_drop,
            State.RETURN_START: self.state_return_start,
            State.COMPLETE: self.state_complete,
            State.ERROR: self.state_error
        }
        
        try:
            while True:
                # Execute current state handler
                handler = state_handlers.get(self.state)
                if handler:
                    handler()
                
                # Check for terminal states
                if self.state in [State.COMPLETE, State.ERROR]:
                    time.sleep(2)
                    break
                
                # Check keyboard
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    print("\nUser quit")
                    break
                elif key == ord('s'):
                    print("\nSkipping to next state...")
                    # Manual state skip for debugging
                    pass
                
                time.sleep(0.05)  # Small delay
        
        except KeyboardInterrupt:
            print("\n\nInterrupted by user")
        
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources"""
        print("\nCleaning up...")
        self.robot.stop()
        self.robot.close()
        self.camera.release()
        cv2.destroyAllWindows()
        print("Shutdown complete")


def main():
    """Main entry point"""
    import sys
    
    # Parse arguments
    serial_port = '/dev/ttyUSB0'
    camera_id = 0
    
    if len(sys.argv) > 1:
        serial_port = sys.argv[1]
    if len(sys.argv) > 2:
        camera_id = int(sys.argv[2])
    
    try:
        robot = ColorBlockRobot(serial_port=serial_port, camera_id=camera_id)
        robot.run()
    except Exception as e:
        print(f"\nFATAL ERROR: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    return 0


if __name__ == "__main__":
    exit(main())

