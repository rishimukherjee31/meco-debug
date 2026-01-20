#!/usr/bin/env python3
"""
MeCO AR Tag Menu Controller - ROS2 Node for ArUco tag-based menu navigation

This controller runs alongside the gesture controller as an alternative input method.
Uses ArUco marker patterns (0-9) for menu control.

Pattern Mapping:
    Navigation:
        - Pattern 5:    Up
        - Pattern 6:    Down
        - Pattern 7:    Back
    
    Selection:
        - Pattern 9:    Select current item
    
    Quick Jump (1-4):
        - Pattern 1:    Jump to item 1
        - Pattern 2:    Jump to item 2
        - Pattern 3:    Jump to item 3
        - Pattern 4:    Jump to item 4
    
    System:
        - Pattern 0:    Kill/Interrupt (always active)
        - Pattern 8:    Stop bagging

Topics:
    Subscribers:
        /aruco_tag_detect/detected_id (Int32) - AR tag detection input
        /artag_toggle (Int8)                  - Enable/disable AR tags (1=on, 0=off)
    
    Publishers:
        /menu_navigate (Int8)   - 1=up, 2=down, 3=back
        /menu_select (Int8)     - Select current item
        /menu_jump (Int8)       - Jump to item 1-4
        /interrupt_flag (Int8)  - 99 = kill all
        /stop_bag (Int8)        - 50 = stop bagging
        /meco/tts (String)      - Text-to-speech feedback
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Int8, String
#from configuration_msgs import JOY
import time


class ARTagMenuController(Node):
    """
    ROS2 Node that translates ArUco tag detections into menu navigation commands.
    
    Features:
    - Time-based debouncing to prevent rapid repeated actions
    - Toggle to disable AR tag input (kill pattern always active for safety)
    - Audio feedback for each detection via TTS
    - Runs in parallel with gesture controller
    """
    
    # Pattern to action mapping
    # Patterns 10/11 are damaged and avoided
    PATTERN_MAP = {
        0: 'kill',      # Kill/Interrupt (always active)
        1: 'jump_1',    # Jump to item 1
        2: 'jump_2',    # Jump to item 2
        3: 'jump_3',    # Jump to item 3
        4: 'jump_4',    # Jump to item 4
        5: 'up',        # Navigate up
        6: 'down',      # Navigate down
        7: 'back',      # Navigate back
        8: 'stop_bag',  # Stop bagging
        9: 'select',    # Select current item
    }
    
    # Human-readable pattern names for logging
    PATTERN_NAMES = {
        0: "KILL",
        1: "JUMP_1",
        2: "JUMP_2",
        3: "JUMP_3",
        4: "JUMP_4",
        5: "UP",
        6: "DOWN",
        7: "BACK",
        8: "STOP_BAG",
        9: "SELECT",
    }
    
    def __init__(self):
        super().__init__('artag_menu_controller')
        
        # AR tag input subscriber
        self.create_subscription(
            Int32, 
            '/aruco_tag_detect/detected_id', 
            self.artag_callback, 
            1
        )
        
        # Toggle subscriber (1 = enabled, 0 = disabled)
        self.create_subscription(
            Int8,
            '/artag_toggle',
            self.toggle_callback,
            1
        )
        
        # Navigation publishers
        self.navigate_pub = self.create_publisher(Int8, '/menu_navigate', 1)
        self.select_pub = self.create_publisher(Int8, '/menu_select', 1)
        self.jump_pub = self.create_publisher(Int8, '/menu_jump', 1)
        
        # System publishers
        self.interrupt_pub = self.create_publisher(Int8, '/interrupt_flag', 1)
        self.stop_bag_pub = self.create_publisher(Int8, '/stop_bag', 1)
        
        # Feedback publishers
        self.tts_pub = self.create_publisher(String, '/meco/tts', 2)
        
        # State variables
        self.artag_enabled = True
        self.previous_action = ""
        self.last_action_time = 0.0
        
        # Timing configuration (in seconds)
        self.debounce_time = 0.5      # Minimum time between any actions
        self.repeat_cooldown = 2.0    # Time before same action can repeat
        
        self.get_logger().info("AR Tag Menu Controller initialized")
        self.get_logger().info(f"AR tags enabled: {self.artag_enabled}")
        self.get_logger().info(f"Debounce: {self.debounce_time}s, Repeat cooldown: {self.repeat_cooldown}s")
        self._log_pattern_map()
    
    def _log_pattern_map(self) -> None:
        """Log the pattern mapping for reference"""
        self.get_logger().info("Pattern mapping:")
        for pattern_id, action in self.PATTERN_MAP.items():
            self.get_logger().info(f"  Pattern {pattern_id}: {action}")
    
    def toggle_callback(self, msg: Int8) -> None:
        """
        Handle AR tag enable/disable toggle.
        
        Args:
            msg: Int8 with data 1 (enable) or 0 (disable)
        """
        self.artag_enabled = (msg.data == 1)
        
        status = "ENABLED" if self.artag_enabled else "DISABLED"
        self.get_logger().info(f"AR tags {status}")
        
        # Provide feedback
        self.publish_tts(f"AR tags {status.lower()}")
    
    def artag_callback(self, msg: Int32) -> None:
        """
        Handle incoming AR tag detections.
        
        Implements time-based debouncing and maps patterns to menu actions.
        """
        pattern_id = msg.data
        
        # Ignore invalid detections (-1 means no marker found)
        if pattern_id < 0:
            return
        
        current_time = time.time()
        
        # Debounce: ignore if too soon after last action
        if current_time - self.last_action_time < self.debounce_time:
            return
        
        action = self.PATTERN_MAP.get(pattern_id, None)
        
        if action is None:
            # Unknown or unmapped pattern (including damaged 10/11)
            self.get_logger().debug(f"Unmapped pattern: {pattern_id}")
            return
        
        # Kill pattern is ALWAYS active (safety feature)
        if action == 'kill':
            self._handle_kill()
            self.last_action_time = current_time
            self.previous_action = action
            return
        
        # All other patterns respect the toggle
        if not self.artag_enabled:
            return
        
        # Check for repeat prevention
        if not self._should_execute(action, current_time):
            return
        
        # Execute the action
        self._execute_action(action)
        
        # Update state
        self.previous_action = action
        self.last_action_time = current_time
    
    def _should_execute(self, action: str, current_time: float) -> bool:
        """
        Determine if an action should be executed based on timing rules.
        
        Args:
            action: The action string to check
            current_time: Current timestamp
            
        Returns:
            True if action should be executed
        """
        # Allow if different from previous action
        if action != self.previous_action:
            return True
        
        # Allow if repeat cooldown has elapsed (permits intentional repeats)
        time_since_last = current_time - self.last_action_time
        if time_since_last >= self.repeat_cooldown:
            return True
        
        return False
    
    def _execute_action(self, action: str) -> None:
        """
        Execute the menu action corresponding to a pattern.
        
        Args:
            action: Action string from PATTERN_MAP
        """
        if action == 'up':
            self._handle_up()
        elif action == 'down':
            self._handle_down()
        elif action == 'back':
            self._handle_back()
        elif action == 'select':
            self._handle_select()
        elif action.startswith('jump_'):
            jump_num = int(action.split('_')[1])
            self._handle_jump(jump_num)
        elif action == 'stop_bag':
            self._handle_stop_bag()
    
    # ==================== Action Handlers ====================
    
    def _handle_up(self) -> None:
        """Navigate up in menu"""
        msg = Int8()
        msg.data = 1  # 1 = up
        self.navigate_pub.publish(msg)
        
        self.publish_tts("up")
        self.get_logger().debug("AR Tag Action: UP")
    
    def _handle_down(self) -> None:
        """Navigate down in menu"""
        msg = Int8()
        msg.data = 2  # 2 = down
        self.navigate_pub.publish(msg)
        
        self.publish_tts("down")
        self.get_logger().debug("AR Tag Action: DOWN")
    
    def _handle_back(self) -> None:
        """Navigate back to parent menu"""
        msg = Int8()
        msg.data = 3  # 3 = back
        self.navigate_pub.publish(msg)
        
        self.publish_tts("back")
        self.get_logger().debug("AR Tag Action: BACK")
    
    def _handle_select(self) -> None:
        """Select current menu item"""
        msg = Int8()
        msg.data = 1  # Any positive value triggers select
        self.select_pub.publish(msg)
        
        self.publish_tts("select")
        self.get_logger().debug("AR Tag Action: SELECT")
    
    def _handle_jump(self, item_num: int) -> None:
        """
        Jump to menu item 1-4.
        
        Args:
            item_num: Item number (1-4)
        """
        msg = Int8()
        msg.data = item_num
        self.jump_pub.publish(msg)
        
        self.publish_tts(f"{item_num}")
        self.get_logger().debug(f"AR Tag Action: JUMP to {item_num}")
    
    def _handle_kill(self) -> None:
        """
        Kill all processes and return to main menu.
        This action is ALWAYS active regardless of AR tag toggle.
        """
        msg = Int8()
        msg.data = 99
        self.interrupt_pub.publish(msg)

        #joy_msg = JOY_MSG()
        #joy_msg.data.arm = 1
        #self.joy_pub.publish(joy_msg)
        
        self.publish_tts("kill")
        self.get_logger().warn("AR Tag Action: KILL (interrupt)")
    
    def _handle_stop_bag(self) -> None:
        """Stop rosbag recording"""
        msg = Int8()
        msg.data = 50
        self.stop_bag_pub.publish(msg)
        
        self.publish_tts("stop bag")
        self.get_logger().debug("AR Tag Action: STOP BAG")

    
    # ==================== Feedback Publishers ====================
    
    def publish_tts(self, text: str) -> None:
        """Publish text-to-speech message"""
        msg = String()
        msg.data = text
        self.tts_pub.publish(msg)


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        controller = ARTagMenuController()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
