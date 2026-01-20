#!/usr/bin/env python3
"""
MeCO Gesture Menu Controller - ROS2 Node for gesture-based menu navigation

Gesture Mapping:
    Navigation:
        - like (4):     Up
        - dislike (1):  Down
        - call (0):     Back
    
    Selection:
        - ok (6):       Select current item
    
    Quick Jump (1-4):
        - one (7):      Jump to item 1
        - two_up (16):  Jump to item 2
        - three (14):   Jump to item 3
        - four (3):     Jump to item 4
    
    System:
        - palm (8):     Kill/Interrupt (always active)
        - rock (11):    Stop bagging

Topics:
    Subscribers:
        /robo_chat_gest/detected_id (Int32) - Gesture detection input
        /gesture_toggle (Int8)              - Enable/disable gestures (1=on, 0=off)
    
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


class GestureMenuController(Node):
    """
    ROS2 Node that translates hand gestures into menu navigation commands.
    
    Features:
    - Time-based gesture debouncing to prevent rapid repeated actions
    - Toggle to disable gestures (kill gesture always active for safety)
    - Audio feedback for each gesture via TTS
    """
    
    # Gesture ID to name mapping
    GESTURE_CLASSES = {
        0: "call",
        1: "dislike",
        2: "fist",
        3: "four",
        4: "like",
        5: "mute",
        6: "ok",
        7: "one",
        8: "palm",
        9: "peace",
        10: "peace_inverted",
        11: "rock",
        12: "stop",
        13: "stop_inverted",
        14: "three",
        15: "three2",
        16: "two_up",
        17: "two_up_inverted",
        18: "no_gesture",
    }
    
    # Gesture to action mapping
    GESTURE_MAP = {
        4: 'up',        # like
        1: 'down',      # dislike
        6: 'select',    # ok
        0: 'back',      # call
        8: 'kill',      # palm (always active)
        7: 'jump_1',    # one
        16: 'jump_2',   # two_up
        14: 'jump_3',   # three
        3: 'jump_4',    # four
        11: 'stop_bag', # rock
    }
    
    def __init__(self):
        super().__init__('gesture_menu_controller')
        
        # Gesture input subscriber
        self.create_subscription(
            Int32, 
            '/robo_chat_gest/detected_id', 
            self.gesture_callback, 
            1
        )
        
        # Toggle subscriber (1 = enabled, 0 = disabled)
        self.create_subscription(
            Int8,
            '/gesture_toggle',
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
        self.gestures_enabled = True
        self.previous_action = ""
        self.last_action_time = 0.0
        
        # Timing configuration (in seconds)
        self.debounce_time = 0.3      # Minimum time between any actions
        self.repeat_cooldown = 1.5    # Time before same action can repeat
        
        self.get_logger().info("Gesture Menu Controller initialized")
        self.get_logger().info(f"Gestures enabled: {self.gestures_enabled}")
        self.get_logger().info(f"Debounce: {self.debounce_time}s, Repeat cooldown: {self.repeat_cooldown}s")
    
    def toggle_callback(self, msg: Int8) -> None:
        """
        Handle gesture enable/disable toggle.
        
        Args:
            msg: Int8 with data 1 (enable) or 0 (disable)
        """
        self.gestures_enabled = (msg.data == 1)
        
        status = "ENABLED" if self.gestures_enabled else "DISABLED"
        self.get_logger().info(f"Gestures {status}")
        
        # Provide feedback
        self.publish_tts(f"gestures {status.lower()}")
    
    def gesture_callback(self, msg: Int32) -> None:
        """
        Handle incoming gesture detections.
        
        Implements time-based debouncing and maps gestures to menu actions.
        """
        current_time = time.time()
        
        # Debounce: ignore if too soon after last action
        if current_time - self.last_action_time < self.debounce_time:
            return
        
        gesture_id = msg.data
        action = self.GESTURE_MAP.get(gesture_id, None)
        
        if action is None:
            # Unknown or unmapped gesture
            return
        
        # Kill gesture is ALWAYS active (at the request of the safety officer)
        if action == 'kill':
            self._handle_kill()
            self.last_action_time = current_time
            self.previous_action = action
            return
        
        # other gestures respect the toggle
        if not self.gestures_enabled:
            return
        
        # repeat prevention
        if not self._should_execute(action, current_time):
            return
        
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
        Execute the menu action corresponding to a gesture.
        
        Args:
            action: Action string from GESTURE_MAP
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
        self.get_logger().debug("Action: UP")
    
    def _handle_down(self) -> None:
        """Navigate down in menu"""
        msg = Int8()
        msg.data = 2  # 2 = down
        self.navigate_pub.publish(msg)
        
        self.publish_tts("down")
        self.get_logger().debug("Action: DOWN")
    
    def _handle_back(self) -> None:
        """Navigate back to parent menu"""
        msg = Int8()
        msg.data = 3  # 3 = back
        self.navigate_pub.publish(msg)
        
        self.publish_tts("back")
        self.get_logger().debug("Action: BACK")
    
    def _handle_select(self) -> None:
        """Select current menu item"""
        msg = Int8()
        msg.data = 1  # Any positive value triggers select
        self.select_pub.publish(msg)
        
        self.publish_tts("select")
        self.get_logger().debug("Action: SELECT")
    
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
        self.get_logger().debug(f"Action: JUMP to {item_num}")
    
    def _handle_kill(self) -> None:
        """
        Kill all processes and return to main menu.
        This action is ALWAYS active regardless of gesture toggle. This will also TO DO: TURN OFF the motors. 
        """
        msg = Int8()
        msg.data = 99
        self.interrupt_pub.publish(msg)

        #joy_msg = JOY_MSG()
        #joy_msg.data.arm = 1
        #self.joy_pub.publish(joy_msg)
        
        self.publish_tts("kill")
        self.get_logger().warn("Action: KILL (interrupt)")
    
    def _handle_stop_bag(self) -> None:
        """Stop rosbag recording"""
        msg = Int8()
        msg.data = 50
        self.stop_bag_pub.publish(msg)
        
        self.publish_tts("stop bag")
        self.get_logger().debug("Action: STOP BAG")
    
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
        controller = GestureMenuController()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
