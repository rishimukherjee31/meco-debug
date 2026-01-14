#!/usr/bin/env python3
"""
MeCO Menu Node - ROS2 Node for ROV Menu System

Navigation Model:
- Gestures 1-4: Quick-jump to highlight items 0-3 (no execution)
- Up/Down: Navigate to items 5+ or fine-tune selection
- Select: Confirm and execute/enter the highlighted item
- Back: Return to parent menu
- Kill: Stop all processes and return to main menu

Topics:
    Subscribers:
        /menu_jump      (Int8) - Quick-jump: 1-4 highlights items 0-3
        /menu_select    (Int8) - Confirm/execute current selection
        /menu_navigate  (Int8) - 1=up, 2=down, 3=back
        /interrupt_flag (Int8) - 99 = kill all, return to main
        /stop_bag       (Int8) - 50 = stop recording
    
    Publishers:
        /meco/oled_menu (String)  - Side display
        /meco/oled_port (String)  - Front port display
        /meco/oled_stbd (String)  - Front starboard display
        /meco/tts       (String)  - Text-to-speech
"""

import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, String
from typing import Optional

from meco_menu.menu import Menu, MenuItem, ProcessMenuItem, MenuBuilder


class MenuNode(Node):
    """
    Main ROS2 Node for the MeCO Menu System.
    
    Handles gesture-based navigation and process management for ROV operations.
    """
    
    def __init__(self, yaml_file: str):
        super().__init__('meco_menu')
        
        # Load menu configuration
        self.root_menu = self._load_menu(yaml_file)
        self.current_menu: Menu = self.root_menu
        self.active_process: Optional[ProcessMenuItem] = None
        
        # Subscribers
        self.create_subscription(Int8, '/menu_select', self.select_callback, 1)
        self.create_subscription(Int8, '/menu_navigate', self.navigate_callback, 1)
        self.create_subscription(Int8, '/menu_jump', self.jump_callback, 1)
        self.create_subscription(Int8, '/interrupt_flag', self.interrupt_callback, 1)
        self.create_subscription(Int8, '/stop_bag', self.stop_bag_callback, 1)
        
        # Publishers
        self.side_pub = self.create_publisher(String, '/meco/oled_menu', 10)
        self.port_pub = self.create_publisher(String, '/meco/oled_port', 10)
        self.stbd_pub = self.create_publisher(String, '/meco/oled_stbd', 10)
        self.tts_publisher = self.create_publisher(String, '/meco/tts', 2)
        
        # Display initial menu
        self.display_menu()
        self.get_logger().info("MeCO Menu Node initialized")
    
    def _load_menu(self, yaml_file: str) -> Menu:
        """Load menu configuration from YAML file"""
        try:
            with open(yaml_file, 'r') as f:
                config = yaml.safe_load(f)
            
            return MenuBuilder.from_yaml_config(config)
        
        except Exception as e:
            self.get_logger().error(f"Failed to load menu config: {e}")
            # Return empty root menu
            return Menu({'id': 'root', 'label': 'Menu', 'type': 'submenu'})
    
    # ==================== Navigation ====================
    
    def navigate_callback(self, msg: Int8) -> None:
        """Handle navigation commands: 1=up, 2=down, 3=back"""
        direction = msg.data
        
        if direction == 1:  # Up
            self.current_menu.navigate_up()
            self.publish_tts("up")
            self.publish_f_port(" Up")
            self.publish_f_stbd(" Nav")
        
        elif direction == 2:  # Down
            self.current_menu.navigate_down()
            self.publish_tts("down")
            self.publish_f_port(" Down")
            self.publish_f_stbd(" Nav")
        
        elif direction == 3:  # Back
            self._go_back()
        
        self.display_menu()
    
    def _go_back(self) -> None:
        """Navigate back to parent menu or root"""
        if self.current_menu.parent is not None:
            self.current_menu = self.current_menu.parent
            self.publish_tts("back")
            self.publish_f_port(" Back")
            self.publish_f_stbd(" Nav")
        else:
            # Already at root
            self.publish_tts("main menu")
            self.publish_f_port(" Main")
            self.publish_f_stbd(" Menu")
        
        self.display_menu()
    
    def jump_callback(self, msg: Int8) -> None:
        """
        Handle quick-jump gestures 1-4.
        Gestures 1-4 jump to menu items 0-3 (highlight only, no execution).
        User must still use SELECT to confirm.
        """
        gesture_num = msg.data  # 1, 2, 3, or 4
        target_index = gesture_num - 1  # Convert to 0-based index
        
        # Validate the jump target exists
        if self.current_menu.set_current_index(target_index):
            item = self.current_menu.get_current_item()
            
            if item:
                self.publish_tts(f"{gesture_num}")
                self.publish_f_port(f" [{gesture_num}]")
                self.publish_f_stbd(item.display_f_stbd)
                self.get_logger().info(f"Jump to item {gesture_num}: {item.label}")
        else:
            # Target index doesn't exist, inform user
            self.publish_tts("invalid")
            self.publish_f_port(" N/A")
            self.publish_f_stbd(f" [{gesture_num}]")
            self.get_logger().warn(
                f"Jump target {gesture_num} out of range "
                f"(max: {self.current_menu.get_child_count()})"
            )
        
        self.display_menu()
    
    def _return_to_root(self) -> None:
        """Return directly to root menu"""
        self.current_menu = self.root_menu
        self.current_menu.current_option = 0
        self.display_menu()
    
    # ==================== Selection ====================
    
    def select_callback(self, msg: Int8) -> None:
        """
        Handle menu selection (SELECT gesture).
        Always selects the currently highlighted item.
        """
        item = self.current_menu.select_option()
        
        if item is None:
            return
        
        if isinstance(item, Menu):
            self._enter_submenu(item)
        elif isinstance(item, ProcessMenuItem):
            self._toggle_process(item)
        
        self.display_menu()
    
    def _enter_submenu(self, submenu: Menu) -> None:
        """Enter a submenu"""
        self.current_menu = submenu
        self.publish_tts(submenu.label)
        self.publish_f_port(" Enter")
        self.publish_f_stbd(submenu.display_f_stbd)
    
    def _toggle_process(self, process_item: ProcessMenuItem) -> None:
        """Toggle a process on/off"""
        was_running = process_item.is_running()
        process_item.toggle()
        
        if was_running:
            self.publish_tts(f"stopped {process_item.label}")
            self.publish_f_port(" Stop")
            self.active_process = None
        else:
            self.publish_tts(f"running {process_item.label}")
            self.publish_f_port(" Run")
            self.active_process = process_item
        
        self.publish_f_stbd(process_item.display_f_stbd)
    
    # ==================== Interrupt / Kill ====================
    
    def interrupt_callback(self, msg: Int8) -> None:
        """Handle interrupt/kill command - stops all processes and returns to main menu"""
        if msg.data == 99:
            self._kill_all_processes()
            self._return_to_root()
            
            self.publish_tts("killed")
            self.publish_f_port(" KILL")
            self.publish_f_stbd(" ALL")
            self._display_stopped()
    
    def stop_bag_callback(self, msg: Int8) -> None:
        """Handle stop bagging command"""
        if msg.data == 50:
            if self.active_process:
                self.active_process.stop()
                self.active_process = None
            
            self._return_to_root()
            self.publish_tts("stop bag")
            self.publish_f_port(" Stop")
            self.publish_f_stbd(" Bag")
            self.display_menu()
    
    def _kill_all_processes(self) -> None:
        """Stop all running processes in the menu tree"""
        self.root_menu.stop_all()
        self.active_process = None
    
    # ==================== Display ====================
    
    def display_menu(self) -> None:
        """Update all displays with current menu state"""
        menu_string = self.current_menu.print_menu()
        print(menu_string)
        self.publish_side_menu(menu_string)
        
        # Show current selection on front displays
        current_item = self.current_menu.get_current_item()
        if current_item:
            self.publish_f_stbd(f"*{current_item.display_f_stbd}")
            self.publish_f_port(current_item.display_f_port or current_item.get_status_indicator())
    
    def _display_stopped(self) -> None:
        """Display process killed message"""
        menu_string = "────────────────────\n"
        menu_string += " Process Killed\n"
        menu_string += "────────────────────"
        print(menu_string)
        self.publish_side_menu(menu_string)
    
    # ==================== Publishers ====================
    
    def publish_side_menu(self, text: str) -> None:
        """Publish to side OLED display"""
        msg = String()
        msg.data = text
        self.side_pub.publish(msg)
    
    def publish_f_port(self, text: str) -> None:
        """Publish to front port OLED display"""
        msg = String()
        msg.data = text
        self.port_pub.publish(msg)
    
    def publish_f_stbd(self, text: str) -> None:
        """Publish to front starboard OLED display"""
        msg = String()
        msg.data = text
        self.stbd_pub.publish(msg)
    
    def publish_tts(self, text: str) -> None:
        """Publish text-to-speech message"""
        msg = String()
        msg.data = text
        self.tts_publisher.publish(msg)


def main(args=None):
    """Main entry point for the menu node"""
    # Default menu file path - can be overridden via parameter
    menu_file_path = "/home/irvlab/meco_ws/src/meco-menu/meco_menu/menu.yaml"
    
    rclpy.init(args=args)
    
    try:
        menu = MenuNode(menu_file_path)
        rclpy.spin(menu)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
