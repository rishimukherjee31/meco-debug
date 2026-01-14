#!/usr/bin/env python3
"""
MeCO Menu Node - DEBUG MODE
Terminal-based version for testing without hardware/OLED displays

Keyboard Controls:
    w/↑     - Navigate up
    s/↓     - Navigate down
    1-4     - Quick jump to items 0-3
    ENTER   - Select current item
    b       - Back to parent menu
    k       - Kill all processes
    q       - Quit debug node

All OLED/TTS outputs are displayed in terminal windows.
"""

import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, String
from typing import Optional
import sys
import tty
import termios
import threading
import os

from meco_menu.menu import Menu, MenuItem, ProcessMenuItem, MenuBuilder


class MenuNodeDebug(Node):
    """
    Debug version of MeCO Menu Node with terminal-based UI.
    
    Displays all OLED screens and TTS output in the terminal.
    Uses keyboard input for navigation instead of ROS topics.
    """
    
    def __init__(self, yaml_file: str):
        super().__init__('meco_menu_debug')
        
        # Load menu configuration
        self.root_menu = self._load_menu(yaml_file)
        self.current_menu: Menu = self.root_menu
        self.active_process: Optional[ProcessMenuItem] = None
        
        # Display state
        self.side_display = ""
        self.port_display = ""
        self.stbd_display = ""
        self.tts_display = ""
        self.status_message = "Debug Mode Ready"
        
        # Subscribers (still available for ROS integration testing)
        self.create_subscription(Int8, '/menu_select', self.select_callback, 1)
        self.create_subscription(Int8, '/menu_navigate', self.navigate_callback, 1)
        self.create_subscription(Int8, '/menu_jump', self.jump_callback, 1)
        self.create_subscription(Int8, '/interrupt_flag', self.interrupt_callback, 1)
        self.create_subscription(Int8, '/stop_bag', self.stop_bag_callback, 1)
        
        # Publishers (for testing with other nodes)
        self.side_pub = self.create_publisher(String, '/meco/oled_menu', 10)
        self.port_pub = self.create_publisher(String, '/meco/oled_port', 10)
        self.stbd_pub = self.create_publisher(String, '/meco/oled_stbd', 10)
        self.tts_publisher = self.create_publisher(String, '/meco/tts', 2)
        
        # Keyboard input thread
        self.running = True
        self.keyboard_thread = threading.Thread(target=self._keyboard_input_loop, daemon=True)
        
        # Display initial menu
        self.display_menu()
        self.refresh_display()
        self.get_logger().info("MeCO Menu Debug Node initialized")
        
        # Start keyboard input
        self.keyboard_thread.start()
    
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
    
    # ==================== Keyboard Input ====================
    
    def _keyboard_input_loop(self):
        """Background thread for keyboard input"""
        # Save terminal settings
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        
        try:
            tty.setcbreak(fd)
            
            while self.running:
                if sys.stdin in select.select([sys.stdin], [], [], 0.1)[0]:
                    char = sys.stdin.read(1)
                    self._handle_keypress(char)
                    
        except Exception as e:
            self.get_logger().error(f"Keyboard input error: {e}")
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    
    def _handle_keypress(self, char: str):
        """Handle keyboard input"""
        # Navigation
        if char in ['w', 'W', '\x1b[A']:  # w or up arrow
            self.current_menu.navigate_up()
            self.publish_tts("up")
            self.publish_f_port(" Up")
            self.publish_f_stbd(" Nav")
            self.status_message = "Navigated UP"
            self.display_menu()
            
        elif char in ['s', 'S', '\x1b[B']:  # s or down arrow
            self.current_menu.navigate_down()
            self.publish_tts("down")
            self.publish_f_port(" Down")
            self.publish_f_stbd(" Nav")
            self.status_message = "Navigated DOWN"
            self.display_menu()
            
        elif char == 'b' or char == 'B':  # Back
            self._go_back()
            self.status_message = "Navigated BACK"
            
        # Quick jump (1-4)
        elif char in ['1', '2', '3', '4']:
            gesture_num = int(char)
            self._handle_jump(gesture_num)
            
        # Select
        elif char == '\n' or char == '\r':  # Enter
            self.status_message = "SELECTED current item"
            self._handle_select()
            
        # Kill all
        elif char == 'k' or char == 'K':
            self._kill_all_processes()
            self._return_to_root()
            self.publish_tts("killed")
            self.publish_f_port(" KILL")
            self.publish_f_stbd(" ALL")
            self.status_message = "KILLED all processes"
            self._display_stopped()
            
        # Quit
        elif char == 'q' or char == 'Q':
            self.status_message = "Quitting..."
            self.refresh_display()
            self.running = False
            self._kill_all_processes()
            rclpy.shutdown()
    
    # ==================== Navigation ====================
    
    def navigate_callback(self, msg: Int8) -> None:
        """Handle navigation commands: 1=up, 2=down, 3=back"""
        direction = msg.data
        
        if direction == 1:  # Up
            self.current_menu.navigate_up()
            self.publish_tts("up")
            self.publish_f_port(" Up")
            self.publish_f_stbd(" Nav")
            self.status_message = "ROS: Navigated UP"
        
        elif direction == 2:  # Down
            self.current_menu.navigate_down()
            self.publish_tts("down")
            self.publish_f_port(" Down")
            self.publish_f_stbd(" Nav")
            self.status_message = "ROS: Navigated DOWN"
        
        elif direction == 3:  # Back
            self._go_back()
            self.status_message = "ROS: Navigated BACK"
        
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
        """Handle quick-jump gestures 1-4"""
        gesture_num = msg.data
        self._handle_jump(gesture_num)
    
    def _handle_jump(self, gesture_num: int):
        """Handle jump to menu item"""
        target_index = gesture_num - 1
        
        if self.current_menu.set_current_index(target_index):
            item = self.current_menu.get_current_item()
            
            if item:
                self.publish_tts(f"{gesture_num}")
                self.publish_f_port(f" [{gesture_num}]")
                self.publish_f_stbd(item.display_f_stbd)
                self.status_message = f"Jumped to item {gesture_num}: {item.label}"
                self.get_logger().info(f"Jump to item {gesture_num}: {item.label}")
        else:
            self.publish_tts("invalid")
            self.publish_f_port(" N/A")
            self.publish_f_stbd(f" [{gesture_num}]")
            self.status_message = f"Jump {gesture_num} out of range"
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
        """Handle menu selection"""
        self._handle_select()
    
    def _handle_select(self):
        """Handle selection of current item"""
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
        self.status_message = f"Entered submenu: {submenu.label}"
    
    def _toggle_process(self, process_item: ProcessMenuItem) -> None:
        """Toggle a process on/off"""
        was_running = process_item.is_running()
        process_item.toggle()
        
        if was_running:
            self.publish_tts(f"stopped {process_item.label}")
            self.publish_f_port(" Stop")
            self.active_process = None
            self.status_message = f"STOPPED: {process_item.label}"
        else:
            self.publish_tts(f"running {process_item.label}")
            self.publish_f_port(" Run")
            self.active_process = process_item
            self.status_message = f"STARTED: {process_item.label}"
        
        self.publish_f_stbd(process_item.display_f_stbd)
    
    # ==================== Interrupt / Kill ====================
    
    def interrupt_callback(self, msg: Int8) -> None:
        """Handle interrupt/kill command"""
        if msg.data == 99:
            self._kill_all_processes()
            self._return_to_root()
            
            self.publish_tts("killed")
            self.publish_f_port(" KILL")
            self.publish_f_stbd(" ALL")
            self.status_message = "KILLED all processes"
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
            self.status_message = "Stopped bag recording"
            self.display_menu()
    
    def _kill_all_processes(self) -> None:
        """Stop all running processes in the menu tree"""
        self.root_menu.stop_all()
        self.active_process = None
    
    # ==================== Display ====================
    
    def display_menu(self) -> None:
        """Update all displays with current menu state"""
        menu_string = self.current_menu.print_menu()
        self.publish_side_menu(menu_string)
        
        # Show current selection on front displays
        current_item = self.current_menu.get_current_item()
        if current_item:
            self.publish_f_stbd(f"*{current_item.display_f_stbd}")
            self.publish_f_port(current_item.display_f_port or current_item.get_status_indicator())
        
        self.refresh_display()
    
    def _display_stopped(self) -> None:
        """Display process killed message"""
        menu_string = "────────────────────\n"
        menu_string += " Process Killed\n"
        menu_string += "────────────────────"
        self.publish_side_menu(menu_string)
        self.refresh_display()
    
    def refresh_display(self):
        """Refresh the terminal display with all current info"""
        # Clear screen
        os.system('clear' if os.name == 'posix' else 'cls')
        
        # Header
        print("╔═══════════════════════════════════════════════════════════════════════╗")
        print("║               MeCO Menu - Debug Mode                                  ║")
        print("╚═══════════════════════════════════════════════════════════════════════╝")
        print()
        
        # Keyboard controls
        print("┌─ KEYBOARD CONTROLS ─────────────────────────────────────────────────────────────────┐")
        print("│ w/↑: Up  │ s/↓: Down  │ 1-4: Jump  │ ENTER: Select  │ b: Back  │ k: Kill  │ q: Quit │")
        print("└─────────────────────────────────────────────────────────────────────────────────────┘")
        print()
        
        # Status message
        print(f"┌─ STATUS ─────────────────────────────────────────────────────────────┐")
        print(f"│ {self.status_message:<69}│")
        print(f"└──────────────────────────────────────────────────────────────────────┘")
        print()
        
        # Main menu display (Side OLED)
        print("┌─ SIDE OLED DISPLAY ──────────────────────────────────────────────────┐")
        for line in self.side_display.split('\n'):
            print(f"│ {line:<69}│")
        print("└──────────────────────────────────────────────────────────────────────┘")
        print()
        
        # Front displays (Port and Starboard)
        print("┌─ FRONT DISPLAYS ─────────────────────────────────────────────────────┐")
        print(f"│ PORT OLED: {self.port_display:<20} │ STBD OLED: {self.stbd_display:<20}    │")
        print("└──────────────────────────────────────────────────────────────────────┘")
        print()
        
        # TTS Output
        print("┌─ TEXT-TO-SPEECH OUTPUT ──────────────────────────────────────────────┐")
        print(f"│ {self.tts_display:<69}│")
        print("└──────────────────────────────────────────────────────────────────────┘")
        print()
        
        # Active process info
        if self.active_process:
            print("┌─ ACTIVE PROCESS ─────────────────────────────────────────────────────┐")
            print(f"│ Process: {self.active_process.label:<60} │")
            print(f"│ Command: {self.active_process.get_launch_command():<60} │")
            print(f"│ Running: {'YES' if self.active_process.is_running() else 'NO':<60} │")
            print("└──────────────────────────────────────────────────────────────────────┘")
            print()
    
    # ==================== Publishers ====================
    
    def publish_side_menu(self, text: str) -> None:
        """Publish to side OLED display (and store for terminal)"""
        self.side_display = text
        msg = String()
        msg.data = text
        self.side_pub.publish(msg)
    
    def publish_f_port(self, text: str) -> None:
        """Publish to front port OLED display (and store for terminal)"""
        self.port_display = text
        msg = String()
        msg.data = text
        self.port_pub.publish(msg)
    
    def publish_f_stbd(self, text: str) -> None:
        """Publish to front starboard OLED display (and store for terminal)"""
        self.stbd_display = text
        msg = String()
        msg.data = text
        self.stbd_pub.publish(msg)
    
    def publish_tts(self, text: str) -> None:
        """Publish text-to-speech message (and store for terminal)"""
        self.tts_display = text
        msg = String()
        msg.data = text
        self.tts_publisher.publish(msg)


def main(args=None):
    """Main entry point for the debug menu node"""
    # You can override this path via command line
    import sys
    if len(sys.argv) > 1:
        menu_file_path = sys.argv[1]
    else:
        menu_file_path = "/home/rishi/meco_ws/src/meco_menu/config/menu.yaml"
    
    # Check if file exists
    if not os.path.exists(menu_file_path):
        print(f"Error: Menu file not found: {menu_file_path}")
        print(f"Usage: python3 menu_node_debug.py [path/to/menu.yaml]")
        return
    
    rclpy.init(args=args)
    
    try:
        # Import select here (needed for keyboard input)
        import select
        globals()['select'] = select
        
        menu = MenuNodeDebug(menu_file_path)
        
        # Create a timer to keep ROS2 spinning
        def spin_once():
            rclpy.spin_once(menu, timeout_sec=0.1)
        
        # Keep spinning until quit
        while menu.running and rclpy.ok():
            spin_once()
            
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        if 'menu' in locals():
            menu.running = False
        rclpy.shutdown()


if __name__ == '__main__':
    main()