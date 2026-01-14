#!/usr/bin/env python3
"""
MeCO Menu - Menu class for hierarchical menu system

A Menu contains MenuItems which can be either:
- A Process (Docker, ROS2 Node, Service, Action, Command)
- A SubMenu (another Menu)

Menu Class:
- Menu[] submenu (children)
- bool process (is this a process item?)
- int process_ID
- bool has_container
- int current_option
- String current_container
- Process process (if this is a process item)

Methods:
- Select Option
- Navigate Menu
- Print Menu
- Create Menu YAML
"""

from typing import Optional, List, Dict, Any, Union
from meco_menu.processes import Process, ProcessFactory, ProcessType


class MenuItem:
    """
    Base class for menu items.
    
    A MenuItem can be either a Process or a SubMenu (Menu).
    Contains display information for OLED screens.
    
    Attributes:
        id: Unique identifier
        label: Display name
        display_side: Text for side OLED display
        display_f_stbd: Text for front starboard OLED
        display_f_port: Text for front port OLED
        is_process: True if this item is a process, False if submenu
    """
    
    def __init__(self, config: Dict[str, Any]):
        self.id = config.get('id', 'unnamed')
        self.label = config.get('label', 'Unknown')
        self.display_side = config.get('display_side', self.label)
        self.display_f_stbd = config.get('display_f_stbd', self.label[:4])
        self.display_f_port = config.get('display_f_port', '')
        self.is_process = False
    
    def get_status_indicator(self) -> str:
        """Returns status indicator: ● running, ○ stopped"""
        return "○"
    
    def is_running(self) -> bool:
        """Check if item is running"""
        return False


class ProcessMenuItem(MenuItem):
    """
    Menu item that wraps a Process.
    
    Attributes:
        process: The underlying Process instance
        has_container: Whether this process runs in a container
    """
    
    def __init__(self, config: Dict[str, Any]):
        super().__init__(config)
        self.is_process = True
        self.has_container = config.get('has_container', False)
        
        # Create the process using factory
        self.process: Optional[Process] = ProcessFactory.create(config)
        
        if self.process:
            self.has_container = self.process.process_type == ProcessType.DOCKER
    
    def execute(self) -> bool:
        """Execute the process"""
        if self.process:
            return self.process.execute()
        return False
    
    def stop(self) -> bool:
        """Stop the process"""
        if self.process:
            return self.process.stop()
        return False
    
    def toggle(self) -> bool:
        """Toggle the process"""
        if self.process:
            return self.process.toggle()
        return False
    
    def is_running(self) -> bool:
        """Check if process is running"""
        if self.process:
            return self.process.is_running()
        return False
    
    def get_status_indicator(self) -> str:
        """Returns status indicator: ● running, ○ stopped"""
        return "●" if self.is_running() else "○"
    
    def get_launch_command(self) -> str:
        """Get the launch command"""
        if self.process:
            return self.process.launch_command
        return ""


class Menu(MenuItem):
    """
    Menu class that contains other menu items (processes or submenus).
    
    Implements hierarchical menu navigation with max depth of 2.
    
    Attributes:
        children: List of child MenuItem objects (ProcessMenuItem or Menu)
        current_option: Currently selected item index
        parent: Reference to parent Menu (None for root)
        depth: Depth in menu hierarchy (0 = root)
        current_container: Name of currently running container (if any)
    """
    
    MAX_DEPTH = 2
    
    def __init__(self, config: Dict[str, Any], parent: Optional['Menu'] = None, depth: int = 0):
        super().__init__(config)
        self.is_process = False
        self.children: List[MenuItem] = []
        self.current_option: int = 0
        self.parent = parent
        self.depth = depth
        self.current_container: str = ""
        
        # Parse children (max depth of 2)
        if 'children' in config and depth < self.MAX_DEPTH:
            for child_config in config['children']:
                child = self._create_child(child_config, depth + 1)
                if child:
                    self.children.append(child)
    
    def _create_child(self, config: Dict[str, Any], child_depth: int) -> Optional[MenuItem]:
        """
        Factory method to create appropriate MenuItem subclass.
        
        Args:
            config: Item configuration dictionary
            child_depth: Depth of the child in hierarchy
            
        Returns:
            MenuItem instance (ProcessMenuItem or Menu) or None
        """
        item_type = config.get('type', 'command')
        
        # Check if this is a submenu
        if item_type == 'submenu':
            if child_depth < self.MAX_DEPTH:
                return Menu(config, parent=self, depth=child_depth)
            else:
                print(f"[Menu] Max depth exceeded, cannot create submenu: {config.get('label')}")
                return None
        else:
            # It's a process
            return ProcessMenuItem(config)
    
    # ==================== Navigation ====================
    
    def navigate_up(self) -> int:
        """Move selection up, returns new index"""
        self.current_option = max(0, self.current_option - 1)
        return self.current_option
    
    def navigate_down(self) -> int:
        """Move selection down, returns new index"""
        self.current_option = min(len(self.children) - 1, self.current_option + 1)
        return self.current_option
    
    def set_current_index(self, index: int) -> bool:
        """
        Set current index if valid.
        Returns True if index was valid and set.
        """
        if 0 <= index < len(self.children):
            self.current_option = index
            return True
        return False
    
    def get_current_item(self) -> Optional[MenuItem]:
        """Get currently selected item"""
        if 0 <= self.current_option < len(self.children):
            return self.children[self.current_option]
        return None
    
    def get_child_count(self) -> int:
        """Get number of children"""
        return len(self.children)
    
    # ==================== Selection ====================
    
    def select_option(self) -> Optional[MenuItem]:
        """
        Select the current option.
        Returns the selected MenuItem for the caller to handle.
        """
        return self.get_current_item()
    
    # ==================== Status ====================
    
    def is_running(self) -> bool:
        """Check if any child process is running"""
        for child in self.children:
            if child.is_running():
                return True
        return False
    
    def get_status_indicator(self) -> str:
        """Returns status indicator: ● if any child running, ○ otherwise"""
        return "●" if self.is_running() else "○"
    
    def stop_all(self) -> None:
        """Stop all running processes in this menu and submenus"""
        for child in self.children:
            if isinstance(child, ProcessMenuItem):
                child.stop()
            elif isinstance(child, Menu):
                child.stop_all()
    
    # ==================== Display ====================
    
    def print_menu(self, max_width: int = 20) -> str:
        """
        Generate display string with current selection marked.
        
        Format:
        ────────────────────
         Menu Label
        ────────────────────
        *1.First Item      ●
         2.Second Item     ○
         3.Third Item      ○
         4.Fourth Item     ○
          .Fifth Item      ○   (items 5+ have no gesture number)
        
        Args:
            max_width: Maximum character width for display
            
        Returns:
            Formatted menu string
        """
        lines = [f"{'─' * max_width}"]
        lines.append(f" {self.label}")
        lines.append(f"{'─' * max_width}")
        
        for i, item in enumerate(self.children):
            # Current selection indicator
            selector = "*" if i == self.current_option else " "
            # Running status indicator
            status = item.get_status_indicator()
            # Gesture number for first 4 items (1-4), blank for others
            gesture_num = str(i + 1) if i < 4 else " "
            # Format: "*1.Label       ●"
            label_display = item.display_side[:max_width - 6]
            lines.append(f"{selector}{gesture_num}.{label_display:<{max_width - 6}} {status}")
        
        return "\n".join(lines)
    
    # ==================== YAML Generation ====================
    
    def create_menu_yaml(self) -> Dict[str, Any]:
        """
        Generate YAML-compatible dictionary from current menu structure.
        
        Returns:
            Dictionary that can be serialized to YAML
        """
        result = {
            'id': self.id,
            'label': self.label,
            'display_side': self.display_side,
            'display_f_stbd': self.display_f_stbd,
            'display_f_port': self.display_f_port,
            'type': 'submenu',
            'children': []
        }
        
        for child in self.children:
            if isinstance(child, Menu):
                result['children'].append(child.create_menu_yaml())
            elif isinstance(child, ProcessMenuItem):
                # Basic process info
                child_dict = {
                    'id': child.id,
                    'label': child.label,
                    'display_side': child.display_side,
                    'display_f_stbd': child.display_f_stbd,
                    'display_f_port': child.display_f_port,
                }
                if child.process:
                    child_dict['type'] = child.process.process_type.value
                    child_dict['command'] = child.process.launch_command
                result['children'].append(child_dict)
        
        return result


class MenuBuilder:
    """
    Builder class for creating Menu structures from YAML config.
    """
    
    @staticmethod
    def from_yaml_config(config: Dict[str, Any]) -> Menu:
        """
        Build a Menu tree from YAML configuration.
        
        Args:
            config: Dictionary loaded from YAML file
            
        Returns:
            Root Menu instance
        """
        root_config = {
            'id': 'root',
            'label': config.get('label', 'MeCO Menu'),
            'display_side': config.get('display_side', 'MeCO Menu'),
            'display_f_stbd': config.get('display_f_stbd', 'MENU'),
            'display_f_port': config.get('display_f_port', ''),
            'type': 'submenu',
            'children': config.get('menu', [])
        }
        
        return Menu(root_config, parent=None, depth=0)
