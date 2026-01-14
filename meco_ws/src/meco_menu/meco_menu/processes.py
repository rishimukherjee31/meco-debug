#!/usr/bin/env python3
"""
MeCO Processes - Process class hierarchy for menu system

Class Hierarchy:
    Process (ABC)
    ├── DockerProcess    - Runs Docker containers
    └── ROS2Process (ABC)
        ├── NodeProcess      - ros2 run / ros2 launch
        ├── ServiceProcess   - ros2 service call
        ├── ActionProcess    - ros2 action send_goal
        └── ROSCommand       - ros2 param set / other ros2 CLI
    └── BashCommand      - Generic shell commands

All processes have a launch_command that is executed via CLI.
"""

import subprocess
import os
from time import sleep
from typing import Optional, Dict, Any
from abc import ABC, abstractmethod
from enum import Enum


class ProcessType(Enum):
    """Enumeration of process types"""
    DOCKER = "docker"
    NODE = "node"
    SERVICE = "service"
    ACTION = "action"
    ROS_COMMAND = "ros_command"
    COMMAND = "command"  # Generic bash command


class Process(ABC):
    """
    Abstract base class for all processes.
    
    All processes have a launch_command that can be executed via CLI.
    
    Attributes:
        id: Unique identifier
        label: Display name
        launch_command: CLI command to execute
        process: subprocess.Popen instance when running
    """
    
    def __init__(self, config: Dict[str, Any]):
        self.id = config.get('id', 'unnamed')
        self.label = config.get('label', 'Unknown')
        self.launch_command: str = ""
        self.process: Optional[subprocess.Popen] = None
        self.process_type: ProcessType = ProcessType.COMMAND
        
        # Build the launch command
        self._build_command(config)
    
    @abstractmethod
    def _build_command(self, config: Dict[str, Any]) -> None:
        """Build the launch_command from config. Must be implemented by subclasses."""
        pass
    
    def execute(self) -> bool:
        """Start the process. Returns True if successful."""
        if self.is_running():
            print(f"[Process] {self.label} already running")
            return False
        
        try:
            self._pre_execute()
            
            self.process = subprocess.Popen(
                self.launch_command,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            print(f"[Process] Started: {self.label}")
            print(f"[Process] Command: {self.launch_command}")
            return True
        except Exception as e:
            print(f"[Process] Error starting {self.label}: {e}")
            return False
    
    def _pre_execute(self) -> None:
        """Hook for subclasses to perform actions before execution."""
        pass
    
    def stop(self) -> bool:
        """Stop the process. Returns True if successful."""
        if not self.is_running():
            return True
        
        try:
            self._pre_stop()
            
            if self.process and self.process.poll() is None:
                self.process.terminate()
                sleep(0.5)
                if self.process.poll() is None:
                    self.process.kill()
            
            print(f"[Process] Stopped: {self.label}")
            return True
        except Exception as e:
            print(f"[Process] Error stopping {self.label}: {e}")
            return False
        finally:
            self.process = None
    
    def _pre_stop(self) -> None:
        """Hook for subclasses to perform actions before stopping."""
        pass
    
    def is_running(self) -> bool:
        """Check if process is running."""
        return self.process is not None and self.process.poll() is None
    
    def toggle(self) -> bool:
        """Toggle the process on/off. Returns True if successful."""
        if self.is_running():
            return self.stop()
        else:
            return self.execute()
    
    def get_status_indicator(self) -> str:
        """Returns status indicator: ● running, ○ stopped"""
        return "●" if self.is_running() else "○"


class DockerProcess(Process):
    """
    Process that runs inside a Docker container.
    
    Attributes:
        container_name: Name for the Docker container
        image: Docker image to use
        flags: Additional docker run flags (e.g., --network host)
        gpu: Enable GPU passthrough (--gpus all)
        command: Command to run inside the container
    """
    
    def __init__(self, config: Dict[str, Any]):
        self.container_name = config.get('container_name', '')
        self.image = config.get('image', '')
        self.flags = config.get('flags', '')
        self.gpu = config.get('gpu', False)
        self.command = config.get('command', '')
        self.process_type = ProcessType.DOCKER
        super().__init__(config)
    
    def _build_command(self, config: Dict[str, Any]) -> None:
        """Build docker run command"""
        gpu_flag = "--gpus all" if self.gpu else ""
        self.launch_command = (
            f"docker run {gpu_flag} {self.flags} "
            f"--name {self.container_name} {self.image} {self.command}"
        ).strip()
        print(f"[DockerProcess] Loaded: {self.label} -> {self.launch_command}")
    
    def _pre_execute(self) -> None:
        """Remove existing container before starting"""
        if self.container_name:
            os.system(f"docker rm -f {self.container_name} 2>/dev/null")
    
    def _pre_stop(self) -> None:
        """Stop and remove the Docker container"""
        if self.container_name:
            os.system(f"docker stop {self.container_name} 2>/dev/null")
            os.system(f"docker rm -f {self.container_name} 2>/dev/null")
    
    def is_running(self) -> bool:
        """Check if container is running"""
        # First check subprocess
        if super().is_running():
            return True
        
        # Also check Docker container status
        if self.container_name:
            result = os.popen(f"docker ps -q -f name=^{self.container_name}$").read().strip()
            return len(result) > 0
        
        return False


class ROS2Process(Process):
    """
    Abstract base class for ROS2-specific processes.
    
    Provides common functionality for ROS2 CLI commands.
    """
    
    def __init__(self, config: Dict[str, Any]):
        self.package = config.get('package', '')
        self.executable = config.get('executable', '')
        self.arguments = config.get('arguments', '')
        super().__init__(config)


class NodeProcess(ROS2Process):
    """
    ROS2 Node process - launches nodes via ros2 run or ros2 launch.
    
    Attributes:
        package: ROS2 package name
        executable: Node executable or launch file name
        arguments: Additional arguments
        use_launch: If True, uses ros2 launch instead of ros2 run
    """
    
    def __init__(self, config: Dict[str, Any]):
        self.use_launch = config.get('use_launch', False)
        self.process_type = ProcessType.NODE
        super().__init__(config)
    
    def _build_command(self, config: Dict[str, Any]) -> None:
        """Build ros2 run or ros2 launch command"""
        if self.use_launch:
            self.launch_command = f"ros2 launch {self.package} {self.executable} {self.arguments}".strip()
        else:
            self.launch_command = f"ros2 run {self.package} {self.executable} {self.arguments}".strip()
        print(f"[NodeProcess] Loaded: {self.label} -> {self.launch_command}")


class ROSCommand(ROS2Process):
    """
    Generic ROS2 CLI command - for param set, topic pub, etc.
    
    Attributes:
        command: The full ros2 command to execute
    """
    
    def __init__(self, config: Dict[str, Any]):
        self.command = config.get('command', '')
        self.process_type = ProcessType.ROS_COMMAND
        super().__init__(config)
    
    def _build_command(self, config: Dict[str, Any]) -> None:
        """Use the provided command directly"""
        self.launch_command = self.command
        print(f"[ROSCommand] Loaded: {self.label} -> {self.launch_command}")
    
    def is_running(self) -> bool:
        """Most ROS commands are one-shot"""
        return super().is_running()


class BashCommand(Process):
    """
    Generic bash/shell command process.
    
    Attributes:
        command: Shell command to execute
        duration: Optional timeout in seconds (0 = no timeout)
    """
    
    def __init__(self, config: Dict[str, Any]):
        self.command = config.get('command', '')
        self.duration = config.get('duration', 0)
        self.process_type = ProcessType.COMMAND
        super().__init__(config)
    
    def _build_command(self, config: Dict[str, Any]) -> None:
        """Build bash command with optional timeout"""
        timeout_prefix = f"timeout {self.duration}" if self.duration > 0 else ""
        self.launch_command = f"{timeout_prefix} {self.command}".strip()
        print(f"[BashCommand] Loaded: {self.label} -> {self.launch_command}")


class ProcessFactory:
    """
    Factory class for creating Process instances from config.
    """
    
    @staticmethod
    def create(config: Dict[str, Any]) -> Optional[Process]:
        """
        Create appropriate Process subclass based on config type.
        
        Args:
            config: Dictionary with 'type' key and process-specific parameters
            
        Returns:
            Process instance or None if type is unknown
        """
        process_type = config.get('type', 'command')
        
        creators = {
            'docker': DockerProcess,
            'node': NodeProcess,
            'ros_command': ROSCommand,
            'command': BashCommand,
        }
        
        creator = creators.get(process_type)
        if creator:
            return creator(config)
        else:
            print(f"[ProcessFactory] Unknown process type: {process_type}")
            return None
