"""
Robot GP primitives and node value types.
"""
from enum import Enum, auto
from dataclasses import dataclass
from typing import Optional


class RobotCommand(Enum):
    """Robot command types for GP nodes."""
    # Function nodes
    PROGN3 = auto()     # Execute 3 commands in sequence
    PROGN2 = auto()     # Execute 2 commands in sequence
    IFWALL = auto()     # Execute left if near wall, right otherwise
    IFBALL = auto()     # Execute left if ball visible, right otherwise
    
    # Terminal nodes
    WALKFRONT = auto()  # Move forward
    WALKBACK = auto()   # Move backward
    LEFT = auto()       # Turn left
    RIGHT = auto()      # Turn right
    ALIGN = auto()      # Orient towards ball


@dataclass
class RobotNodeValue:
    """Node value that holds a robot command."""
    cmd: RobotCommand
    
    def children_count(self) -> int:
        """Get number of children required for this command."""
        if self.cmd == RobotCommand.PROGN3:
            return 3
        elif self.cmd in (RobotCommand.PROGN2, RobotCommand.IFWALL, RobotCommand.IFBALL):
            return 2
        return 0
    
    def is_function(self) -> bool:
        """Check if this is a function node."""
        return self.children_count() > 0
    
    def to_char(self) -> str:
        """Get character representation (for compatibility with existing code)."""
        char_map = {
            RobotCommand.PROGN3: '3',
            RobotCommand.PROGN2: '2',
            RobotCommand.IFWALL: 'I',
            RobotCommand.IFBALL: 'C',
            RobotCommand.WALKFRONT: 'F',
            RobotCommand.WALKBACK: 'B',
            RobotCommand.LEFT: 'L',
            RobotCommand.RIGHT: 'R',
            RobotCommand.ALIGN: 'A'
        }
        return char_map.get(self.cmd, '?')
    
    @classmethod
    def from_char(cls, c: str) -> Optional['RobotNodeValue']:
        """Create from character (for compatibility with existing code)."""
        cmd_map = {
            '3': RobotCommand.PROGN3,
            '2': RobotCommand.PROGN2,
            'I': RobotCommand.IFWALL,
            'C': RobotCommand.IFBALL,
            'F': RobotCommand.WALKFRONT,
            'B': RobotCommand.WALKBACK,
            'L': RobotCommand.LEFT,
            'R': RobotCommand.RIGHT,
            'A': RobotCommand.ALIGN
        }
        cmd = cmd_map.get(c)
        if cmd is not None:
            return cls(cmd)
        raise ValueError(f"Invalid robot command character: {c}")
    
    def __str__(self) -> str:
        """String representation of the node value."""
        return self.to_char()