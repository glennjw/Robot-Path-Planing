"""
Common utility functions.
"""

import random

from a5_value_it.msg import Position
from a5_value_it.msg import Move

def apply_move(pos, move):
    """Returns the resulting (x, y) tuple if the given move is applied.

    parameters:
      pos -- a Position, which is a message containing x and y int32 fields
      move -- a Move, which is a message containing a direction int32 field
    """
    
    x = pos.x
    y = pos.y
    if move.direction == 1:
        # East
        return x + 1, y
    elif move.direction == 2:
        # Northeast
        return x + 1, y + 1
    elif move.direction == 3:
        # North
        return x, y + 1
    elif move.direction == 4:
        # Northwest
        return x - 1, y + 1
    elif move.direction == 5:
        # West
        return x - 1, y
    elif move.direction == 6:
        # Southwest
        return x - 1, y - 1
    elif move.direction == 7:
        # South
        return x, y - 1
    elif move.direction == 8:
        # Southeast
        return x + 1, y - 1
    else:
        # Any other value (including 0) is treated as a null movement.
        return x, y

def apply_move_with_noise(pos, move):
    """With 50% probability a new random movement direction is chosen."""
    if random.random() > 0.5:
        move = Move(random.randint(1, 8))
    return apply_move(pos, move)
