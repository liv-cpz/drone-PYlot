#!/usr/bin/env python3

##
# @file controller.py
#
# @brief Simple controller implementation for the UGV02
#
# @section libraries Libraries
# - enum
# - serial_interface
#
# @section todo TODO 
# - Add different speed settings
# - Add serial port as an argument to constructor

# Imports
from enum import Enum
from serial_interface import SerialInterface

# Constant declarations
SPEED = 0.1
TURN_ADJUSTMENT = 0.05

# Enum declarations
MovementDirection = Enum('MovementDirection', ['FORWARD', 'NONE', 'BACK'])
TurnDirection = Enum('TurnDirection', ['LEFT', 'NONE', 'RIGHT'])

class Controller:
    """
    @brief Class that implements a simple controller for the UGV02
    """

    def __init__(self):
        """
        Constructor for the Controller class

        TODO Add serial port as an argument
        """

        # Establish serial connection
        self._serial = SerialInterface('/dev/ttyACM0')

        # Set initial velocity to zero
        self._movement = MovementDirection.FORWARD
        self._turn = TurnDirection.NONE

        # Disable continuous feedback
        self._serial.write(r"""{"T":131,"cmd":0}""")

    def change_movement_direction(self, direction : MovementDirection):
        """
        Updates the longitudinal movement direction of the UGV

        @param direction MovementDirection enum of the new movement direction
        """
        self._movement = direction

    def change_turn_direction(self, direction : TurnDirection):
        """
        Updates the turn direction of the UGV

        @param direction TurnDirection enum of the new turn direction
        """
        self._turn = direction
    
    def update_velocity(self):
        """
        Calculates the new velocity of the UGV and writes to serial
        """
        
        # Update longitudinal velocity
        match self._movement:
            case MovementDirection.FORWARD:
                left = SPEED
                right = SPEED
            case MovementDirection.NONE:
                left = 0.0
                right = 0.0
            case MovementDirection.BACK:
                left = -SPEED
                right = -SPEED
            case _:
                left = 0.0
                right = 0.0               
        
        # Update turn direction
        match self._turn:
            case TurnDirection.LEFT:
                left -= TURN_ADJUSTMENT
                right += TURN_ADJUSTMENT
            case TurnDirection.NONE:
                pass
            case TurnDirection.RIGHT:
                left += TURN_ADJUSTMENT
                right -= TURN_ADJUSTMENT
            case _:
                left = 0.0
                right = 0.0
            
        # Write velocity to serial in JSON format
        self._serial.write(r"""{"T":1,"L":""" f"{str(left)}" r""","R":""" f"{str(right)}" r"}")