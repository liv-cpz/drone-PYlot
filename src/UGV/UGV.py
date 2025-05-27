#!/usr/bin/env python3

##
# @file UGV.py
#
# @brief Main loop that allows for controlling a UGV05
#
# @section libraries Libraries
# - controller
# - input_reader
#
# @section todo TODO 
# - Add serial port as a command line argument

# Imports
from controller import Controller, MovementDirection, TurnDirection
from input_reader import InputReader

def main():
    """
    Main loop that allows for controlling a UGV05
    """

    controller = Controller() # TODO Add argument for serial port
    inputs = InputReader()

    # Main loop
    while True:

        # Get all keys being pressed
        pressed_keys = inputs.get_pressed_keys()
        
        # XOR for forward velocity
        if ('w' in pressed_keys) ^ ('s' in pressed_keys):
            if 'w' in pressed_keys:
                controller.change_movement_direction(MovementDirection.FORWARD)
            else:
                controller.change_movement_direction(MovementDirection.BACK)
        else:
            controller.change_movement_direction(MovementDirection.NONE)

        # XOR for turn direction
        if ('a' in pressed_keys) ^ ('d' in pressed_keys):
            if 'a' in pressed_keys:
                controller.change_turn_direction(TurnDirection.LEFT)
            else:
                controller.change_turn_direction(TurnDirection.RIGHT)
        else:
            controller.change_turn_direction(TurnDirection.NONE) 

        # Send new velocity to the UGV
        controller.update_velocity()       

if __name__ == '__main__':
    main()