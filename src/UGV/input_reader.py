#!/usr/bin/env python3

##
# @file input_reader.py
#
# @brief Simple keyboard input reader
#
# @section libraries Libraries
# - pynput
#
# @section todo TODO 
# 

# Imports
from pynput.keyboard import Key, Listener

class InputReader:
    """
    @brief Class that reads inputs from a keyboard
    """

    def __init__(self):
        """
        Constructor for the InputReader class
        """
        self._pressed_keys = {}

        # Create a separate thread to listen for key presses
        self._listener = Listener(on_press=self._on_press, on_release = self._on_release)
        self._listener.start()

    def _on_press(self, key):
        """
        Callback function for when a key is pressed

        @key Key object of the key that was pressed
        """
        self._pressed_keys[key.char] = True
        
    def _on_release(self, key):
        """
        Callback function for when a key is released

        @key Key object of the key that was released
        """
        self._pressed_keys.pop(key.char, None)

    def get_pressed_keys(self):
        """
        Returns a list of all keys currently being pressed
        """
        return list(self._pressed_keys.keys())