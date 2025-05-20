#!/usr/bin/env python3

##
# @file serial_interface.py
#
# @brief Implementation of a two-way serial connection
#
# @section libraries Libraries
# - serial
# - threading
#
# @section todo TODO 
# 

# Imports
import serial
import threading

class SerialInterface:
    """
    @brief Class that implements a two-way serial connection
    """
    def __init__(self, port : str):
        """
        Constructor for the SerialInterface class
        """

        # Create serial port
        self._ser = serial.Serial(port, baudrate=115200, dsrdtr=None)

        # Create a separate thread to read serial inputs
        self._serial_recv_thread = threading.Thread(target=self._read)
        self._serial_recv_thread.daemon = True
        self._serial_recv_thread.start()

    def _read(self):
        """
        Reads in the next line of serial data
        """
        while True:
            data = self._ser.readline().decode('utf-8')
            if data:
                print(f"Received: {data}", end='')

    def write(self, msg : str):
        """
        Writes a message to the serial port

        @param msg String of the message to send to serial
        """
        self._ser.write(msg.encode() + b'\n')