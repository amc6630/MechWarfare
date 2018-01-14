#! /usr/bin/python

# Jeremy Lim
# May 11, 2017
# This class acts as an interface to a pololu maestro servo controller.
# supports all versions, and has built in support for setting
# servo zero positions via a json config file.
# Dependencies: pyserial, numpy.

# Works on: Linux (Debian systems)
# This includes Raspian!

# This class requires some settings on the maestro:
# Baud rate: 115200 (8N1)
# Must be set to dual mode (recieve commands over usb serial)
# Min/Max period for each servo: 600/2400 (in microseconds)

import math
import numpy as np
import serial
import struct
import time
import json   # For reading initial configuration.

class MaestroInterface:
    
    def __init__(self,config_file="nope"):

        self.pololu = None # Serial connection to the servo controller.
        if config_file == "nope":
            # Default to maestro 6.
            self.servo_num = 6
            self.pololu_file = "/dev/ttyACM0" # This is set up for linux (debian) systems.
            self.baud_rate = 115200           # you can change this for a different baud rate.
            self.servo_positions = [1500,1500,1500,1500,1500,1500]         # list of all of the servo angles. Size should equal the number of servos.
            self.servo_offsets = [0, 0, 0, 0, 0, 0]           # list of all of the servo offsets. Small changes in duty cycle for calibration purposes.
        else:
            try:
                with open(config_file) as json_f:
                    json_thing = json.load(json_f)
                    self.servo_num = json_thing["servoNum"]
                    # only support 6, 12, 18, and 24 maestro
                    if self.servo_num != 6 and self.servo_num != 12 and self.servo_num != 18 and self.servo_num != 18:
                        raise Exception("Invalid Maestro Detected.") # lazy error handling.
                    # we're trusting that our offset list is the correct size...
                    self.pololu_file = json_thing["portname"]
                    self.baud_rate = json_thing["baudrate"]
                    self.servo_offsets = json_thing["offsetList"]
            except Exception as e:
                print "Json parse failure."
                exit()

    def open_connection(self):
        self.pololu = serial.Serial(self.pololu_file, self.baud_rate)
        # wait a bit for the chip to initialize
        time.sleep(0.5)
        #send an initialization byte.
        self.pololu.write(bytearray(struct.pack('B',0xAA)))

    def close_connection(self):
        self.pololu.close()

    # set pololu[servo_index] to be servo_input
    def send_single_command(self,servo_index, servo_input):
        if servo_index >= self.servo_num:
            raise Exception("Servo index out of bounds.")
        else:
            command_bytes = struct.pack('BBBB',0x84,servo_index,*self.get_duty_cycle(servo_input))
            pololu.write(command_bytes)

    # set pololu[servo_start_index...servo_start_index+num_servos-1] to be the values in command_list
    def send_multi_command(self,servo_start_index,num_servos,command_list):
        #if this is the micro maestro 6, must send 6 single commands.
        if servo_start_index >= self.servo_num or servo_start_index+num_servos >=self.servo_num or num_servos < 1:
            raise Exception("Servo index(es) out of bounds.")
        if len(command_list) != num_servos:
            raise Exception("Servo number & command mismatch.")

        command_bytes = []
        command_bytes.append(np.int8(0x9F))  # Command byte
        command_bytes.append(np.int8(num_servos))     # Number of channels we're controlling
        command_bytes.append(np.int8(servo_start_index))     # The start channel (the first one in this case)

        
        for a in range(0,num_servos):
            val = command_list[a]+self.servo_offsets[a + servo_start_index]
            if val > 2400:
                val = 2400
            elif val < 600:
                val = 600
            command_bytes += self.get_duty_cycle(val)

        self.pololu.write(bytearray(np.array(command_bytes)))
        
    def get_duty_cycle(self, in_micro_secs):
        if in_micro_secs > 2400 or in_micro_secs < 600:
            raise Exception("Servo command out of bounds(600-2400).")
        # Multiply by 4: the maestro operates in quarter-microseconds.
        duty = long(4 * in_micro_secs)
        # we send the high 7 bits and low 7 bits.
        # only take 14 bits of the number.
        low_bits = duty & 0x007F
        high_bits = (duty >> 7) & 0x7F
        # list of np.int8s
        byte_list = [np.int8(low_bits), np.int8(high_bits)]
        return byte_list
