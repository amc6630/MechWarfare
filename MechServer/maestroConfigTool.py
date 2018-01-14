#! /usr/bin/python

# Jeremy Lim
# May 11, 2017
# This script will act as a quick config tool, allowing you to setup
# a config file for the maestro interface, by connecting and commanding the
# servos live. This should be used for fine calibrations in zeroing out servos.

# Dependencies: serial, numpy

# This depends on some maestro settings:
# Serial settings: 8N1 (baud rate can be varied)
# Maestro must be set to dual mode (to receive commands over serial)
# Min/Max Period for each servo: 600/2400 (in microseconds)

# Works on: Linux (Debian systems)
# This includes Raspian!

import math
import numpy as np
import serial
import struct
import time

import json
import os


def get_duty_cycle(in_micro_secs):
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

def dispatch_commands(pololu,servo_commands):
    for b in range(0,len(servo_commands)):
        command_bytes = [np.int8(0x84), np.int8(b)]
        command_bytes += get_duty_cycle(servo_commands[b])
        pololu.write(bytearray(np.array(command_bytes)))

def main():
    oldfile = False

    save_file = ""
    dev_file = ""
    baud = 0
    servo_num = 0
    #offsetL = None
    json_obj = {}
    servo_com = []

    input_str = "y"
    input_str = raw_input("(R)ead existing file, (N)ew file: ")
    if input_str.lower() == "r":
        oldfile = True
        input_str = raw_input("Enter Filename: ")
        with open(input_str) as jsonFile:
            save_file = input_str
            json_obj = json.load(jsonFile)
            dev_file = json_obj["portname"]
            #baud = json_thing["baudrate"]
            servo_num = json_obj["servoNum"]
            for c in range(0,servo_num):
                servo_com.append(1500+json_obj["offsetList"][c])
            
    else:
        input_str = raw_input("Enter Filename: ")
        save_file = input_str
        input_str = raw_input("Enter maestro device directory: ")
        #dev_file = input_str
        json_obj["portname"] = input_str
        dev_file = json_obj["portname"]
        input_str = raw_input("Enter Baud Rate: ")
        #baud = int(input_str)
        json_obj["baudrate"] = int(input_str)
        input_str = raw_input("Enter Servo Count: ")
        #servo_num = int(input_str)
        json_obj["servoNum"] = int(input_str)
        servo_num = json_obj["servoNum"]

        json_obj["offsetList"] = []
        for c in range(0,servo_num):
            servo_com.append(1500)
            json_obj["offsetList"].append(0)

    pololu = serial.Serial(dev_file,baud)
    servIndex = 0

    pololu.write(bytearray(struct.pack('B',0xAA)))
    time.sleep(0.5)

    while input_str != "q":
        input_str = raw_input("(Q)uit, (S)et servo index, enter offset value.")
        if str(input_str).lower() == "s":
            input_str = raw_input("Enter new index: ")
            servIndex = int(input_str)
            print("servo index changed to " + str(servIndex))
        elif str(input_str).lower() != "q":
            offsetNum = int(input_str)
            print("Offset for servo # " + str(servIndex) + " is: " + str(offsetNum))
            servo_com[servIndex] = 1500 + offsetNum
            json_obj["offsetList"][servIndex] = offsetNum

        dispatch_commands(pololu,servo_com)   # write commands to serial.


    print("Saving config settings to " + save_file)
    if oldfile:
        os.remove(save_file)

    with open(save_file,'w') as outFile:
        json.dump(json_obj,outFile,indent=4)


    pololu.close()
        

if __name__ == '__main__': main()
