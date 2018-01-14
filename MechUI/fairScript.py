#! /usr/bin/python

#jeremy lim

#script that spoofs the mech's control
#does random animations.

import time
import socket

import numpy as np

def main():
    stateNum = 0
    count = 0
    while 1:

        if count < 180:
            count = count + 1
        else:
            #switch state
            count = 0
            if stateNum == 0:
                stateNum = 1
            elif stateNum == 1:
                stateNum = 2
            elif stateNum == 2:
                stateNum = -1
            else:
                stateNum = 0

        #determine values
        if stateNum == 1:
            rotation = 25
        elif stateNum == -1:
            rotation = -25
        else:
            rotation = 0
        
        xMove = 0
        yMove = 0


        #write message
        toggleValue = np.int8(0)
        sendStr = []
        sendStr.append(np.int8(212))
        sendStr.append(np.int8(xMove))
        sendStr.append(np.int8(yMove))
        sendStr.append(np.int8(rotation))
        sendStr.append(toggleValue)
        #might be unneeded.
        writeStr = np.array(sendStr)
        msgStr = writeStr.tostring()

        #use sendto to send control info.
        #(host, portnum) format for address.
        controlSocket.sendto(msgStr, ('127.0.0.1',8090))

        time.sleep(1.0/60.0)

if __name__ == '__main__': main()
