import struct
import pygame
import socket

def main():
    #effectDist
    #walkHeight
    #liftHeight
    #l_sweep
    #l_extend
    #triangleExpand
    #max_speed
    #rad_max
    #aim_update_const
    #pitchMax
    #pitchMin
    #yawMax
    #yawMin
    while 1:
        controlSocket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        sendStr = [effectDist, walkHeight, liftHeight, l_sweep, l_extend, triangleExpand, max_speed,
        rad_max, aim_update_const, pitchMax, pitchMin, yawMax, yawMin]
        msgStr = struct.pack('Bbbbbbbbbbbbb', *sendStr)
        controlSocket.sendTo(msgStr, ('192.168.43.32', 8092))
    #while 1:
        #do stuff
    
if __name__=='__main__':
    main()