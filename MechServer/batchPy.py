import os

def byte(text):
    return "printf \"\\x$(printf \"%s\" $1)\""%(text)


if(False):
    lst = []
    for x in lst:
        os.system("byte " + x + " > " + Device)

else:
    Device  = "/dev/ttyACM0"
    powers = [1500,1500,800]
    start = 3
    os.system(byte ("0x9F") + " > " + Device)
    os.system(byte( str(len(powers)) ) + " > " + Device)
    os.system(byte( str(start) ) + " > " + Device)

    for x in powers:
        item = x
        
        os.system(byte ( str(item & 0x7F) ) + " > " + Device)
        os.system(byte (str(item >> 7 & 0x7F) )+ " > " + Device)




