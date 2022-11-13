import oneWire
import time
import struct

onew = oneWire.DS2480B('COM15')

if not onew.DS2480B_Detect():
    print("DS2480B not detected...")
    exit(-1)

print("DS2480B connected...")
print("Scanning 1-wire network...")

rslt = onew.OWFirst()
num = 1
while rslt:
    print("{:03d}: ".format(num), end='')
    num += 1
    onew.printROM()
    rslt = onew.OWNext()

print("Finished scanning 1-wire network")

print("Assume that the only device is DS18B20 - try to read temp!")

if(onew.OWFirst()):
    if onew.ROM_NO[0] != 0x28:
        print("This is not 18B20...")
    else:
        id = list(onew.ROM_NO)
        onew.OWReset()
        onew.OWBlock([0x55] + id)  # match rom (0x55) + ROM_ID
        onew.OWWriteBytePower(0x44)  # convert_t (0x44) and power the chip
        time.sleep(1)
        onew.OWLevel(onew.MODE_NORMAL)  # disable power
        onew.OWReset()
        onew.OWBlock([0x55] + id + [0xBE])  # match rom (0x55) + ROM_ID + read SPAD (0xBE)
        spad = onew.OWReadBlock(9)
        for i in range(9):
            print("{:02X}".format(spad[i]), end=' ')
        print()
        # calculate temperature - account for negative sign
        if spad[1] & 0b11111000 != 0:
            sign = -1
        else:
            sign = 1
        spad[1] &= 0b00000111
        t = (spad[1] * 256 + spad[0]) / 16.0 * sign  # [degC]
        print("T = {:09.5f}".format(t))