#!/usr/bin/env python

import time
import serial

ser = serial.Serial(

    port="/dev/ttyAMA0",
    baudrate = 115200,
    parity = serial.PARITY_EVEN,
    stopbits = serial.STOPBITS_ONE,
    bytesize = serial.EIGHTBITS,
    timeout=1
    )


while 1:
    ser.write('start')
    print('I sent start')
    time.sleep(2)
    ser.write('backup')
    print('Backing up')
    time.sleep(2)
    ser.write('stop')
    print('I sent STOP')
    time.sleep(2)

