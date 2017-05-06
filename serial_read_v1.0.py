#!/usr/bin/env python

import time
import serial



from gpiozero import PWMOutputDevice

ser = serial.Serial(

    port='/dev/ttyAMA0',
    baudrate = 115200,
    parity = serial.PARITY_EVEN,
    stopbits = serial.STOPBITS_ONE,
    bytesize = serial.EIGHTBITS,
    timeout = 1
    )

a = PWMOutputDevice(4)

while 1:
    string = ser.readline()

    if string == 'start':
        print('.. asked to start')
        a.on()
        a.value = .2
    else:
        if string == 'stop':
            a.off()
            print('okay I will stop...')
        else:
            if string == 'backup':
                print('backing up, beep beep!')
                a.on()
                a.value = .01        
