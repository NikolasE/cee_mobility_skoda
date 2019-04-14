#! /usr/bin/python

import serial
import time
import readchar


dev = "/dev/serial/by-id/usb-Silicon_Labs_CP2104_USB_to_UART_Bridge_Controller_014A1259-if00-port0"
ser = serial.Serial(dev, 115200)  # open serial port

# for pos in range(800, 2200, 100):
#     print(pos)
#     ser.write('%i\n' % pos)
#     time.sleep(1)

step = 10
pos = 900

while True:
    print("Reading a char:")
    h = readchar.readchar()

    if h == 'q':
        break
    if h == 'A':
        pos += step
    if h == 'B':
        pos -= step
    if h == 'C':
        step = min(30, step + 1)
    if h == 'D':
        step = max(5, step - 1)

    pos = min(2400, max(800, pos))

    print("pos: %i, step: %i" % (pos, step))
    ser.write('%i\n' % pos)



ser.close()
