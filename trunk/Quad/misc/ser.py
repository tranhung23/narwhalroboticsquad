from visual import *
import string
import math
import socket
import struct
import serial

ser = serial.Serial(port='com7', baudrate=115200, timeout=1)
while 1:
    line = ser.readline()
    words = line.split(' ')
    print(words[0])
    print(words[1])
    print(words[2])
    print(words[3])
ser.close
