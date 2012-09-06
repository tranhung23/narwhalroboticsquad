from visual import *
import string
import math
import socket
import struct
#import serial

#ser = serial.Serial(port='com7', baudrate=115200, timeout=1)

#dot = box(length=0.8, height=0.2, width=0.6, color=color.yellow)
dot = box(length=0.4, height=0.2, width=1.5, color=color.yellow)

xarrow = arrow(pos=(0,0,0), axis=(1,0,0), color=color.red)
yarrow = arrow(pos=(0,0,0), axis=(0,1,0), color=color.green)
zarrow = arrow(pos=(0,0,0), axis=(0,0,1), color=color.blue)
scene.forward = (1,-0.8,0)
scene.up=(0,0,1)
scene.lights = []
lamp=distant_light(direction=(-0.22,-0.44, 0.88), color=color.gray(0.6)) 
lamp=distant_light(direction=(0,1, 0), color=color.gray(0.8)) 
lamp=distant_light(direction=(-0.88, 0.22, 0.44), color=color.gray(0.3))


UDP_IP="127.0.0.1"
UDP_PORT=9091


sock = socket.socket( socket.AF_INET, # Internet
                          socket.SOCK_DGRAM ) # UDP
sock.bind( (UDP_IP,UDP_PORT) )

while 1:
      #line = ser.readline()
      #words = string.split(line)
    data, addr = sock.recvfrom( 200 )

    data = str( data, encoding='utf8' )
    words = data.split(' ')
    print(words)
    w11 = float(words[0])
    w21 = float(words[1])
    w31 = float(words[2])
    w12 = float(words[3])
    w22 = float(words[4])
    w32 = float(words[5])
    w13 = float(words[6])
    w23 = float(words[7])
    w33 = float(words[8])

    
##    axis=(w33, w23, w13)
##    up=(w31, w21, w11)

    axis=(w11, w21, w31)
    up=(w13, w23, w33)

    dot.axis=axis
    dot.up=up

    
    xarrow.axis=axis
    xarrow.up=up
    yarrow.axis=axis
    yarrow.up=up
    yarrow.rotate(angle=radians(90), axis=up)

    zarrow.axis=axis
    zarrow.up=up
    zarrow.rotate(angle=radians(90), axis=up)
    zarrow.rotate(angle=radians(90), axis=axis)
    
