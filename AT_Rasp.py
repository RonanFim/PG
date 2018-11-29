#! /usr/bin/python

# To stop getty:
# sudo systemctl stop serial-getty@ttyAMA0.service

# Problemas de permissao:
# sudo chmod a+rw /dev/ttyAMA0


import threading
import serial
from time import sleep
from driver import ATSPdriver
from is_msgs.common_pb2 import Pose, Speed
from is_msgs.robot_pb2 import RangeScan


port = '/dev/ttyAMA0'

ps = Pose()
sp = Speed()
sp2 = Speed()
rs = RangeScan()

try:
    robo = ATSPdriver(port)
except serial.SerialException:
    print("Erro ao abrir Serial ", port)

  
print("Inicio!")

sleep(1)

sp.linear = 100.0
sp.angular = 0.0

robo.set_speed(sp)

for x in range(150):

    sleep(0.075)
    
    # ps = robo.get_pose()
    # print("pos_x: ",ps.position.x)
    # print("pos_y: ",ps.position.y)
    # print("heading: ",ps.orientation.roll)

    # rs = robo.get_sonar_scan()
    # print("esquerda: ",rs.ranges[0])
    # print("meio: ",rs.ranges[1])
    # print("direita: ",rs.ranges[2])

    sp2 = robo.get_speed()
    print("linear: ",sp2.linear)
    print("angular: ",sp2.angular)

    print(" ")

    # robo.set_speed(sp)
    # sp.linear += 0.005
    # sp.angular += 0.0005

robo.stop()

del robo

print('fim!')