
import serial
from is_msgs.common_pb2 import Pose, Speed
from is_msgs.robot_pb2 import RangeScan
from copy import copy

import Robot_pb2

class ATSPdriver:

    pos = Pose()
    sp = Speed()
    ultsom = RangeScan()
    sp_rec = Speed()
    ser = serial.Serial()

    pckin = Robot_pb2.Odometry()
    pckout = Robot_pb2.PIDs()

    def __init__(self, port):       
        self.ser = serial.Serial(port, 115200, timeout=1)  # open serial port 115200
        self.ultsom.angles.append(45.0)
        self.ultsom.angles.append(90.0)
        self.ultsom.angles.append(135.0)

    def __del__(self):
        self.stop()
        self.ser.close()
       

    def send_speed(self):
        self.pckout.velLin = (int)(self.sp.linear*1000)
        self.pckout.velAng = (int)(self.sp.angular*1000)
        tx_buf = self.pckout.SerializeToString()
        # Envia pela serial
        self.ser.write(b'#')
        self.ser.write(b'#')
        self.ser.write(b'*')
        self.ser.write(bytes([len(tx_buf)]))
        self.ser.write(tx_buf)
        tx_buf = bytearray(64)

    def get_odometry(self):
        rx_buf = []
        # recebe da serial
        self.ser.write(b'#')
        self.ser.write(b'#')
        self.ser.write(b'#')

        ch = self.ser.read(1)
        for x in range(int.from_bytes(ch, byteorder='little')):
            ch = self.ser.read(1)
            rx_buf.append(ch)

        sin = bytes.join(b'', rx_buf)
        rx_buf.clear()
        self.pckin.ParseFromString(sin)

        self.pos.position.x = self.pckin.pos_x/1000.0
        self.pos.position.y = self.pckin.pos_y/1000.0
        self.pos.orientation.roll = self.pckin.heading/1000.0
        self.ultsom.ranges.append(self.pckin.dist_US_Esquerdo)
        self.ultsom.ranges.append(self.pckin.dist_US_Frente)
        self.ultsom.ranges.append(self.pckin.dist_US_Direito)
        self.sp_rec.linear = self.pckin.vel_linear/1000.0
        self.sp_rec.angular = (self.pckin.vel_angular/1000.0)*3.14/180

    def stop(self):
        self.sp.linear = 0.001
        self.sp.angular = 0.001
        self.send_speed()

    def get_speed(self):
        self.get_odometry()
        return self.sp_rec

    def set_speed(self, sp_new):
        self.sp.linear = sp_new.linear
        self.sp.angular = sp_new.angular
        self.send_speed()

    def get_pose(self):
        self.get_odometry()
        return self.pos

    def get_sonar_scan(self):
        self.get_odometry()
        tmp = copy(self.ultsom)
        self.ultsom.ranges.pop()
        self.ultsom.ranges.pop()
        self.ultsom.ranges.pop()
        return tmp