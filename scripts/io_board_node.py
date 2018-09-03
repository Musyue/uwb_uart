#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Ioboard class.

python serial port code for 24 io board
the code '55C81900000055' means closing all io port
and the code '55C819FFFFFF55' means opening all io port
@author: Lzyue
"""
import rospy
from std_msgs.msg import String
import serial
from time import sleep
class Io_board():
    def __init__(self,nodename):
        self.nodename=nodename
        self.iostatebuff=[]
    def Init_node(self):
        rospy.init_node(self.nodename)
    def Io_callback(self,msg):
        print msg.data
        self.iostatebuff.append(msg.data)
    def Io_Sub(self,topicname):
        sub = rospy.Subscriber(topicname, String, self.Io_callback)
    def recv(self,serial):
        while True:
            data = serial.read_all()
            if data == '':
                continue
            else:
                break
            sleep(0.02)
        return data
def main():
    iob=Io_board("Io_board_node")
    iob.Init_node()
    iob.Io_Sub("io_state")
    import serial
    serial = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.5)  #/dev/ttyUSB0
    if serial.isOpen() :
        print("open port success")
    else :
        print("open port failed")
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        #data =recv(serial)
        if len(iob.iostatebuff)!=0:
            iocmd=iob.iostatebuff[-1]
            print("receive : ",iocmd)
            serial.write(iocmd.decode("hex")) #数据写回
        else:
            pass
        # data ='55C81900000055'
        rate.sleep()
if __name__ == '__main__':
    main()