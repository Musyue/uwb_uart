#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
from uwb_uart.msg import Uwbdis
"""
calibration
"""
import time
class Uwbdisinfo:
    def __init__(self,nodename):
        self.nodename=nodename
        self.pub=rospy.Publisher("uwb_dis_info",Uwbdis,queue_size=10)
    def Init_node(self):
        rospy.init_node(self.nodename)
    def uwbinfo_callback(self,msg):
        try:
            uwbinfo=Uwbdis()
            str=msg.data
            str_data=str.split("\r\n",2)[:2]
            mcdata=str_data[0].split(" ",9)
            mrdata=str_data[1].split(" ",9)
            uwbinfo.MID = mcdata[0]
            uwbinfo.T_A0 = int(mcdata[2],16)
            uwbinfo.T_A1 = int(mcdata[3], 16)
            uwbinfo.T_A2 = int(mcdata[4], 16)
            uwbinfo.T_A3 = int(mcdata[5], 16)
            uwbinfo.RangeId = int(mcdata[6], 16)
            uwbinfo.TimeId = int(time.time())
            uwbinfo.Unit = "mm"
            print mcdata
            self.pub.publish(uwbinfo)
        except:
            pass
    def Uwb_Sub(self,topicname):
        sub = rospy.Subscriber(topicname, String, self.uwbinfo_callback)
def main():
    uwb0=Uwbdisinfo("uwb_distance_info_get")
    uwb0.Init_node()
    uwb0.Uwb_Sub("uwb_read")
    rospy.spin()
if __name__ == '__main__':
    main()