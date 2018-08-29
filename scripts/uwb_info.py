#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
from uwb_uart.msg import Uwbdis
"""
MID
功能
消息 ID, 一共有三类,分别为 mr, mc, ma
mr 代表标签-基站距离(原生数据)
mc 代表标签-基站距离(优化修正过的数据,用于定位标签)
ma 代表基站-基站距离(修正优化过,用于基站自动定位)
MASK 表示 RANGE0, RANGE1, RANGE2, RANGE3 有哪几个消息是有效的;
例如: MASK=7 (0000 0111) 表示 RANGE0, RANGE1, RANGE2 都有效
RANGE0 如果 MID = mc 或 mr,表示标签 x 到基站 0 的距离,单位:毫米
RANGE1 如果 MID = mc 或 mr,表示标签 x 到基站 1 的距离,单位:毫米
如果 MID = ma,
RANGE2
如果 MID = mc 或 mr,表示标签 x 到基站 2 的距离,单位:毫米
如果 MID = ma,
RANGE3
表示基站 0 到基站 1 的距离,单位:毫米
表示基站 0 到基站 2 的距离,单位:毫米
如果 MID = mc 或 mr,表示标签 x 到基站 3 的距离,单位:毫米
如果 MID = ma,
表示基站 1 到基站 2 的距离,单位:毫米
NRANGES unit raw range 计数值(会不断累加)
RSEQ range sequence number 计数值(会不断累加)
DEBUG 如果 MID=ma,代表 TX/RX 天线延迟
aT:A T 是标签 ID,A 是基站 ID
此处提到的 ID 只是一个 short ID,完整的 ID 是 64 bit 的
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