#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
import time
class TaskStateRead():
    def __init__(self):
       # self.nodename=nodename
        self.task_state_buf = []
    def Init_node(self):
        rospy.init_node("read_task_state_node")
        sub = rospy.Subscriber("/task_state", Bool, self.callback)
        return sub
    # def Count_100(self,temp):

    def callback(self,msg):
        # print msg
        if len(self.task_state_buf)==10:
            self.task_state_buf=self.task_state_buf[1:]
            self.task_state_buf.append(msg.data)
            #print "---------self.uvlist_buf",self.uvlist_buf

        else:
            self.task_state_buf.append(msg.data)
def main():
    uv0=TaskStateRead()
    uv0.Init_node()
    ratet = 1
    rate = rospy.Rate(ratet)
    while not rospy.is_shutdown():
        if len(uv0.task_state_buf)!=0:
            print "task_state_buf",uv0.task_state_buf[-1]

        #uv0.uvlist=[]
    #rospy.spin()
    rate.sleep()
if __name__=="__main__":
    main()