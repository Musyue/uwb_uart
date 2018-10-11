#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy,time
import os
import numpy
from std_msgs.msg import String
from std_msgs.msg import Bool
from frompitoangle import *
from ur5_kinematics import *
# from ur5_pose_get import *
# from transfer import *
from task_state_sub import *
import serial

import modbus_tk
import modbus_tk.defines as cst
from modbus_tk import modbus_rtu

class UrCircle:
    def __init__(self,weights,radius,cont,Port):
        self.weights = weights
        self.radius=radius#m
        self.cont=cont
        self.Port=Port
    def Init_node(self):
        rospy.init_node("move_ur5_circle")
        pub = rospy.Publisher("/ur_driver/URScript", String, queue_size=10)
        return pub
    def get_urobject_ur5kinetmatics(self):
        ur0 = Kinematic()
        return ur0
    def get_draw_circle_xy(self,t,xy_center_pos):
        x = xy_center_pos[0] + self.radius * math.cos( 2 * math.pi * t / self.cont )
        y = xy_center_pos[1] + self.radius * math.sin( 2 * math.pi * t / self.cont)
        return  [x,y]
    def get_draw_line_xy(self,t,xy_center_pos,flag):
        if flag==0:
            x = xy_center_pos[0] + self.radius  * t / self.cont
            y = xy_center_pos[1]
            return  [x,y]
        elif flag==1:
            x = xy_center_pos[0] - self.radius  * t / self.cont
            y = xy_center_pos[1]
            return  [x,y]
        else:
            pass
    def get_IK_from_T(self,T,q_last):
        ur0 = self.get_urobject_ur5kinetmatics()
        return ur0.best_sol(self.weights,q_last,T)
    def get_q_list(self,T_list,qzero):
        try:
            ur0 = self.get_urobject_ur5kinetmatics()
            tempq=[]
            resultq=[]
            for i in xrange(len(T_list)):
                if i==0:
                    tempq=qzero
                    firstq = ur0.best_sol(self.weights, tempq, T_list[i])
                    tempq=firstq
                    resultq.append(firstq.tolist())
                    # print "firstq", firstq
                else:
                    qq = ur0.best_sol(self.weights, tempq, T_list[i])
                    tempq=qq
                    # print "num i qq",i,qq
                    resultq.append(tempq.tolist())
            return resultq
        except:
            print "ur kinematics error"
    # T is numpy.array
    def get_T_translation(self, T):
        trans_x = T[3]
        trans_y = T[7]
        trans_z = T[11]
        return [trans_x, trans_y, trans_z]
    def insert_new_xy(self,T,nx,ny):
        temp=[]
        for i in xrange(12):
            if i==3:
                temp.append(nx)
            elif i==7:
                temp.append(ny)
            else:
                temp.append(T[i])
        return temp
    def get_circle_T(self,InitiT,xy_center_pos):
        temp=[]
        for i in xrange(self.cont):
            new_xy=self.get_draw_circle_xy(i,xy_center_pos)
            # new_xy = self.get_draw_line_xy(i, xy_center_pos)
            print "new_xy------------------",new_xy
            new_T=self.insert_new_xy(InitiT,new_xy[0],new_xy[1])
            #print "initial T\n",InitiT
            #print "new_T\n",i,new_T
            temp.append(new_T)
        return temp
    def get_line_T(self,InitiT,xy_center_pos):
        left=[]
        right=[]
        for i in xrange(2*self.cont):
            # new_xy=self.get_draw_circle_xy(i,xy_center_pos)
            if i< self.cont:
                new_xy = self.get_draw_line_xy(i, xy_center_pos,0)
                print "new_xy_left------------------",new_xy
                new_T=self.insert_new_xy(InitiT,new_xy[0],new_xy[1])
                #print "initial T\n",InitiT
                #print "new_T\n",i,new_T
                left.append(new_T)
            elif i>=self.cont:
                new_xy = self.get_draw_line_xy((i-self.cont), xy_center_pos,1)
                print "new_xy_right------------------",new_xy
                new_T=self.insert_new_xy(InitiT,new_xy[0],new_xy[1])
                #print "initial T\n",InitiT
                #print "new_T\n",i,new_T
                right.append(new_T)
        return left,right
    def urscript_pub(self, pub, qq, vel, ace, t):

        ss = "movej([" + str(qq[0]) + "," + str(qq[1]) + "," + str(qq[2]) + "," + str(
            qq[3]) + "," + str(qq[4]) + "," + str(qq[5]) + "]," + "a=" + str(ace) + "," + "v=" + str(
            vel) + "," + "t=" + str(t) + ")"
        print("---------ss:", ss)
            # ss="movej([-0.09577000000000001, -1.7111255555555556, 0.7485411111111111, 0.9948566666666667, 1.330836666666667, 2.3684322222222223], a=1.0, v=1.0,t=5)"
        pub.publish(ss)
    """
    flag=-1,upward
    flag=1,downward
    cont#负上,正下 77mm/300,每一个数量级相当于向上77/300mm
    """
    def Move_Upward_Motor(self,flag,cont):
        logger = modbus_tk.utils.create_logger("console")

        try:
            # Connect to the slave
            master = modbus_rtu.RtuMaster(
                serial.Serial(port=self.Port, baudrate=115200, bytesize=8, parity='O', stopbits=1, xonxoff=0)
            )
            master.set_timeout(5.0)
            master.set_verbose(True)
            logger.info("connected")
            #
            logger.info(master.execute(3, cst.READ_HOLDING_REGISTERS, 0, 8))
            # #change to SigIn SON enable driver
            output_value_0=flag*cont
            output_value_1 = flag * 5000
            logger.info(master.execute(3, cst.WRITE_SINGLE_REGISTER, 3, output_value=1))
            logger.info(master.execute(3, cst.WRITE_SINGLE_REGISTER, 109, output_value=2))
            logger.info(master.execute(3, cst.WRITE_SINGLE_REGISTER, 112, output_value=50))
            logger.info(master.execute(3, cst.WRITE_SINGLE_REGISTER, 117, output_value=1))
            logger.info(master.execute(3, cst.WRITE_SINGLE_REGISTER, 120, output_value=output_value_0))#负上,正下 77mm/300,每一个数量级相当于向上77/300mm
            logger.info(master.execute(3, cst.WRITE_SINGLE_REGISTER, 121, output_value=output_value_1))
            logger.info(master.execute(3, cst.WRITE_SINGLE_REGISTER, 128, output_value=250))
            logger.info(master.execute(3, cst.WRITE_SINGLE_REGISTER, 69, output_value=1024))
            logger.info(master.execute(3, cst.WRITE_SINGLE_REGISTER, 71, output_value=32767))
            time.sleep(1)
            logger.info(master.execute(3, cst.WRITE_SINGLE_REGISTER, 71, output_value=31743))
            logger.info(master.execute(3, cst.READ_HOLDING_REGISTERS, 0, 4))

        except modbus_tk.modbus.ModbusError as exc:
            logger.error("%s- Code=%d", exc, exc.get_exception_code())
    """
    flag=-1,anticlockwise
    flag=1,clockwise
    cont## 负逆时针90度/162,相当于每一个数量级，旋转90/162度
    """
    def Move_Rotation_Motor(self,flag,cont):
        logger = modbus_tk.utils.create_logger("console")

        try:
            # Connect to the slave
            master = modbus_rtu.RtuMaster(
                serial.Serial(port=self.Port, baudrate=115200, bytesize=8, parity='O', stopbits=1, xonxoff=0)
            )
            master.set_timeout(5.0)
            master.set_verbose(True)
            logger.info("connected")
            #
            logger.info(master.execute(2, cst.READ_HOLDING_REGISTERS, 0, 8))
            # #change to SigIn SON enable driver
            output_value_0=flag*cont
            output_value_1 = flag * 5000
            logger.info(master.execute(2, cst.WRITE_SINGLE_REGISTER, 3, output_value=1))
            logger.info(master.execute(2, cst.WRITE_SINGLE_REGISTER, 109, output_value=2))
            logger.info(master.execute(2, cst.WRITE_SINGLE_REGISTER, 112, output_value=50))
            logger.info(master.execute(2, cst.WRITE_SINGLE_REGISTER, 117, output_value=1))
            logger.info(master.execute(2, cst.WRITE_SINGLE_REGISTER, 120,output_value=output_value_0))  # 负逆时针90度/162,相当于每一个数量级，旋转90/162度，x*162/90
            logger.info(master.execute(2, cst.WRITE_SINGLE_REGISTER, 121, output_value=output_value_1))
            logger.info(master.execute(2, cst.WRITE_SINGLE_REGISTER, 128, output_value=100))
            logger.info(master.execute(2, cst.WRITE_SINGLE_REGISTER, 69, output_value=1024))
            logger.info(master.execute(2, cst.WRITE_SINGLE_REGISTER, 71, output_value=32767))
            time.sleep(1)
            logger.info(master.execute(2, cst.WRITE_SINGLE_REGISTER, 71, output_value=31743))
            logger.info(master.execute(2, cst.READ_HOLDING_REGISTERS, 0, 4))

        except modbus_tk.modbus.ModbusError as exc:
            logger.error("%s- Code=%d", exc, exc.get_exception_code())
    """
    back zero,move rotation and upward moto go back zero
    """
    def Move_Rotation_Upward_Motor(self,ro_flag,up_flag,rotation_cont,upward_cont):
        logger = modbus_tk.utils.create_logger("console")

        try:
            # Connect to the slave
            master = modbus_rtu.RtuMaster(
                serial.Serial(port=self.Port, baudrate=115200, bytesize=8, parity='O', stopbits=1, xonxoff=0)
            )
            master.set_timeout(5.0)
            master.set_verbose(True)
            logger.info("connected")
            #
            # logger.info(master.execute(2, cst.READ_HOLDING_REGISTERS, 0, 8))
            # #change to SigIn SON enable driver
            rotation_output_value_0=ro_flag*rotation_cont
            rotation_output_value_1 = ro_flag * 5000

            upward_output_value_0=up_flag*upward_cont
            upward_output_value_1 = up_flag * 5000

            logger.info(master.execute(2, cst.READ_HOLDING_REGISTERS, 0, 8))
            logger.info(master.execute(3, cst.READ_HOLDING_REGISTERS, 0, 8))
            #change to SigIn SON enable driver
            logger.info(master.execute(2, cst.WRITE_SINGLE_REGISTER, 3, output_value=1))
            logger.info(master.execute(3, cst.WRITE_SINGLE_REGISTER, 3, output_value=1))
            logger.info(master.execute(2, cst.WRITE_SINGLE_REGISTER, 109, output_value=2))
            logger.info(master.execute(3, cst.WRITE_SINGLE_REGISTER, 109, output_value=2))
            logger.info(master.execute(2, cst.WRITE_SINGLE_REGISTER, 112, output_value=50))
            logger.info(master.execute(3, cst.WRITE_SINGLE_REGISTER, 112, output_value=50))
            logger.info(master.execute(2, cst.WRITE_SINGLE_REGISTER, 117, output_value=1))
            logger.info(master.execute(3, cst.WRITE_SINGLE_REGISTER, 117, output_value=1))
            logger.info(master.execute(2, cst.WRITE_SINGLE_REGISTER, 120, output_value=rotation_output_value_0))#负逆时针
            logger.info(master.execute(3, cst.WRITE_SINGLE_REGISTER, 120, output_value=upward_output_value_0))#正的向下,负的向上
            logger.info(master.execute(2, cst.WRITE_SINGLE_REGISTER, 121, output_value=rotation_output_value_1))#负逆时针
            logger.info(master.execute(3, cst.WRITE_SINGLE_REGISTER, 121, output_value=upward_output_value_1))#正的向下,负的向上
            logger.info(master.execute(2, cst.WRITE_SINGLE_REGISTER, 128, output_value=100))
            logger.info(master.execute(3, cst.WRITE_SINGLE_REGISTER, 128, output_value=250))#下250,上500
            logger.info(master.execute(2, cst.WRITE_SINGLE_REGISTER, 69, output_value=1024))
            logger.info(master.execute(3, cst.WRITE_SINGLE_REGISTER, 69, output_value=1024))
            logger.info(master.execute(2, cst.WRITE_SINGLE_REGISTER, 71, output_value=32767))
            logger.info(master.execute(3, cst.WRITE_SINGLE_REGISTER, 71, output_value=32767))
            time.sleep(4)
            logger.info(master.execute(2, cst.WRITE_SINGLE_REGISTER, 71, output_value=31743))
            logger.info(master.execute(3, cst.WRITE_SINGLE_REGISTER, 71, output_value=31743))
            logger.info(master.execute(2, cst.READ_HOLDING_REGISTERS, 0, 4))

        except modbus_tk.modbus.ModbusError as exc:
            logger.error("%s- Code=%d", exc, exc.get_exception_code())
def main():
    t=0
    vel=0.1
    ace=50
    qstart=[90.68,-0.32,-82.49,-8.68,92.89,5.41]
        # [88,-2.06,-83.67,-1.06,86.53,5.41]
        # [90.68,-0.32,-82.49,-8.68,92.89,5.41]
    # qq=[45.91,-72.37,61.52,-78.56,-90.49,33.71]
    # q=display(getpi(qq))
    qstop= [88.31,-0.40,-160.64,77.43,85.36,27.03]
    ratet = 3
    radius=0.35
    weights = [1.] * 6
    cont=15
    PORT = "/dev/ttyUSB3"
    urc=UrCircle(weights,radius,cont,PORT)
    pub=urc.Init_node()
    rate = rospy.Rate(ratet)

    # first step go to initial pos
    q = display(getpi(qstart))
    q_stop=display(getpi(qstop))
    # urc.urscript_pub(pub,q,vel,ace,t)
    # second get T use urkinematics
    Task_state_read = TaskStateRead()
    read_sub = rospy.Subscriber("/task_state", Bool, Task_state_read.callback)


    urk = urc.get_urobject_ur5kinetmatics()
    F_T = urk.Forward(q)
    # print "F_T", F_T
    TransT = urc.get_T_translation(F_T)
    # print "TransT", TransT
    xy_center_pos = [TransT[0], TransT[1]]
    # print "xy_center_pos", xy_center_pos
    T_list_left,T_list_right = urc.get_line_T(F_T, xy_center_pos)
    # print "T_list", T_list
    # print "T_list_left,T_list_right ",T_list_left,"\n----------------",T_list_right
    reslut_q_left = urc.get_q_list(T_list_left, q)
    reslut_q_right = urc.get_q_list(T_list_right, q)
    print "reslut_q_right",reslut_q_right
    reslut_q_left=reslut_q_left+reslut_q_left[len(reslut_q_left)-1::-1]
    # reslut_q = reslut_q_right+reslut_q_left
    reslut_q_right = reslut_q_right + reslut_q_right[len(reslut_q_right) - 1::-1]
    # print "reslut_q", reslut_q
    cn_left=0
    cn_right=0
    left_flag_0=0
    right_flag_0=0
    turn_motor_upward=0
    cont_upward_motor=0
    turn_rotaion_flag=0
    task_over=0
    turn_on_motors=0
    air_output_flag=0
    while not rospy.is_shutdown():
        try:
            if len(Task_state_read.task_state_buf)!=0:
                if Task_state_read.task_state_buf[-1]==True:
                    if turn_on_motors == 0:
                        """
                        Open upward ,roation,uplink motor breaking
                        """
                        os.system('rostopic pub /io_state std_msgs/String "55C8190000F055" --once')
                        time.sleep(2)
                        turn_on_motors = 1
                    if task_over==0:
                        if turn_rotaion_flag==0:
                            urc.Move_Rotation_Motor(-1, 75)
                            time.sleep(2)

                        if reslut_q_left!=None and reslut_q_right !=None and turn_rotaion_flag==1:

                            if left_flag_0==0:
                                urc.urscript_pub(pub, reslut_q_left[cn_left], vel, ace, t)
                                cn_left+=1
                                if cn_left==len(reslut_q_left):
                                    cn_left=0
                                    left_flag_0=1
                                    right_flag_0=1
                                time.sleep(0.009)
                                if air_output_flag==0 and cn_left==38:
                                    os.system('rostopic pub /io_state std_msgs/String "55C8190008F055" --once')
                                    time.sleep(2)
                                    air_output_flag=1
                            if right_flag_0==1:
                                urc.urscript_pub(pub, reslut_q_right[cn_right], vel, ace, t)
                                cn_right += 1
                                if cn_right == len(reslut_q_right):
                                    cn_right = 0
                                    left_flag_0 = 0
                                    right_flag_0 = 0
                                    turn_motor_upward =1
                                print "cn_right-----\n,cn_right",reslut_q_right[cn_right],cn_right
                                time.sleep(0.009)

                        if turn_motor_upward==1 and cont_upward_motor<1200:
                            urc.Move_Upward_Motor(-1, 300)
                            time.sleep(5)
                            turn_motor_upward=0
                            cont_upward_motor+=300
                            if cont_upward_motor==1200:
                                # cont_upward_motor=0
                                turn_motor_upward=0
                                task_over=1
                        turn_rotaion_flag=1
                    else:
                        print "today task is over....welcome to next task-----------"
                    if task_over==1:
                        os.system('rostopic pub /io_state std_msgs/String "55C8190000F055" --once')
                        urc.Move_Rotation_Upward_Motor(1,1, 75,cont_upward_motor)#ro_flag,up_flag,rotation_cont,upward_cont
                        urc.urscript_pub(pub, q_stop, 0.2, ace, t)
                        time.sleep(10)
                        os.system('rostopic pub /io_state std_msgs/String "55C81900000055" --once')
                        task_over=3
                else:
                    print "Our smart car is running........,please wait--------------"
        except:
            print "demo is over-----------"
            # pass
        rate.sleep()
if __name__ == '__main__':
        main()