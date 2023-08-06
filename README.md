#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from pickletools import pyfrozenset
import time
import random
import ctypes
from turtle import position
import roslib
import rospy
import smach
import smach_ros
import threading
import string
import math
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from spark_carry_object.msg import *
x = 0
y = 0
z = 0


class GraspObject():
    '''
    监听主控，用于物品抓取功能
    '''

    def __init__(self):
        '''
        初始化
        '''

        global xc, yc, xc_prev, yc_prev, found_count, can
        xc = 0
        yc = 0
        xc_prev = xc
        yc_prev = yc
        found_count = 0
        self.is_found_object = False
        # self.sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_cb, queue_size=1)
        # 订阅机械臂抓取指令
        self.sub2 = rospy.Subscriber(
            '/grasp', String, self.grasp_cp, queue_size=1)
        # 发布机械臂位姿
        self.pub1 = rospy.Publisher(
            'position_write_topic', position, queue_size=1)
        # 发布机械臂吸盘
        self.pub2 = rospy.Publisher('pump_topic', status, queue_size=1)
        # 发布机械臂状态
        self.grasp_status_pub = rospy.Publisher(
            'grasp_status', String, queue_size=1)
        # 发布TWist消息控制机器人底盘
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        
        # pos = position()
        # pos.x = 120
        # pos.y = 0
        # pos.z = 35
        # self.pub1.publish(pos)


        

    def grasp_cp(self,msg):
        if msg.data == "J" or msg.data == 'K' or msg.data == 'L':
            # 订阅摄像头话题,对图像信息进行处理
            # self.sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_cb, queue_size=1)
            # self.is_found_object = False
            # rate = rospy.Rate(10)
            # times=0
            # steps=0
            # while not self.is_found_object:
            #     rate.sleep()
            #     times+=1
                # 转一圈没有发现可抓取物体,退出抓取
                # if steps>=5:
                #     self.sub.unregister()
                #     print("stop grasp\n")
                #     status=String()
                #     status.data='-1'
                #     self.grasp_status_pub.publish(status)
                #     return
                # 旋转一定角度扫描是否有可供抓取的物体
                # if times>=30:
                #     times=0
                #     steps+=5
                #     #self.turn_body()
                #     print("not found\n")
            # print("unregisting sub\n")
            # self.sub.unregister()
            # print("unregisted sub\n")
            # 抓取检测到的物体    
            # mag_msg = msg.data
            # print(mag_msg)
            # self.grasp(mag_msg)
            # status=String()
            # status.data='1'
            # self.grasp_status_pub.publish(status) 
            pos = position()
            global x, y, z
            pos.x = 195
            x = pos.x
            pos.y = 0
            pos.z = -47
            self.pub1.publish(pos)
            rospy.sleep(0.5)
            self.pub2.publish(1)
            rospy.sleep(0.5)
            pos = position()
            if msg.data == "J":
                pos.z = -20
                z = pos.z
                x = 180
                self.pub1.publish(180,0,pos.z)
            elif msg.data == "K":
                pos.z = 85
                z = pos.z
                x = 180 
                self.pub1.publish(180,0,pos.z)
            elif msg.data == "L":
                pos.z = 170
                z = pos.z
                x = 195
                self.pub1.publish(195,0,pos.z)
            rospy.sleep(0.5)
            self.is_found_object = False

        elif msg.data == "1" or msg.data == "2" or msg.data == "3" or msg.data=='Y' :
            pos = position()
            # global x, y, z
            pos.x = 195
            x = pos.x
            pos.y = 0
            pos.z = -85.
            self.pub1.publish(pos)
            rospy.sleep(0.5)
            self.pub2.publish(1)
            rospy.sleep(0.75)
            pos = position()
            if msg.data == "1":
                pos.z = -15
                z = pos.z
                x = 180
                self.pub1.publish(180,0,pos.z)
            elif msg.data == "2":
                pos.z = 65
                z = pos.z
                x = 180 
                self.pub1.publish(180,0,pos.z)
            elif msg.data == "3":
                pos.z = 155
                z = pos.z
                x = 195
                self.pub1.publish(195,0,pos.z)
            elif msg.data == "Y":
                pos.z = 10
                z = pos.z
                x = 180
                self.pub1.publish(180,0,pos.z)
            rospy.sleep(0.5)
            self.is_found_object = False

        elif msg.data == "h":
            # 放下物体
            # mag_msg = msg.data
            # print(mag_msg)
            self.is_found_object = False
            # #self.release_object(mag_msg)
            self.pub2.publish(0)
            # rospy.sleep(1.2)
            # status=String()
            # status.data='0'
            # self.grasp_status_pub.publish(status) 
        
        elif msg.data == "H" :
            # pos = position()
            # pos.x = x
            # pos.y = y
            # z = z + 10
            # pos.z = z
            x = 250
            y = 0
            z = 80 
            self.pub1.publish(250,0,80)
            rospy.sleep(0.1)
            self.is_found_object = False


        elif msg.data == "q":
            # pos = position()
            # pos.x = x
            # pos.y = y
            # z = z - 10
            # pos.z = z
            x = 180
            y = 0
            z = 80
            self.pub1.publish(180,0,80)
            rospy.sleep(0.1)
            self.is_found_object = False


        elif msg.data == "e":
            # pos = position()
            # x = x + 10
            # pos.x = x
            # pos.y = y
            # pos.z = z
            x = 195
            y = 0
            z = 170
            self.pub1.publish(195,0,170)
            rospy.sleep(0)
            self.is_found_object = False


        elif msg.data == "f":
        #     pos = position()
        #     x = x - 10
        #     pos.x = x
        #     pos.y = y
        #     pos.z = z
            global can
            self.pub2.publish(1)
            self.pub1.publish(200,0,-125)
            rospy.sleep(0.1)
            self.pub1.publish(200,0,170)
            can = 1
            x = 200
            y = 0
            z = 170
            self.is_found_object = False

        elif msg.data == "4":
            # self.pub2.publish(1)
            # self.pub1.publish(150,0,-47)
            # rospy.sleep(0.1)
            # self.pub1.publish(180,0,-20)
            pos = position()
            z = z + 10
            pos.x = x
            pos.y = y
            pos.z = z
            # rospy.sleep(0.1)
            self.pub1.publish(pos)
            self.is_found_object = False

        elif msg.data == "6":
            pos = position()
            z = z - 10
            pos.x = x
            pos.y = y
            pos.z = z
            # rospy.sleep(0.1)
            self.pub1.publish(pos)
            self.is_found_object = False

        elif msg.data == "8":
            pos = position()
            x = x + 10
            pos.x = x
            pos.y = y
            pos.z = z
            # rospy.sleep(0.1)
            self.pub1.publish(pos)
            self.is_found_object = False

        elif msg.data == "5":
            pos = position()
            x = x - 10
            pos.x = x
            pos.y = y
            pos.z = z
            # rospy.sleep(0.1)
            self.pub1.publish(pos)
            self.is_found_object = False

        elif msg.data == "7":
            self.pub2.publish(1)
            self.is_found_object = False

        elif msg.data == "9":
            self.pub2.publish(0)
            self.is_found_object = False
            y = 0

        # elif msg.data == "7":
        #     pos = position()
        #     pos.x = 120
        #     x = pos.x
        #     pos.y = 0
        #     pos.z = 65
        #     self.pub1.publish(pos.x,pos.y,pos.z)
        #     rospy.sleep(0.1)
        #     self.is_found_object = False


        # elif msg.data == "9":
            # pos = position()
            # x = x - 3
            # pos.x = x
            # y = y - 10
            # pos.y = y
            # pos.z = z
            # self.pub1.publish(pos.x,pos.y,pos.z)
            # rospy.sleep(0.1)
            # self.pub2.publish(1)
            # self.is_found_object = False 

        # elif msg.data == ".":
            # self.pub2.publish(1)
            # self.pub2.publish(0)
            # pos = position()
            # pos.x = 200
            # x = pos.x
            # pos.y = 0
            # pos.z = -20
            # self.pub1.publish(pos.x,pos.y,pos.z)
            # rospy.sleep(0.1)
            # self.is_found_object = False


        elif msg.data == "0":
            self.pub2.publish(0)
            self.is_found_object = False
            y = 0

        # elif msg.data == "F":
        #     pos = position()
        #     y = y + 5
        #     pos.x = x
        #     pos.y = y
        #     y = pos.y
        #     pos.z = z
        #     self.pub1.publish(pos.x,pos.y,pos.z)
        
        # elif msg.data == "G":
        #     pos = position()
        #     y = y - 5
        #     pos.x = x
        #     pos.y = y
        #     y = pos.y
        #     pos.z = z
        #     self.pub1.publish(pos.x,pos.y,pos.z)
        
    # 执行抓取
    def grasp(self,msg_data):
        print("start to grasp\n")
        global xc, yc, found_count
        # stop function

        filename = os.environ['HOME'] + "/thefile.txt"
        file_pix = open(filename, 'r')
        s = file_pix.read()
        file_pix.close()
        print(s)
        arr=s.split()
        a1=arr[0]
        a2=arr[1]
        a3=arr[2]
        a4=arr[3]
        a = [0]*2
        b = [0]*2
        a[0]=float(a1)
        a[1]=float(a2)
        b[0]=float(a3)
        b[1]=float(a4)
        print('k and b value:',a[0],a[1],b[0],b[1])
        r1 = rospy.Rate(0.095)
        r2 = rospy.Rate(10)
        pos = position()
        # 物体所在坐标+标定误差
        pos.x = 200#a[0] * yc + a[1]
        pos.y = 0#b[0] * xc + b[1]
        pos.z = 20
	    # pos.z = 20
        print("z = 20\n")
        self.pub1.publish(pos)
        r2.sleep()
        # go down -100 
        pos.z = -48
        self.pub1.publish(pos)
        print("z = -83\n")
        r2.sleep()

        # 开始吸取物体
        self.pub2.publish(1)
        r2.sleep()
        #print(msg_data)
        # 提起物体
    
        pos = position()
        if msg_data == "J":
            pos.z = -20

        elif msg_data=="K":
            pos.z = 85 

        elif msg_data== "L":
            pos.z = 160
        # go forward
        self.pub1.publish(150,0,pos.z)
        rospy.sleep(0.5)

        #pos.x = 200  #160
        #pos.y = 0
        #pos.z = 170  #55
        self.pub1.publish(pos)
        r1.sleep()

        
    # 使用CV检测物体       
    def image_cb(self, data):
        global xc, yc, xc_prev, yc_prev, found_count
        # change to opencv
        try:
            cv_image1 = CvBridge().imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print('error')

        # change rgb to hsv
        cv_image2 = cv2.cvtColor(cv_image1, cv2.COLOR_BGR2HSV)

        # 蓝色物体颜色检测范围
        LowerBlue = np.array([95, 90, 80])
        UpperBlue = np.array([130, 255, 255])
        mask = cv2.inRange(cv_image2, LowerBlue, UpperBlue)
        cv_image3 = cv2.bitwise_and(cv_image2, cv_image2, mask=mask)

        # gray process
        cv_image4 = cv_image3[:, :, 0]

        # smooth and clean noise
        blurred = cv2.blur(cv_image4, (9, 9))
        (_, thresh) = cv2.threshold(blurred, 90, 255, cv2.THRESH_BINARY)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (25, 25))
        cv_image5 = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
        cv_image5 = cv2.erode(cv_image5, None, iterations=4)
        cv_image5 = cv2.dilate(cv_image5, None, iterations=4)

        # detect contour
        # cv2.imshow("win1", cv_image1)
        # cv2.imshow("win2", cv_image5)
        # cv2.waitKey(1)
        contours, hier = cv2.findContours(cv_image5, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # if find contours, pick the biggest box
        if len(contours) > 0:
            size = []
            size_max = 0
            for i, c in enumerate(contours):
                rect = cv2.minAreaRect(c)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                x_mid = (box[0][0] + box[2][0] + box[1][0] + box[3][0]) / 4
                y_mid = (box[0][1] + box[2][1] + box[1][1] + box[3][1]) / 4
                w = math.sqrt((box[0][0] - box[1][0]) ** 2 + (box[0][1] - box[1][1]) ** 2)
                h = math.sqrt((box[0][0] - box[3][0]) ** 2 + (box[0][1] - box[3][1]) ** 2) 
                size.append(w * h)
                if size[i] > size_max:
                    size_max = size[i]
                    index = i
                    xc = x_mid
                    yc = y_mid
            # if box is not moving for 20 times
            # print found_cont
            if found_count >= 30:
                self.is_found_object = True
                cmd_vel = Twist()
                self.cmd_vel_pub.publish(cmd_vel)
            else:
                # if box is not moving
                if abs(xc - xc_prev) <= 2 and abs(yc - yc_prev) <= 2:
                    found_count = found_count + 1
                else:
                    found_count = 0
        else:
            found_count = 0
        xc_prev = xc
        yc_prev = yc


    # 释放物体
    #def release_object(self):

        #pos = position()
        #if msg_data == "2":

         #   pos.z = -20

        #elif msg_data=="5":
          #  pos.z = 80 

        #elif msg_data == "8":
         #   pos.z = 170
        # go forward
        #self.pub1.publish(200,0,pos.z)
        #rospy.sleep(0.3)

    # stop pump
        #self.pub2.publish(0)
        #
        #return 'succeeded'

    # 转动机器人到一定角度       
    def turn_body(self):
        cmd_vel = Twist()
        cmd_vel.angular.z = 0.25
        rate = rospy.Rate(10)
        for i in range(40):            
            self.cmd_vel_pub.publish(cmd_vel)            
            rate.sleep()
       

        
if __name__ == '__main__':
    try:
        rospy.init_node('GraspObject', anonymous=False)
        rospy.loginfo("Init GraspObject main")   
        GraspObject()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("End spark GraspObject main")

