#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from copy import deepcopy
import os
import rospy
import math
import cmath
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import Image,LaserScan
from cv_bridge import CvBridge, CvBridgeError
from spark_carry_object.msg import *
from matplotlib import pyplot as plt


class GraspObject():
    '''
    监听主控，用于物品抓取功能
    '''
    image_last_time = None

    def __init__(self):
        '''
        初始化
        '''
        self.xc = 0
        self.yc = 0
        self.xc_prev = 0
        self.yc_prev = 0
        self.found_count = 0
        self.is_have_object = False
        self.is_found_object = False
        self.object_union = []
        self.object_union1 = []
        self.last_object_union = []
        self.last_object_union1 = []
        self.count = 0
        self.fang = False

        filename = os.environ['HOME'] + "/thefile.txt"
        with open(filename, 'r') as f:
            s = f.read()
        arr = s.split()
        self.x_kb = [float(arr[0]), float(arr[1])]
        self.y_kb = [float(arr[2]), float(arr[3])]
        rospy.logwarn('X axia k and b value: ' + str(self.x_kb))
        rospy.logwarn('X axia k and b value: ' + str(self.y_kb))

        # self.sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_cb, queue_size=1)
        # 订阅机械臂抓取指令
        self.sub2 = rospy.Subscriber(
            '/grasp', String, self.grasp_cp, queue_size=1)
        # 发布机械臂位姿
        self.pub1 = rospy.Publisher(
            'position_write_topic', position, queue_size=10)
        # 发布机械臂吸盘
        self.pub2 = rospy.Publisher('pump_topic', status, queue_size=1)
        # 发布机械臂状态
        self.grasp_status_pub = rospy.Publisher(
            'grasp_status', String, queue_size=1)
        # 发布TWist消息控制机器人底盘
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.arm_to_home()

    def hsv_fliter(self, img):
        # change rgb to hsv
        cv_img2 = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # 蓝色物体颜色检测范围
        LowerBlue = np.array([100, 110, 110])
        UpperBlue = np.array([130, 255, 255])
        mask = cv2.inRange(cv_img2, LowerBlue, UpperBlue)
        cv_img3 = cv2.bitwise_and(cv_img2, cv_img2, mask=mask)

        # gray process
        cv_img4 = cv_img3[:, :, 0]

        # smooth and clean noise
        blurred = cv2.blur(cv_img4, (9, 9))
        (_, thresh) = cv2.threshold(blurred, 90, 255, cv2.THRESH_BINARY)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        cv_img_ret = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
        cv_img_ret = cv2.erode(cv_img_ret, None, iterations=4)
        cv_img_ret = cv2.dilate(cv_img_ret, None, iterations=4)
        return cv_img_ret

    # 使用CV检测物体       
    def image_cb(self, data):
        xc = -1
        yc = -1
        # change to opencv
        try:
            cv_image1 = CvBridge().imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print('error')

        cv_image5 = self.hsv_fliter(cv_image1)

        contours, hier = cv2.findContours(cv_image5, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # if find contours, pick the biggest box
        if len(contours) == 0:
            self.is_have_object = False
            self.is_found_object = False
            self.found_count = 0
        else:
            self.is_have_object = True
            index = -1
            p_min = 10000
            self.object_union.clear()
            for i, c in enumerate(contours):
                area = cv2.contourArea(c)
                if area >500:
                    rect = cv2.minAreaRect(c)
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    cv2.drawContours(cv_image1, [box], 0, (0, 255, 0), 2)

                    x_mid = (box[0][0] + box[2][0] + box[1][0] + box[3][0]) / 4
                    y_mid = (box[0][1] + box[2][1] + box[1][1] + box[3][1]) / 4

                    w = math.sqrt((box[0][0] - box[1][0]) ** 2 + (box[0][1] - box[1][1]) ** 2)
                    h = math.sqrt((box[0][0] - box[3][0]) ** 2 + (box[0][1] - box[3][1]) ** 2)

                    # print("w=%s", w)
                    # print("h=%s", h)
                    size = w * h
                    # if size[i] > size_max:
                    #     size_max = size[i]
                    #     index = i
                    #     self.xc = x_mid
                    #     self.yc = y_mid

                    p, theta = cmath.polar(complex(x_mid - 320, 480 - y_mid))
                    cn = cmath.rect(p, theta)
                    # print("p=%s", p)         
                    if p > 350:
                        continue
                    self.object_union.append((p, theta, w, h, size, x_mid, y_mid))
                    
                    if p < p_min:
                        index = index + 1
                        p_min = p
                        xc = x_mid
                        yc = y_mid
                else:
                    continue
            self.object_union.sort(key=lambda x:x[0]) # 按抓取长度小到大排序
                    
            # if box is not moving for 20 times
            # print found_count
            if self.found_count >= 22:
                self.found_count = 0
                self.is_found_object = True
                self.xc = xc
                self.yc = yc
            else:
                # if box is not moving
                if index == -1:
                    # rospy.logwarn("No object eligible for grasp")
                    self.found_count = 0
                    self.is_found_object = False
                elif abs(xc - self.xc_prev) <= 8 and abs(yc - self.yc_prev) <= 8:
                    # cn = cmath.rect(self.object_union['p'][index], self.object_union['theta'][index])
                    # cv2.line(cv_image1, (320, 480), (int(320 + cn.real), int(480 - cn.imag)), (255, 0, 0), 2)
                    self.found_count = self.found_count + 1
                else:
                    self.found_count = 0
        self.xc_prev = xc
        self.yc_prev = yc
        
        cv2.imshow("source", cv_image1)
        # cv2.imshow("gray", cv_image5)
        cv2.waitKey(1)

        self.image_last_time = rospy.get_rostime()

    def image_cb1(self, data):
        xc = -1
        yc = -1
        # change to opencv
        try:
            cv_image1 = CvBridge().imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print('error')

        cv_image5 = self.hsv_fliter(cv_image1)

        contours, hier = cv2.findContours(cv_image5, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # self.object_union1.clear()
        # self.last_object_union1.clear()
        # if find contours, pick the biggest box
        if len(contours) == 0:
            self.is_have_object = False
            self.is_found_object = False
            self.found_count = 0
            self.object_union1.clear()
            self.last_object_union1.clear()
        else:
            self.is_have_object = True
            index = -1
            p_min = 10000
            self.object_union1.clear()
            self.last_object_union1.clear()
            for i, c in enumerate(contours):
                rect = cv2.minAreaRect(c)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                cv2.drawContours(cv_image1, [box], 0, (0, 255, 0), 2)

                x_mid = (box[0][0] + box[2][0] + box[1][0] + box[3][0]) / 4
                y_mid = (box[0][1] + box[2][1] + box[1][1] + box[3][1]) / 4

                w = math.sqrt((box[0][0] - box[1][0]) ** 2 + (box[0][1] - box[1][1]) ** 2)
                h = math.sqrt((box[0][0] - box[3][0]) ** 2 + (box[0][1] - box[3][1]) ** 2)

                # print("w=%s", w)
                # print("h=%s", h)
                size = w * h
                # if size[i] > size_max:
                #     size_max = size[i]
                #     index = i
                #     self.xc = x_mid
                #     self.yc = y_mid

                p, theta = cmath.polar(complex(x_mid - 320, 480 - y_mid))
                cn = cmath.rect(p, theta)
                # print("p=%s", p)         
                # if p > 350:
                #     continue
                self.object_union1.append((p, theta, w, h, size, x_mid, y_mid))
                
                # if p < p_min:
                #     index = index + 1
                #     p_min = p
                #     xc = x_mid
                #     yc = y_mid
            self.object_union1.sort(key=lambda x:x[0]) # 按抓取长度小到大排序

            self.found_count = 0
            self.is_found_object = True
            self.xc = xc
            self.yc = yc    
            # if box is not moving for 20 times
            # print found_count
            # if self.found_count >= 22:
            #     self.found_count = 0
            #     self.is_found_object = True
            #     self.xc = xc
            #     self.yc = yc
            # else:
            #     # if box is not moving
            #     if index == -1:
            #         # rospy.logwarn("No object eligible for grasp")
            #         self.found_count = 0
            #         self.is_found_object = False
            #     elif abs(xc - self.xc_prev) <= 8 and abs(yc - self.yc_prev) <= 8:
            #         # cn = cmath.rect(self.object_union['p'][index], self.object_union['theta'][index])
            #         # cv2.line(cv_image1, (320, 480), (int(320 + cn.real), int(480 - cn.imag)), (255, 0, 0), 2)
            #         self.found_count = self.found_count + 1
            #     else:
            #         self.found_count = 0
        self.xc_prev = xc
        self.yc_prev = yc
        
        cv2.imshow("source", cv_image1)
        # cv2.imshow("gray", cv_image5)
        cv2.waitKey(1)

        self.image_last_time = rospy.get_rostime()

    def grasp_cp(self, msg):
        if msg.data == '1':
            # 订阅摄像头话题,对图像信息进行处理
            self.sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_cb, queue_size=1)
            self.is_found_object = False
            times=0
            state=0
            rospy.sleep(1) # 等待1秒在当前画面寻找物体
            # 转一圈没有发现可抓取物体,退出抓取
            rospy.loginfo("-----search object")
            vel = Twist()
            vel.angular.z = 0.3
            rate = rospy.Rate(10)
            if not self.is_have_object:
                while not self.is_have_object:
                    rate.sleep()
                    times = times + 1
                    if (times > math.pi * 2 / vel.angular.z * 10):
                        rospy.logerr("can not found object!!! Waiting next clicked_point...")
                        self.grasp_status_pub.publish(String('0'))
                        state = -1
                        break
                    self.cmd_vel_pub.publish(vel)

            if (state != -1):
                times = 0
                rospy.loginfo("-----locate object")
                rospy.sleep(0.5)
                self.is_found_object = False
                while not self.is_found_object:
                    rate.sleep()
                    times = times + 1
                    if (times > 60):
                        rospy.logerr("object founded but can not location it!!! Waiting next clicked_point...")
                        self.grasp_status_pub.publish(String('0'))
                        state = -1
                        break

            if (state != -1):
                # 抓取检测到的物体    
                rospy.loginfo("-----start to grasp")
                self.last_object_union = deepcopy(self.object_union)
                grasp_retry_num = len(self.last_object_union)
                for i in range(grasp_retry_num):
                    # rospy.loginfo("-----xc=%.2f",self.last_object_union[i][5])  
                    # rospy.loginfo("-----yc=%.2f",self.last_object_union[i][6])  
                    result = self.grasp(self.last_object_union[i][5], self.last_object_union[i][6])
                    if result:
                        self.grasp_status_pub.publish(String('1')) 
                        break
                    elif i >= 2 or i == grasp_retry_num - 1:
                        self.grasp_status_pub.publish(String('0')) 
                        break
            else:
                self.grasp_status_pub.publish(String('0'))
            self.sub.unregister()
            rospy.loginfo("unregisted sub")

            # 几率性会报非 ROS 的错误，待解决
            # cv2.destroyAllWindows()
            # cv2.waitKey(1)

        if msg.data=='0':
            # 放下物体
            rospy.loginfo("-----start to release")
            self.fang = True
            # self.is_found_object = False
            # self.object_union1.clear()
            # self.last_object_union1.clear()
            self.sub1 = rospy.Subscriber("/camera/color/image_raw", Image, self.image_cb1, queue_size=1)
            self.last_object_union1 = deepcopy(self.object_union1)
            grasp_retry_num1 = len(self.last_object_union1)
            if self.is_found_object == True:
                rospy.loginfo("----- to release")
                for i in range(grasp_retry_num1):
                    rospy.loginfo("-----start to ")
                    # print(self.last_object_union[i][5])
                    # print(self.last_object_union[i][6])
                    self.release_object(xc=self.last_object_union1[i][5], yc=self.last_object_union1[i][6])
                    break          
                    
            else:
                self.release_object(xc=0, yc=0)
            rospy.loginfo("last_object_union_lenght: %d" % (len(self.last_object_union1)))
            rospy.loginfo("last_object_union: " + str(self.last_object_union1))
            if len(self.last_object_union) > 1:
                self.grasp_status_pub.publish(String('2'))
            else:
                self.grasp_status_pub.publish(String('2'))


        

    # 执行抓取
    def grasp(self, xc, yc):
        r1 = rospy.Rate(0.095)
        r2 = rospy.Rate(10)
        r3 = rospy.Rate(0.25)
        if self.fang == False:
            pos = position()

            # 物体所在坐标+标定误差
            pos.x = self.x_kb[0] * yc + self.x_kb[1]
            pos.y = self.y_kb[0] * xc + self.y_kb[1]
            pos.z = 20

            # pos.z = 20
            rospy.loginfo("to z = 20")
            self.pub1.publish(pos)
            # r3.sleep()
            rospy.sleep(0.25)

            # go down -50
            pos.z = -50
            self.pub1.publish(pos)
            rospy.loginfo("to z = -50")
            rospy.sleep(0.20)
            # r3.sleep()

            # 开始吸取物体
            rospy.loginfo("opening pump...")
            self.pub2.publish(1)
            # r3.sleep()
            rospy.sleep(0.25)


            rospy.loginfo("Grasp !!!")
            pos.x = 250  #160
            pos.y = 0
            pos.z = 150
            self.pub1.publish(pos)
            # pos.x = 20
            # pos.y = 90
            # pos.z = 100
            rospy.sleep(0.15)
            self.pub1.publish(45,-280,160)
            r3.sleep()
            self.is_found_object = False
        return True


    def arm_to_home(self):
        pos = position()
        pos.x = 120
        pos.y = 0
        pos.z = 35
        self.pub1.publish(pos)
           

    # 释放物体
    def release_object(self,xc, yc ,check=True):
        r1 = rospy.Rate(0.15)  # 5s
        r2 = rospy.Rate(1)     # 1s
        # if self.count == 0 or self.count == 1  :
        #     pos = position()
        #     pos.x = 230
        #     pos.y = 0
        #     pos.z = 80
        #     self.pub1.publish(pos)
        #     r2.sleep()
                    
        #     # stop pump
        #     rospy.loginfo("stop pump")
        #     self.pub2.publish(0)
        #     r2.sleep()

        #     rospy.loginfo("to home")
        #     self.arm_to_home()
        #     self.count += 1
        # elif self.count == 2:
        #     pos = position()
        #     pos.x = 230
        #     pos.y = 0
        #     pos.z = 160
        #     self.pub1.publish(pos)
        #     r2.sleep()
                    
        #     # stop pump
        #     rospy.loginfo("stop pump")
        #     self.pub2.publish(0)
        #     r2.sleep()

        #     rospy.loginfo("to home")
        #     self.arm_to_home()
        #     self.count = 0
        print("fang")
        if xc == 0 and yc == 0:
            # self.object_union1.clear()
            # self.last_object_union1.clear()
            pos = position()
            # go forward
            # pos.x = 200
            # pos.y = 0
            # pos.z = -40  #-80
            self.pub1.publish(190,0,-35)
            # r1.sleep()
            rospy.sleep(4.5)
            # stop pump
            self.pub2.publish(0)
            r2.sleep()
            # pos.x = 120
            # pos.y = 0
            # pos.z = 150
            # self.pub1.publish(pos)
            self.arm_to_home()
            self.count = 1
            r1.sleep()
        else:
            pos = position()
            pos.z = 160
            self.pub1.publish(pos)
            pos.x = self.x_kb[0] * yc + self.x_kb[1]
            pos.y = self.y_kb[0] * xc + self.y_kb[1]
            # rospy.loginfo("pos.x=%.2f",pos.x)
            # rospy.loginfo("pos.y=%.2f",pos.y)
            rospy.loginfo("self.count=%d",self.count)
            if self.count == 1:
                pos.z = 80
                self.pub1.publish(pos)
                self.count = self.count + 1
            else:
                pos.z = 170
                self.pub1.publish(pos)
                self.count = 0
            # rospy.sleep(0.35)
            rospy.sleep(4)
            self.pub2.publish(0)
            rospy.sleep(0.25)
            # pos.x = 120
            # pos.y = 0
            # pos.z = 150
            # self.pub1.publish(pos)
            # r1.sleep()
            self.arm_to_home()
        self.fang = False
        return True

    # 转动机器人到一定角度       
    # def turn_body(self):
    #     cmd_vel = Twist()
    #     cmd_vel.angular.z = 0.25
    #     rate = rospy.Rate(10)
    #     for i in range(40):            
    #         self.cmd_vel_pub.publish(cmd_vel)            
    #         rate.sleep()
       

        
if __name__ == '__main__':
    try:
        rospy.init_node('GraspObject', anonymous=False)
        rospy.loginfo("Init GraspObject main")   
        GraspObject()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("End spark GraspObject main")

