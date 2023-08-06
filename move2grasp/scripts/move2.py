#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
 
import rospy  
import actionlib  
from actionlib_msgs.msg import *
from std_msgs.msg import String  
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PointStamped  
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  
from tf.transformations import quaternion_from_euler  
from visualization_msgs.msg import Marker  
from math import radians, pi 
import tf 
import tf.transformations as tf_transformations
import numpy as np
from math import radians, pi
 
class Move2Grasp():
    STATE_PENDING = 0
    STATE_GOING = 1
    STATE_GRASP = 0
    STATE_HOMING = 0
    STATE_DROP = 0
    def __init__(self):

        self.count = 0

        rospy.init_node('move2grasp', anonymous=False)  
 
        rospy.on_shutdown(self.shutdown)
        #订阅RVIZ上的点击事件
        rospy.Subscriber('clicked_point', PointStamped, self.cp_callback)
        #订阅机械臂抓取状态
        rospy.Subscriber('/grasp_status', String, self.grasp_status_cp, queue_size=1)
        # Publisher to manually control the robot (e.g. to stop it)  
        # 发布TWist消息控制机器人  
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        # 发布机械臂抓取指令 
        self.grasp_pub = rospy.Publisher('/grasp', String, queue_size=1)
 
        # Subscribe to the move_base action server  
        # 订阅move_base服务器的消息  
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)  
 
        rospy.loginfo("Waiting for move_base action server...")  
 
        # Wait 60 seconds for the action server to become available  
        # 等待move_base服务器建立  
        self.move_base.wait_for_server(rospy.Duration(60))  
 
        rospy.loginfo("Connected to move base server")  
        rospy.loginfo("Starting navigation test")

        self.listener = tf.TransformListener()
        rospy.sleep(0.9)
        self.base_position = self.get_currect_pose()
        self.drop_position = np.dot(self.base_position, tf_transformations.compose_matrix(translate=[0.5, 0, 0], angles=[0, 0, pi]))
        self.last_position = None
        self.task_running = self.STATE_PENDING
  

    def get_currect_pose(self):
         (target_trans, target_rot) = self.listener.lookupTransform("map", "base_footprint", rospy.Time(0))
         return tf_transformations.compose_matrix(translate=target_trans, angles=tf_transformations.euler_from_quaternion(target_rot))
 
    def cp_callback(self, msg):
            rospy.loginfo("POINT:%f,%f,%f", msg.point.x, msg.point.y, msg.point.z)

            # Intialize the wayint goal  
            # 初始化goal为MoveBaseGoal类型  
            goal = MoveBaseGoal()  
 
            # Use the map frame to define goal poses  
            # 使用map的frame定义goal的frame id  
            goal.target_pose.header.frame_id = 'map'  
 
            # Set the time stamp to "now"  
            # 设置时间戳  
            goal.target_pose.header.stamp = rospy.Time.now()  
 
            # Set the goal pose 
            # 设置目标点  
            #pose = Pose(Point(0.0,0.0,0.0), Quaternion(0.0, 0.0, 0.0, 1.0))
            # pose = Pose(Point(msg.point.x,msg.point.y,msg.point.z), Quaternion(0.0, 0.0, 0.0, 1.0))
            quat = tf_transformations.quaternion_from_matrix(self.base_position)
            pose = Pose(Point(msg.point.x, msg.point.y, msg.point.z),
                        Quaternion(quat[0], quat[1], quat[2], quat[3]))
            goal.target_pose.pose=pose
 
            # Start the robot moving toward the goal  
            # 机器人移动  
            status=self.move(goal)
            # 如果到达指定地点,就发送抓取指令
            if status==True:
                #print('goal reached and start grasp')
                msg=String()
                msg.data='1'
                self.grasp_pub.publish(msg)

    def grasp_status_cp(self, msg):
            # 物体抓取成功,让机器人回起始点
            if msg.data=='1': 
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = 'map' 
                goal.target_pose.header.stamp = rospy.Time.now() 
                # pose = Pose(Point(-0.5,0.0,0.0), Quaternion(0.0, 0.0, 1.0, 0.0))
                tran = tf_transformations.translation_from_matrix(self.drop_position)
                quat = tf_transformations.quaternion_from_matrix(self.drop_position)
                pose = Pose(Point(tran[0], tran[1], tran[2]), Quaternion(quat[0], quat[1], quat[2], quat[3]))
                goal.target_pose.pose=pose  
                status=self.move(goal)
                #status=True
                # 到达起始点,放下物体
                if status==True:
                    if self.count == 0 or self.count == 1 or self.count == 2:
                         r1  = rospy.Rate(10)
                         vel = Twist()
                         vel.linear.x = 0.2
                         for i in range(10):
                              self.cmd_vel_pub.publish(vel)
                              r1.sleep()
                         self.grasp_pub.publish(String("0"))
                    elif self.count > 2:
                         r1  = rospy.Rate(10)
                         vel = Twist()
                         vel.linear.x = 0.15
                         for i in range(10):
                              self.cmd_vel_pub.publish(vel)
                              r1.sleep()
                         self.grasp_pub.publish(String("0"))
                    # msg=String()
                    # msg.data='0'
                    # self.grasp_pub.publish(msg)
                    self.count += 1
                    
                    


    def move(self, goal):  
            # Send the goal pose to the MoveBaseAction server  
            # 把目标位置发送给MoveBaseAction的服务器  
            self.move_base.send_goal(goal)  
            
            # Allow 20 scend to get there  
            # 设定20s的时间限制  
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(20))   
 
            # If we don't get there in time, abort the goal  
            # 如果一分钟之内没有到达，放弃目标  
            if not finished_within_time:  
                self.move_base.cancel_goal()  
                rospy.loginfo("Timed out achieving goal")
                return False  
            else:  
                # We made it!  
                state = self.move_base.get_state()  
                if state == GoalStatus.SUCCEEDED:  
                    rospy.loginfo("Goal succeeded!!!!!!!")
                    return True  
 
 
 
    def shutdown(self):  
        rospy.loginfo("Stopping the robot...")  
        # Cancel any active goals  
        self.move_base.cancel_goal()  
        rospy.sleep(2)  
        # Stop the robot  
        self.cmd_vel_pub.publish(Twist())  
        rospy.sleep(1)  
 
if __name__ == '__main__':  
    try:  
        Move2Grasp()
        rospy.spin()  
    except rospy.ROSInterruptException:  
        rospy.loginfo("Move2grasp finished.")
