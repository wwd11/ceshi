#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import rospy
import actionlib
from actionlib_msgs.msg import *
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PointStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf
import tf.transformations as tf_transformations
from visualization_msgs.msg import Marker
from math import radians, pi
import numpy as np


class Move2Grasp():
    STATE_PENDING = 0
    STATE_GOING = 1
    STATE_GRASP = 0
    STATE_HOMING = 0
    STATE_DROP = 0
    count = 0
    def __init__(self):
        rospy.init_node('move2grasp', anonymous=False)

        rospy.on_shutdown(self.shutdown)
        # 订阅RVIZ上的点击事件
        rospy.Subscriber('clicked_point', PointStamped, self.cp_callback)
        self.clicked_point_pub = rospy.Publisher('clicked_point', PointStamped, queue_size=1)
        # 订阅机械臂抓取状态
        rospy.Subscriber('/grasp_status', String,
                         self.grasp_status_cp, queue_size=1)
        # Publisher to manually control the robot (e.g. to stop it)
        # 发布TWist消息控制机器人
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        # 发布机械臂抓取指令
        self.grasp_pub = rospy.Publisher('/grasp', String, queue_size=1)

        # Subscribe to the move_base action server
        # 订阅move_base服务器的消息
        self.move_base = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")

        # Wait 60 seconds for the action server to become available
        # 等待move_base服务器建立
        self.move_base.wait_for_server(rospy.Duration(60))

        rospy.loginfo("Connected to move base server")

        self.listener = tf.TransformListener()
        rospy.sleep(1) # need to delay
        self.base_position = self.get_currect_pose()
        self.drop_position = np.dot(self.base_position, tf_transformations.compose_matrix(translate=[0.5, 0, 0], angles=[0, 0, pi]))
        self.last_position = None
        self.task_running = self.STATE_PENDING

        rospy.loginfo("move node init ok...")

    def get_currect_pose(self):
        (target_trans, target_rot) = self.listener.lookupTransform("map", "base_footprint", rospy.Time(0))
        return tf_transformations.compose_matrix(translate=target_trans, angles=tf_transformations.euler_from_quaternion(target_rot))


    def cp_callback(self, msg):
        if (self.task_running == self.STATE_PENDING):
            self.task_running = self.STATE_GOING
            self.last_position = msg
            rospy.loginfo("POINT:%f,%f,%f", msg.point.x, msg.point.y, msg.point.z)

            # Intialize the waypoint goal
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
            quat = tf_transformations.quaternion_from_matrix(self.base_position)
            pose = Pose(Point(msg.point.x, msg.point.y, msg.point.z),
                        Quaternion(quat[0], quat[1], quat[2], quat[3]))
            goal.target_pose.pose = pose
            
            # Start the robot moving toward the goal
            # 机器人移动
            status = self.move(goal)
            # 如果到达指定地点,就发送抓取指令
            if status == True:
                rospy.loginfo('goal reached and start grasp')
                self.grasp_pub.publish(String('1'))
                self.task_running = self.STATE_GRASP
            else:
                rospy.logerr("navigation failed!!! Waiting next clicked_point...")
                self.task_running = self.STATE_PENDING
        else: rospy.logwarn("task still running. ignore")

    def grasp_status_cp(self, msg):

        # 抓取失败,任务重新开始
        if msg.data == '0':
            rospy.logerr("Grasp failed!!! Waiting next clicked_point...")
            self.task_running = self.STATE_PENDING

        # 物体抓取成功,让机器人回起始点
        elif msg.data == '1':
            self.task_running = self.STATE_HOMING
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            tran = tf_transformations.translation_from_matrix(self.drop_position)
            quat = tf_transformations.quaternion_from_matrix(self.drop_position)
            pose = Pose(Point(tran[0], tran[1], tran[2]), Quaternion(quat[0], quat[1], quat[2], quat[3]))
            goal.target_pose.pose = pose
            status = self.move(goal)
            # status=self.move(goal)
            # 到达起始点,放下物体
            if status != True:
                rospy.logerr("This is an unexpected situation because the robot")
                rospy.logerr("cannot return to the place where it was placed.")
                rospy.logerr("The robot will drop the object at the currect pose.")
                rospy.logerr("Than re-click point on Rviz...")
            else:
                # if self.count == 0 or self.count == 1 or self.count == 2:
                #     r1 = rospy.Rate(10)
                #     vel = Twist()
                #     vel.linear.x = 0.25
                #     for i in range(7):
                #         self.cmd_vel_pub.publish(vel)
                #         r1.sleep()
                #         self.count += 1
                        
                # elif self.count > 2:
                #     r1 = rospy.Rate(10)
                #     vel = Twist()
                #     vel.linear.x = 0.15
                #     for i in range(7):
                #         self.cmd_vel_pub.publish(vel)
                #         r1.sleep()
                r1 = rospy.Rate(10)
                vel = Twist()
                vel.linear.x = 0.15
                for i in range(8):
                    self.cmd_vel_pub.publish(vel)
                    r1.sleep()
                rospy.sleep(2)
            self.grasp_pub.publish(String('0'))
            self.task_running = self.STATE_DROP

        # 放置成功,任务结束
        elif msg.data == '2':
            rospy.loginfo("task finish!!! Waiting next clicked_point...")
            self.task_running = self.STATE_PENDING
            r1 = rospy.Rate(10)
            vel = Twist()
            vel.linear.x = -0.1
            for i in range(10):
                self.cmd_vel_pub.publish(vel)
                r1.sleep()

        # 最近一次抓取的位置还有能抓的方块, 再导航过去抓取
        elif msg.data == '3':
            rospy.loginfo("There is still an object to grab at the last grab position. Go to the last position")
            r1 = rospy.Rate(10)
            vel = Twist()
            vel.linear.x = -0.1
            for i in range(10):
                self.cmd_vel_pub.publish(vel)
                r1.sleep()
            self.clicked_point_pub.publish(self.last_position)



    def move(self, goal):
        # Send the goal pose to the MoveBaseAction server
        # 把目标位置发送给MoveBaseAction的服务器
        self.move_base.send_goal(goal)

        # Allow 1 minute to get there
        # 设定1分钟的时间限制
        finished_within_time = self.move_base.wait_for_result(
            rospy.Duration(60))

        # If we don't get there in time, abort the goal
        # 如果一分钟之内没有到达，放弃目标
        if not finished_within_time:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
            return False
        else:
            # We made it!
            state = self.move_base.get_state()
            rospy.loginfo(state)
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal succeeded!!!!!!!")
                rospy.sleep(2) # 停稳
                return True

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        # Cancel any active goals
        if (self.move_base.get_state == 1):
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
