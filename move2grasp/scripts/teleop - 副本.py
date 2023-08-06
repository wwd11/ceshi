#!/usr/bin/env python3
# -*- coding: utf-8 -*
 
import  os
import  sys
import  tty, termios
import turtle
import roslib
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
 
#全局变量
cmd = Twist()
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
grasp_pub = rospy.Publisher('/grasp', String, queue_size=1)

global can_grasp
global can_release
global can_move
can = 0

def grasp_status_cp(msg):
    global can_release,can_grasp,can_move
    # 物体抓取成功,让机器人回起始点
    if msg.data=='1':
        can_release=True
        can_grasp=False
        can_move = True
    if msg.data=='0': #or msg.data=='-1':
        can_grasp=True
        can_move = False

def keyboardLoop():
    rospy.init_node('teleop')
    #初始化监听键盘按钮时间间隔
    rate = rospy.Rate(rospy.get_param('~hz', 10))
 
    #速度变量
    # 慢速
    walk_vel_ = rospy.get_param('walk_vel', 0.2)
    run_vel_ = rospy.get_param('run_vel', 0.5)
    # 快速
    yaw_rate_ = rospy.get_param('yaw_rate', 0.5)
    yaw_rate_run_ = rospy.get_param('yaw_rate_run', 0.5)
    # walk_vel_前后速度
    max_tv = walk_vel_
    # yaw_rate_旋转速度
    max_rv = yaw_rate_
    # 参数初始化
    speed=0
    global can_release,can_grasp, can_move, can
    can_grasp=True
    can_release=False
    
    print ("使用[WASD]控制机器人")
    print ("按[J]抓取1st,按[K]抓取2nd,按[L]抓取3rd")
    print ("按[I]放下物体")
    print ("按[Q]退出" )
 
    #读取按键循环
    while not rospy.is_shutdown():
        # linux下读取键盘按键
        fd = sys.stdin.fileno()
        turn =0
        old_settings = termios.tcgetattr(fd)
		#不产生回显效果
        old_settings[3] = old_settings[3] & ~termios.ICANON & ~termios.ECHO
        try :
            tty.setraw( fd )
            ch = sys.stdin.read( 1 )
        finally :
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        grasp_status=rospy.Subscriber('/grasp_status', String, grasp_status_cp, queue_size=1)
        # ch代表获取的键盘按键
        if ch == "j"  or ch == "J":
            if can == 0:
                if can_grasp:
                    print("抓取高度1")
                    msg=String()
                    msg.data= "J"
                    grasp_pub.publish(msg)
                    can_grasp=False
                    can_move=True
                    speed = 0
                    turn = 0
            else:
                if can_grasp:
                    print("抓取高度1")
                    msg=String()
                    msg.data= "1"
                    grasp_pub.publish(msg)
                    can_grasp=False
                    can_move=True
                    speed = 0
                    turn = 0

        elif ch == "k"  or ch == "K":
            if can == 0:
                if can_grasp:
                    print("抬高高度2")
                    msg=String()
                    msg.data= "K"
                    grasp_pub.publish(msg)
                    can_grasp=False
                    can_move=True
                    speed = 0
                    turn = 0
            else:
                if can_grasp:
                    print("抓取高度1")
                    msg=String()
                    msg.data= "2"
                    grasp_pub.publish(msg)
                    can_grasp=False
                    can_move=True
                    speed = 0
                    turn = 0
        
        elif ch == "l"  or ch == "L":
            if can == 0:
                if can_grasp:
                    print("抬高高度3")
                    msg=String()
                    msg.data= "L"
                    grasp_pub.publish(msg)
                    can_grasp=False
                    speed = 0
                    turn = 0
            else:
                if can_grasp:
                    print("抬高高度3")
                    msg=String()
                    msg.data= "3"
                    grasp_pub.publish(msg)
                    can_grasp=False
                    speed = 0
                    turn = 0


        elif ch =="i" or ch == "I" :
            if not can_grasp:
                print("fang")
                msg.data="h"
                grasp_pub.publish(msg)
                can_grasp=True
                speed = 0
                turn = 0

        elif ch =="h" or ch == "H" :
                print("ergao")
                msg = String()
                msg.data = "H"
                grasp_pub.publish(msg)
                can_grasp = True
                can = 0
                speed = 0
                turn = 0
        
        elif ch == "q"  or ch == "Q":
                print("secondfl")
                msg = String()
                msg.data = "q"
                grasp_pub.publish(msg)
                can_grasp = True
                speed = 0
                turn = 0

        elif ch == "e"  or ch == "E":
                print("thirdfl")
                msg = String()
                msg.data = "e"
                grasp_pub.publish(msg)
                can_grasp = True
                speed = 0
                turn = 0

        elif ch == "f"  or ch == "F":
                print("Back")
                msg = String()
                msg.data = "f"
                grasp_pub.publish(msg)
                can_grasp = True
                can = 1
                speed = 0
                turn = 0

        elif ch == 'y' or ch == 'Y':
            if can_move:
                print("Moving the arm to target point...")
                # 设置机械臂的目标位置（例如 goal.pos_x 和 goal.pos_y）
                goal.pos_x = 0.5  # 设置目标 x 坐标
                goal.pos_y = 0.2  # 设置目标 y 坐标
                
                # 向机械臂发送移动命令（根据实际情况修改）
                client.send_goal_and_wait(goal, rospy.Duration.from_sec(30))
                print("Arm moved to target point.")


        # elif ch == "4":
        #     print("dixi")
        #     msg = String()
        #     msg.data = "4"
        #     grasp_pub.publish(msg)
        #     can_grasp = True
        #     speed = 0
        #     turn = 0
        
        elif ch == "4":
            print("sheng")
            msg = String()
            msg.data = "4"
            grasp_pub.publish(msg)
            can_grasp = True
            speed = 0
            turn = 0

        elif ch == "6":
            print("jiang")
            msg = String()
            msg.data = "6"
            grasp_pub.publish(msg)
            can_grasp = True
            speed = 0
            turn = 0

        elif ch == "8":
            print("qian")
            msg = String()
            msg.data = "8"
            grasp_pub.publish(msg)
            can_grasp = True
            speed = 0
            turn = 0
        
        elif ch == "5":
            print("hou")
            msg = String()
            msg.data = "5"
            grasp_pub.publish(msg)
            can_grasp = True
            speed = 0
            turn = 0

        elif ch == "7":
            print("xi")
            msg = String()
            msg.data = "7"
            grasp_pub.publish(msg)
            speed = 0
            turn = 0

        elif ch == "9":
            print("fang")
            msg = String()
            msg.data = "9"
            grasp_pub.publish(msg)
            speed = 0
            turn = 0

        # elif ch == "7":
        #     print("Left")
        #     msg = String()
        #     msg.data = "7"
        #     grasp_pub.publish(msg)
        #     speed = 0
        #     turn = 0

        # elif ch == "9":
        #         print("XI")
        #         msg = String()
        #         msg.data = "9"
        #         grasp_pub.publish(msg)
        #         speed = 0
        #         turn = 0

        elif ch == "o":
            print("Fang")
            msg = String()
            msg.data = "0"
            grasp_pub.publish(msg)
            can_grasp = True
            speed = 0
            turn = 0


        elif ch == 'w':
            max_tv = walk_vel_
            speed = 0.5
            turn = 0
        elif ch == 's':
            max_tv = walk_vel_
            speed = -0.5
            turn = 0
        elif ch == 'a':
            max_rv = yaw_rate_
            speed = 0
            turn = 0.5
        elif ch == 'd':
            max_rv = yaw_rate_
            speed = 0
            turn = -0.5
        elif ch == 'W':
            max_tv = run_vel_
            speed = 5
            turn = 0
        elif ch == 'S':
            max_tv = run_vel_
            speed = -5
            turn = 0
        elif ch == 'A':
            max_rv = yaw_rate_run_
            speed = 0
            turn = 3.25
        elif ch == 'D':
            max_rv = yaw_rate_run_
            speed = 0
            turn = -3.25
        elif ch == "q":
            max_tv = walk_vel_
            speed = 0
            turn = 0.2
        elif ch == "e":
            max_rv = walk_vel_
            speed = 0
            turn=-0.2
        
        elif ch == 'q':
            exit()
        else:
            speed = 0
            turn = 0
 
        #发送消息
        cmd.linear.x = speed * max_tv
        cmd.angular.z = turn * max_rv
        pub.publish(cmd)
        # rate.sleep(0.05)
		#停止机器人
        #stop_robot()
 
def stop_robot():
    cmd.linear.x = 0.0
    cmd.angular.z = 0.0
    pub.publish(cmd)
 
if __name__ == '__main__':
    try:
        keyboardLoop()
    except rospy.ROSInterruptException:
        pass

