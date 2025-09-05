#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty
from std_msgs.msg import Float64

msg = """
Control mbot!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly

CTRL-C to quit
"""

moveBindings = {
        'i':(1,0),
        'o':(1,-1),
        'j':(0,1),
        'l':(0,-1),
        'u':(1,1),
        ',':(-1,0),
        '.':(-1,1),
        'm':(-1,-1),
           }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
          }

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

speed = .2
turn = 1

def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('sentry_teleop')
    pub_spin_LF = rospy.Publisher('/sentry/spin_leftfront_joint_controller/command', Float64, queue_size=1)
    pub_spin_LB = rospy.Publisher('/sentry/spin_leftback_joint_controller/command', Float64, queue_size=1)
    pub_spin_RF = rospy.Publisher('/sentry/spin_rightfront_joint_controller/command', Float64, queue_size=1)
    pub_spin_RB = rospy.Publisher('/sentry/spin_rightback_joint_controller/command', Float64, queue_size=1)
    pub_wheel_LF = rospy.Publisher('/sentry/wheel_leftfront_joint_controller/command', Float64, queue_size=1)
    pub_wheel_LB = rospy.Publisher('/sentry/wheel_leftback_joint_controller/command', Float64, queue_size=1)
    pub_wheel_RF = rospy.Publisher('/sentry/wheel_rightfront_joint_controller/command', Float64, queue_size=1)
    pub_wheel_RB = rospy.Publisher('/sentry/wheel_rightback_joint_controller/command', Float64, queue_size=1)

    x = 0
    th = 0
    status = 0
    count = 0
    acc = 0.1
    target_speed = 0
    target_turn = 0
    control_speed = 0
    control_turn = 0
    try:
        print(msg)
        print(vels(speed,turn))
        angle = 0.0;
        while(1):
            key = getKey()
            # 运动控制方向键（1：正方向，-1负方向）
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][1]
                count = 0
            # 速度修改键
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]  # 线速度增加0.1倍
                turn = turn * speedBindings[key][1]    # 角速度增加0.1倍
                count = 0

                print(vels(speed,turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            # 停止键
            elif key == ' ' or key == 'k' :
                x = 0
                th = 0
                control_speed = 0
                control_turn = 0
            else:
                count = count + 1
                if count > 4:
                    x = 0
                    th = 0
                if (key == '\x03'):
                    break

            # 目标速度=速度值*方向值
            target_speed = speed * x
            target_turn = turn * th

            # 速度限位，防止速度增减过快
            if target_speed > control_speed:
                control_speed = min( target_speed, control_speed + 0.02 )
            elif target_speed < control_speed:
                control_speed = max( target_speed, control_speed - 0.02 )
            else:
                control_speed = target_speed

            if target_turn > control_turn:
                control_turn = min( target_turn, control_turn + 0.1 )
            elif target_turn < control_turn:
                control_turn = max( target_turn, control_turn - 0.1 )
            else:
                control_turn = target_turn

            # 舵轮控制
            line_speed = Float64();
            angle_speed = Float64();
            line_speed = -50 * control_speed;
            angle_speed = -0.05 * control_turn;
            angle = angle + angle_speed;
            print("line_speed:",line_speed)
            print("angle:",angle)
            if angle > 1 :
                angle = 1
            elif angle < -1:
                angle = -1
            pub_spin_LF.publish(angle)
            pub_spin_LB.publish(0)
            pub_spin_RF.publish(angle)
            pub_spin_RB.publish(0)
            pub_wheel_LF.publish(line_speed)
            pub_wheel_LB.publish(line_speed)
            pub_wheel_RF.publish(-line_speed)
            pub_wheel_RB.publish(-line_speed)

            # 差速控制
            # left_speed = Float64();
            # right_speed = Float64();
            # left_speed = 50 * control_speed - 3*control_turn;
            # right_speed = 50 * control_speed + 3*control_turn;
            # print("left_speed:",left_speed)
            # print("right_speed:",right_speed)
            # pub_spin_LF.publish(0)
            # pub_spin_LB.publish(0)
            # pub_spin_RF.publish(0)
            # pub_spin_RB.publish(0)
            # pub_wheel_LF.publish(-left_speed)
            # pub_wheel_LB.publish(-left_speed)
            # pub_wheel_RF.publish(right_speed)
            # pub_wheel_RB.publish(right_speed)


    except:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
