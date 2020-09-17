#!/usr/bin/env python

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy

from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
from mavros_msgs.srv import CommandBool
import sys, select, termios, tty
from sensor_msgs.msg import NavSatFix  
vs=TwistStamped()
pub_gps=None
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
def arm( state):
    try:
        rospy.wait_for_service('uav0/mavros/cmd/arming', 30)
        Arming = rospy.ServiceProxy('uav0/mavros/cmd/arming', CommandBool)
        arm_state=Arming(True)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
def calback_gps(data):
    pub_gps.publish(data)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node("keyboard_control")
    pub = rospy.Publisher('uav0/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=5)
    pub_gps= rospy.Publisher('uav0/my_gps', NavSatFix, queue_size=10)
    rospy.Subscriber("uav0/mavros/global_position/global", NavSatFix, calback_gps)
    rate= rospy.Rate(20)
    arm(True)
    while(1):
        try :
            key = getKey()
            if key=='w':
                rospy.loginfo("up")  
                vs.twist.linear.z += 0.1
            elif key=='s':
                vs.twist.linear.z -=0.1
                rospy.loginfo("down") 
            elif key=='j':
                vs.twist.linear.y +=0.1
                rospy.loginfo("left")
            elif key=='l':
                vs.twist.linear.y -=0.1
                rospy.loginfo("right")
            elif key=='i':
                vs.twist.linear.x +=0.1
                rospy.loginfo("forward")
            elif key=='k':
                vs.twist.linear.x -=0.1
                rospy.loginfo("backward")
            elif key=='u':
                vs.twist.angular.z +=0.1
                rospy.loginfo("roatate left ")
            elif key=='o':
                vs.twist.angular.z -=0.1
                rospy.loginfo("rotate right")
            elif key =='h':
                vs.twist.linear.z =0
                vs.twist.linear.y=0
                vs.twist.linear.x=0
                vs.twist.angular.z=0
                rospy.loginfo("Stop now")
            elif key=='z':
                raise rospy.ROSInterruptException()

            vs.header.stamp=rospy.Time.now()
            vs.header.seq +=1
            pub.publish(vs)
            rospy.loginfo("Publish cmd_vel")

        except rospy.ROSInterruptException:
            exit() 
   
        rate.sleep()
            #print("loop: {0}".format(count))
            #print("target: vx: {0}, wz: {1}".format(target_speed, target_turn))
            #print("publihsed: vx: {0}, wz: {1}".format(twist.linear.x, twist.angular.z))
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
