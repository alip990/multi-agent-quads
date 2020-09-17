#!/usr/bin/env python

from MY_drone import MY_drone
import rospy
from geometry_msgs.msg import PoseStamped 
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64
import math
import numpy as np 
import threading 
class Follower():
    def __init__(self,followe_prefix,leader_prefix,safe_distance=10,critical_safe_distance=5):
        self.leader_prefix = leader_prefix
        self.drone = MY_drone(prefix=followe_prefix)
        self.leader_localPose = PoseStamped() 
        self.my_localPose=PoseStamped()
        self.myGoal_localPose = PoseStamped()# this position will be calculated from leader local pose in  calculate_localPose
        self.leader_local_velocity = TwistStamped() 
        self.leader_compass = Float64()
        self.leader_local_velocity_sub = rospy.Subscriber(self.leader_prefix+"mavros/local_position/velocity",TwistStamped , self.leader_velocity_sub_callback)
        self.leader_localPose_sub=rospy.Subscriber(self.leader_prefix+"/mavros/local_position/pose",PoseStamped,self.leader_localPose_sub_callback)
        self.my_localPose_sub=rospy.Subscriber(self.drone.mavros_prefix+"/mavros/local_position/pose",PoseStamped,self.mylocalPose_sub_callback)

        self.localPose_pub=rospy.Publisher(self.drone.mavros_prefix+"/mavros/setpoint_position/local",
                                            PoseStamped,queue_size=10)
        self.leader_compass_sub = rospy.Subscriber(self.leader_prefix+"/mavros/global_position/compass_hdg",Float64,self.leader_compass_callback)
        self.publish_waypoint_thread = threading.Thread(target=self.publish_waypoint_thread_func)
        self.safe_distance=safe_distance # if distance of leader and follower is less than safe distance the follwer will stop and current location will be poblish 
        self.critical_safe_distance=critical_safe_distance # # if distance of leader and follower is less than critical safe distance  the follower try to find best driction to get away from leader 
    def leader_localPose_sub_callback(self , msg):
        self.leader_localPose = msg
        self.calculate_localPose()
    def mylocalPose_sub_callback(self , msg):
        self.my_localPose = msg

    def leader_velocity_sub_callback(self,msg):
        self.leader_local_velocity = msg
    def leader_compass_callback(self,msg):
        self.leader_compass = msg
    def calculate_localPose(self):
        d=-10
        vf=self.leader_compass.data-90
        vf= -vf
        dn=d*np.cos(vf*3.14/180)
        de=d*np.sin(vf*3.14/180)
        
        self.myGoal_localPose.pose.position.x=self.leader_localPose.pose.position.x + dn
        self.myGoal_localPose.pose.position.y=self.leader_localPose.pose.position.y + de
        self.myGoal_localPose.pose.position.z=self.leader_localPose.pose.position.z+1
        '''
        print("leader x =   ",self.leader_localPose.pose.position.x)
        print("leader y =   ",self.leader_localPose.pose.position.y)
        print("followe x = " , self.myGoal_localPose.pose.position.x)
        print("follower y =" , self.myGoal_localPose.pose.position.y)
        print("dn is",dn)
        print("de is ",de)
        print("vf=",vf)
        '''
        self.myGoal_localPose.header.stamp=rospy.Time.now()
        self.myGoal_localPose.header.seq+=1
        self.myGoal_localPose.header.frame_id=self.leader_localPose.header.frame_id

    def  StartFollow (self):
        self.drone.setArm()
        for i in range (200):
            self.calculate_localPose()
        self.drone.setCustomeMode('OFFBOARD')
        rospy.loginfo("Drone armed and mode is OFFBOARD")
    
    def publish_waypoint_thread_func()
        
        self.localPose_pub.publish(self.myGoal_localPose)
        rospy.loginfo("Publish new goal position  ")
    def check_critical_safe_distance(self):
        pass

    def check_safe_distance(self): 
        distance =math.sqrt(pow(self.leader_localPose.pose.x - self.my_localPose.pose.x,2)+
                            pow(self.leader_localPose.pose.y - self.my_localPose.pose.y,2))
        if distance < self.safe_distance: # not in safe distance 
            return False 
        else :
            return True
    def check_watpoint_safe_distance(self):
        distance =math.sqrt(pow(self.myGoal_localPose.pose.x - self.my_localPose.pose.x,2)+
                            pow(self.myGoal_localPose.pose.y - self.my_localPose.pose.y,2))
        if distance < self.safe_distance: # not in safe distance 
            return False 
        else :
            return True
   def set_leader(self,leader): #TODO
	pass



if __name__ == "__main__":
    rospy.init_node("Follower")
    fol=Follower('/uav1','/uav0')
    fol.StartFollow()
    '''
    m=rospy.Publisher('uav1/mavros/setpoint_position/local',PoseStamped,queue_size=10)
    r=rospy.Rate(30)
    t=PoseStamped()
    t.pose.position.x=10
    t.pose.position.z=10
    while(True):
        r.sleep()
        t.header.stamp=rospy.Time.now()
        m.publish(t)

    '''
    while(True):
        pass
