#!/usr/bin/env python

import rospy
from mavros_msgs.srv import SetMode as mavros_SetMode
from mavros_msgs.srv import CommandBool as mavros_CommandBool
from mavros_msgs.srv import CommandTOL as mavros_CommandTOL
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix  
from My_Queue import My_Queue
from copy import deepcopy
import math 
class MY_drone:
  def __init__(self,prefix=''):
    self.mavros_prefix=prefix # for use in multi vehicle 
    self.location=NavSatFix()
    self.fence_center=NavSatFix()
    self.fence_radius=50
    self.location_updated=False
    self.last_5_location=My_Queue()
    self.isModeChanged= False #TODO sth px4 refuse to change mode this should be handled or raise exeption 
    self.FlightMode='' #TODO Flight mode should be read from drone to be sure it is not changed by others 
    rospy.Subscriber(self.mavros_prefix+"/mavros/global_position/raw/fix", NavSatFix, self.globalPositionCallback)
    velocity_pub = rospy.Publisher(self.mavros_prefix+'/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    self.armService = rospy.ServiceProxy(self.mavros_prefix+'/mavros/cmd/arming', mavros_CommandBool)

    
  def globalPositionCallback(self,msg):
    self.location=msg
    self.location_updated=True
    self.last_5_location.enqueue(deepcopy(msg))

  def setArm(self):
    rospy.wait_for_service(self.mavros_prefix+'/mavros/cmd/arming')
    try:
      self.armService(True)
    except rospy.ServiceException, e:
      print ("Service arm call failed: %s"%e)

  def setDisarm(self):
    rospy.wait_for_service(self.mavros_prefix+'/mavros/cmd/arming')
    try:
      self.armService(False)
    except rospy.ServiceException, e:
      print ("Service arm call failed: %s"%e)

  def setHoldMode( self):
    rospy.wait_for_service(self.mavros_prefix+'/mavros/set_mode')
    try:
      self.flightModeService = rospy.ServiceProxy(self.mavros_prefix+'/mavros/set_mode', mavros_SetMode)
      self.isModeChanged = self.flightModeService(custom_mode='AUTO.LOITER') #return true or false
      self.FlightMode='AUTO.LOITER'
    except rospy.ServiceException, e:
      print ("service set_mode call failed: %s."%e)

  def setTakeoffMode(self): #TODO get gps position and use it as origin 
    rospy.wait_for_service(self.mavros_prefix+'/mavros/cmd/takeoff')
    try:
      self.takeoffService = rospy.ServiceProxy(self.mavros_prefix+'/mavros/cmd/takeoff', mavros_CommandTOL)
      self.takeoffService(altitude = 2, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
    except rospy.ServiceException, e:
      print ("Service takeoff call failed: %s"%e)

  def setLandMode(self):#TODO get gps 
    rospy.wait_for_service(self.mavros_prefix+'/mavros/cmd/land')
    try:
      self.landService = rospy.ServiceProxy(self.mavros_prefix+'/mavros/cmd/land', mavros_CommandTOL)
        #http://wiki.ros.org/mavros/CustomModes for custom modes
      self.isLanding = self.landService(altitude = 0, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
    except rospy.ServiceException, e:
        print ("service land call failed: %s. The vehicle cannot land "%e)
  
  def setCustomeMode(self,Mode):
    rospy.wait_for_service(self.mavros_prefix+'/mavros/set_mode')
    try:
      print("change mode")
      self.flightModeService = rospy.ServiceProxy(self.mavros_prefix+'/mavros/set_mode', mavros_SetMode)
      self.isModeChanged = self.flightModeService(custom_mode="OFFBOARD") #return true or false
      self.FlightMode=Mode
    except rospy.ServiceException, e:
      print ("service set_mode call failed: %s."%e)
      return self.isModeChanged    
  
  def check_safe_distance(self,other_gps):#TODO check safe distance with given gps(it can be drone)
    pass
  def check_be_in_fence_global(self,gps): #TODO check if given postion is out of fence or not 
      distance=self.haversine((gps.latitude , gps.longitude) , (self.location.latitude , self.location.longitude)  )
      if distance > self.fence_radius :
        return False
      else :
        return True 
  def set_fence(self,radius,fence_center=None): # if fence center posiion   donot pass , it will be the avg of 5 last location topic when func be called
    if  fence_center == None :
      sum_long=0
      for i in self.last_5_location.queue:
        sum_long+=i.longitude
        sum_lat+=i.latitude
      self.fence_center.longitude=sum_long/len(self.last_5_location.size())
      self.fence_center.latitude=sum_lat/len(self.last_5_location.size())
    else :
      self.fence_center=fence_center
    self.fence_radius=radius
    rospy.loginfo("New fense set !")

    def haversine(coord1, coord2):
      R = 6372800  # Earth radius in meters
      lat1, lon1 = coord1
      lat2, lon2 = coord2
      
      phi1, phi2 = math.radians(lat1), math.radians(lat2) 
      dphi       = math.radians(lat2 - lat1)
      dlambda    = math.radians(lon2 - lon1)
      
      a = math.sin(dphi/2)**2 + \
          math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
      
      return 2*R*math.atan2(math.sqrt(a), math.sqrt(1 - a))







  if __name__ == "__main__":
    rospy.init_node("testmydrone")  
    drone=MY_drone()
    drone.setTakeoffMode()
    while(True):
        if(drone.location_updated):
            print("location_recived")
            break
