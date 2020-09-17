#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <math.h>
#include <angles/angles.h>

float speed;
//radius
int R = 6371000;
geometry_msgs::TwistStamped vs;
sensor_msgs::NavSatFix currentGlobalPos;
ros::Publisher vel_sp_pub;
void gotoGPSPoint(double lat, double lon){
    double delta_x = R * (lat - currentGlobalPos.latitude) * float(3.1415926535898 / 180);
    double delta_y = R * (lon - currentGlobalPos.longitude) * float(3.1415926535898 / 180);
    vs.twist.linear.x = delta_x*0.1;
    vs.twist.linear.y = -delta_y*0.1;
    vs.header.stamp = ros::Time::now();
    vel_sp_pub.publish(vs);
}

void globalPositionReceived(const sensor_msgs::NavSatFixConstPtr& msg){
   currentGlobalPos = *msg;
   ROS_INFO_STREAM("Current GPS");	
}



int main(int argc, char **argv){
  ros::init(argc, argv, "px4_offboard_velocity_control_node");
  ros::NodeHandle nodeHandle;
  //publish set_velocity topic 100ms
  vel_sp_pub = nodeHandle.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
  ros::Subscriber globalPositionSubsciber =  nodeHandle.subscribe("/mavros/global_position/raw/fix", 10, globalPositionReceived);
  // ros::Subscriber attitudeSubsciber =  nodeHandle.subscribe("/mavros/global_position/raw/fix", 10, attitudeReceived);
  speed = 0.2;
//ros rate 
  ros::Rate loop_rate(10.0);

  while(ros::ok())
  {

    vs.header.seq++;
      //GPS topic 
      // ROS_INFO_STREAM("GPS longitude" << currentGlobalPos.longitude);
      // ROS_INFO_STREAM("GPS longitude" << currentGlobalPos.latitude);
      // ROS_INFO_STREAM("GPS longitude" << currentGlobalPos.altitude);
    vs.header.stamp = ros::Time::now();
        //ROS_INFO_STREAM("send ps" << ps);
    //vel_sp_pub.publish(vs);
    gotoGPSPoint(47.3930309,8.5475932);
    ROS_INFO_STREAM("GPS");
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0 ;
}
