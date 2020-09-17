#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <sensor_msgs/NavSatFix.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64.h>
#include <bits/stdc++.h>
#include <math.h>
#include <tf/transform_broadcaster.h>// global variables
mavros_msgs::State current_state;
sensor_msgs::NavSatFix global_position;
bool global_position_received = false;
geographic_msgs::GeoPoseStamped my_goal_position;
geometry_msgs::TwistStamped gps_vel ; 
geometry_msgs::TwistStamped local_vel ; 

std_msgs::Float64 compass;
ros::Publisher check_goal_position;
void set_new_global_location(void){
    const int d = -30;
    const int R=6356752; // osp! it seems to be 6373.0 km or 6356752m
    //double vf=atan2(local_vel.twist.linear.x,local_vel.twist.linear.y);
    double vf=compass.data;
    double dn=d*cos((vf*3.14)/180);
    double de= d*sin((vf*3.14)/180);
    double  dlat= dn/R;
    double dlon=de/(R*cos(global_position.latitude*3.14/180));
    dlat=dlat*180/3.14;
    dlon=dlon*180/3.14;
    ROS_INFO("leader global position: [%f, %f]", global_position.latitude, global_position.longitude);    
    ROS_INFO("dn = %f",dn);
    ROS_INFO("de = %f",de);
    ROS_INFO("dlat = %f", dlat);
    ROS_INFO("dlong = %f",dlon);
    ROS_INFO("leader global position: [%f, %f]", my_goal_position.pose.position.latitude, my_goal_position.pose.position.longitude); 
    
    my_goal_position.pose.position.latitude = global_position.latitude+dlat;
    my_goal_position.pose.position.longitude = global_position.longitude+dlon;
    my_goal_position.pose.position.altitude=global_position.altitude-48;/// I think sth is wrong between attiude of  mavros/attiude  and mavros/global // offset is 48 
 
    my_goal_position.pose.orientation= tf::createQuaternionMsgFromRollPitchYaw 	( 	0,
	                                                                            	0,
	                                                                            	-(vf-90)*3.14/180 
	                                                                                                    ) 	;

    //check_goal_position.publish(my_goal_position);


   /*
   double delta = d/R;
   double theta=vf*3.14/180;
   double phi1=global_position.latitude ;//leader lat
   double landa1=global_position.longitude;
   double phi2= asin(sin(phi1)*cos(delta) +cos(phi1)*sin(delta)*cos(theta));
   double landa2=landa1+ atan2(sin(theta)*sin(delta)*cos(phi1),cos(delta)-sin(phi1)*sin(phi2));
    my_goal_position.header.stamp = ros::Time::now();
    my_goal_position.header.seq++;
    my_goal_position.pose.position.latitude = phi2;
    my_goal_position.pose.position.longitude = landa2;
    my_goal_position.pose.position.altitude=global_position.altitude-48;
    check_goal_position.publish(my_goal_position);*/
}
// callback functions
void globalPosition_cb(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    global_position = *msg;
    global_position_received = true;
    ROS_INFO_ONCE("Got global position: [%.2f, %.2f, %.2f]", msg->latitude, msg->longitude, msg->altitude);
    set_new_global_location();
}

void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}
void get_gps_vel(const geometry_msgs::TwistStamped::ConstPtr& msg){
    gps_vel=*msg;
    ROS_INFO_ONCE("Gps velocity got !  ");

}
void get_local_vel(const geometry_msgs::TwistStamped::ConstPtr& msg){
    local_vel=*msg;
    ROS_INFO_ONCE("Gps velocity got !  ");

}
void get_compass(const std_msgs::Float64::ConstPtr& msg){
    compass = *msg;
}

// main function
int main(int argc, char **argv) {
    ros::init(argc, argv, "test");
    ros::NodeHandle nh;
    check_goal_position=nh.advertise < geographic_msgs::GeoPoseStamped> ("uav1/check_goal_position",10);
    ros::ServiceClient arming_client = nh.serviceClient < mavros_msgs::CommandBool > ("uav1/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient < mavros_msgs::SetMode > ("uav1/mavros/set_mode");
    ros::Subscriber state_sub = nh.subscribe < mavros_msgs::State > ("uav1/mavros/state", 10, state_cb);
    ros::Subscriber global_pos_sub = nh.subscribe < sensor_msgs::NavSatFix > ("uav0/my_gps", 1, globalPosition_cb);
    ros::Publisher goal_pos_pub = nh.advertise < geographic_msgs::GeoPoseStamped > ("uav1/mavros/setpoint_position/global", 10);
    ros::Subscriber gps_vel_sub= nh.subscribe < geometry_msgs::TwistStamped >("uav0/mavros/global_position/raw/gps_vel", 1,get_gps_vel);
    ros::Subscriber local_vel_sub= nh.subscribe < geometry_msgs::TwistStamped >("uav0/mavros/local_position/velocity", 1,get_local_vel);
    // wait for fcu connectionstd_msgs::Float64
    ros::Subscriber compass_sub= nh.subscribe < std_msgs::Float64 >("uav0/mavros/global_position/compass_hdg", 1,get_compass);
    ros::Rate rate(20.0);

    while (ros::ok() && !current_state.connected) {
        ROS_INFO_ONCE("Waiting for FCU connection...");
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("FCU connected");
    
    // wait for position information
    while (ros::ok() && !global_position_received) {
        ROS_INFO_ONCE("Waiting for GPS signal...");
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("GPS position received");
  
    // set target position
    /*mavros_msgs::GlobalPositionTarget goal_position;
    goal_position.latitude = global_position.latitude;
    goal_position.longitude = global_position.longitude;
    goal_position.altitude = global_position.altitude+10;
*/
    // send a few setpoints before starting
    for (int i=0; i<20; ++i) {
        my_goal_position.header.stamp = ros::Time::now();
        goal_pos_pub.publish(my_goal_position);
        ros::spinOnce();
        rate.sleep();
    }

    // set mode
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.base_mode = 0;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
        ROS_INFO("OFFBOARD enabled");
    } else {
        ROS_ERROR("Failed to set OFFBOARD");
    }

    // arm
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
        ROS_INFO("Vehicle armed");
    } else {
        ROS_ERROR("Arming failed");
    }

    // take off to 5m above ground
    //goal_position.altitude = goal_position.altitude + 5.0;
    while (ros::ok()) {
        //goal_position.header.stamp = ros::Time::now();
        my_goal_position.header.stamp = ros::Time::now();
        my_goal_position.header.seq++;
        goal_pos_pub.publish(my_goal_position);
        ros::spinOnce();
        ROS_INFO_THROTTLE(1, "At altitude %.2f", my_goal_position.pose.position.altitude);
        rate.sleep();
    }

    return 0;
}

