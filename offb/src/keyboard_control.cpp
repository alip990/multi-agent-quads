#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <keyboard/Key.h>
#include <math.h>

geometry_msgs::TwistStamped vs;


void sendCommand(const keyboard::Key &key){
   switch (key.code)
   {
   		case 'w':
   		{
   			ROS_INFO_STREAM("up");
   			vs.twist.linear.z += 0.1;
   			break;
   		}
      case 's':
      {
        ROS_INFO_STREAM("down");
        vs.twist.linear.z -= 0.1;
        break;
      }
   		case 'j':
   		{
   			ROS_INFO_STREAM("left");
   			vs.twist.linear.y += 0.1;
   			break;
   		}
   		case 'l':
   		{
   			ROS_INFO_STREAM("right");
   			vs.twist.linear.y -= 0.1;
   			break;
   		}
      case 'i':
      {
        ROS_INFO_STREAM("forward");
        vs.twist.linear.x += 0.1;
        break;
      }
      case 'k':
      {
        ROS_INFO_STREAM("backward");
        vs.twist.linear.x -= 0.1;
        break;
      }
      case 'u':
      {
        ROS_INFO_STREAM("rotate left");
        vs.twist.angular.z += 0.1;
        break;
      }
      case 'o':
      {
        ROS_INFO_STREAM("rotate right");
        vs.twist.angular.z -= 0.1;
        break;
      }
      case 'p':
      {
      //	isGetpoint = true;
      //	isGetGPSPoint = false;
     // isSearch=false;
      	ROS_INFO_STREAM("get to point 10 10 3");
              break;
      }
      //GPS point
      case 'g':
      {
      	//isGetGPSPoint = true;
      //	isGetpoint = false;
     // isSearch=false;
      	ROS_INFO_STREAM("get to GPS point");
      	break;
      }
      //case search plan q
      case 'q':
      {
        //isGetpoint=false;
        //isGetGPSPoint=false;
        //isSearch=true;
        ROS_INFO_STREAM("get to search target");
        break;
      }

      case 'h':
      {
        // turn to manual mode
        //isGetpoint = false;
        //isGetGPSPoint =false;
        vs.twist.linear.x = 0;
        vs.twist.linear.y = 0;
        vs.twist.linear.z = 0;
        vs.twist.angular.z = 0;
        ROS_INFO_STREAM("Manual Mode");
        break;
      }
      
      case 'a':
      {
        speed += 0.1;
        ROS_INFO_STREAM("speed up" << speed);
        break;
      }
      case 'd':
      {
        speed -= 0.1;
        ROS_INFO_STREAM("speed down" << speed);
        break;
      }

   		
   		default:
   		{
   			
   		}

   }
}


int main(int argc, char **argv){
  ros::init(argc, argv, "px4_keyboard_Control");
  ros::NodeHandle nodeHandle;

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
        vel_sp_pub.publish(vs);
      
    ros::spinOnce();

    loop_rate.sleep();
  }
}
