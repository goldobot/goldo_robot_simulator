#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <iostream>
#include <string.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "goldo_comm_uart");
  
  ros::NodeHandle n;
  ros::Publisher raw_message_pub = n.advertise<std_msgs::String>("stm32/in/console", 1000);
  ros::Rate loop_rate(10);  

  while (ros::ok())
  {
	std_msgs::String msg;
	msg.data = "hello world";	
	raw_message_pub.publish(msg);
	
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
