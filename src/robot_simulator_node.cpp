#include "ros/ros.h"

#include "goldo_msgs/RobotPose.h"
#include "robot_simulator.hpp"
#include "simple_odometry.hpp"
#include "propulsion_controller.hpp"

#include "goldo_msgs/SetBool.h"
#include "goldo_msgs/MotorsSetPwm.h"
#include "goldo_msgs/PropulsionPointTo.h"

goldo::RobotSimulator robot_simulator;
goldo::SimpleOdometry odometry;
goldo::PropulsionController propulsion_controller(&odometry);
  
  
  bool set_motors_enable(goldo_msgs::SetBool::Request &req, goldo_msgs::SetBool::Response &res)
  {
	  robot_simulator.setMotorsEnable(req.value);
	  return true;
  };
  
    bool set_propulsion_enable(goldo_msgs::SetBool::Request &req, goldo_msgs::SetBool::Response &res)
  {
	  propulsion_controller.setEnable(req.value);
	  return true;
  };
  
  bool motors_set_pwm(goldo_msgs::MotorsSetPwm::Request &req, goldo_msgs::MotorsSetPwm::Response &res)
  {
	  robot_simulator.setMotorsPwm(req.left, req.right);
	  return true;
  };
    bool propulsion_point_to(goldo_msgs::PropulsionPointTo::Request &req, goldo_msgs::PropulsionPointTo::Response &res)
  {
	  propulsion_controller.executePointTo(goldo::Vector2D{req.target.x, req.target.y}, req.yaw_rate, req.angular_acceleration, req.angular_decceleration);
	  return true;
  };
  
int main(int argc, char **argv)
{
  ros::init(argc, argv, "goldo_robot_simulator");
  
  ros::NodeHandle n;
  
  
  ros::Publisher odometry_pub = n.advertise<goldo_msgs::RobotPose>("stm32/odometry", 1000);
  
  
  
  ros::ServiceServer service1 = n.advertiseService("stm32/motors/set_enable", set_motors_enable);
  ros::ServiceServer service5 = n.advertiseService("stm32/motors/set_pwm", motors_set_pwm);
  ros::ServiceServer service2 = n.advertiseService("stm32/propulsion/set_enable", set_propulsion_enable);
  ros::ServiceServer service3 = n.advertiseService("stm32/propulsion/rotation", set_propulsion_enable);
  ros::ServiceServer service4 = n.advertiseService("stm32/propulsion/translation", set_propulsion_enable);
  ros::ServiceServer service6 = n.advertiseService("stm32/propulsion/point_to", propulsion_point_to);


  goldo::RobotSimulatorConfig simulator_config;
  simulator_config.speed_coeff = 1.7f;
  simulator_config.wheels_spacing = 0.2f;
  simulator_config.encoders_spacing = 0.3f;
  simulator_config.encoders_counts_per_m = 1 / 1.5e-05f;
  robot_simulator.setConfig(simulator_config);
  
  goldo::OdometryConfig odometry_config;
  odometry_config.dist_per_count_left = 1.5e-05f;
  odometry_config.dist_per_count_right = 1.5e-05f;
  odometry_config.wheel_spacing = 0.3f;
  odometry_config.update_period = 1e-3f;
  odometry_config.speed_filter_period = 1e-3f;
  odometry_config.encoder_period = 8192;
  
  odometry.setConfig(odometry_config);
  
  {
	  auto encoder_values = robot_simulator.readEncoders();
      odometry.reset(std::get<0>(encoder_values), std::get<1>(encoder_values));
  }
  
  ros::Rate loop_rate(100);
  uint32_t tick = 0;
  while (ros::ok())
  {
	  for(int i=0; i<10; i++)
	  {
		  robot_simulator.doStep();
		  auto encoder_values = robot_simulator.readEncoders();
		  odometry.update(std::get<0>(encoder_values), std::get<1>(encoder_values));
		  propulsion_controller.update();
		  if(propulsion_controller.state() != goldo::PropulsionController::State::Inactive)
		  {
			  robot_simulator.setMotorsPwm(propulsion_controller.leftMotorPwm(), propulsion_controller.rightMotorPwm());
		  }
		  tick++;
	  }
	  
	  auto odometry_pose = odometry.pose();
	  
	  goldo_msgs::RobotPose pose;
	  pose.position.x = odometry_pose.position.x;
	  pose.position.y = odometry_pose.position.y;
	  pose.yaw = odometry_pose.yaw;
	  pose.speed = odometry_pose.speed;
	  pose.yaw_rate = odometry_pose.yaw_rate;
	  pose.acceleration = 0;
	  pose.angular_acceleration = 0;
	  
	  odometry_pub.publish(pose);
	//std_msgs::String msg;
	//msg.data = "hello world";	
	//raw_message_pub.publish(msg);
	
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
