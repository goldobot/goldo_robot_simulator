#include "ros/ros.h"

#include "goldo_msgs/RobotPose.h"
#include "robot_simulator.hpp"
#include "goldo/odometry/simple_odometry.hpp"
#include "goldo/control/propulsion_controller.hpp"

#include "goldo_msgs/SetBool.h"
#include "goldo_msgs/MotorsSetPwm.h"
#include "goldo_msgs/PropulsionPointTo.h"
#include "goldo_msgs/PropulsionMoveTo.h"
#include "goldo_msgs/PropulsionExecuteTrajectory.h"

#include "std_msgs/Bool.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/Float32.h"

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
  
      bool propulsion_move_to(goldo_msgs::PropulsionMoveTo::Request &req, goldo_msgs::PropulsionMoveTo::Response &res)
  {
	  propulsion_controller.executeMoveTo(goldo::Vector2D{req.target.x, req.target.y}, req.speed, req.acceleration, req.decceleration);
	  return true;
  };
  
        bool propulsion_execute_trajectory(goldo_msgs::PropulsionExecuteTrajectory::Request &req, goldo_msgs::PropulsionExecuteTrajectory::Response &res)
  {
	  std::vector<goldo::Vector2D> points;
	  for(auto pt : req.trajectory.points)
	  {
		  points.push_back(goldo::Vector2D{pt.x, pt.y});
	  };
	  propulsion_controller.executeTrajectory(points.data(), points.size(), req.speed, req.acceleration, req.decceleration);
	  return true;
  };

  
int main(int argc, char **argv)
{
  ros::init(argc, argv, "goldo_robot_simulator");
  
  ros::NodeHandle n;
  
  
  ros::Publisher odometry_pub = n.advertise<goldo_msgs::RobotPose>("stm32/odometry", 1000);
  ros::Publisher propulsion_state_pub = n.advertise<std_msgs::UInt32>("propulsion/state", 1000, true);
  ros::Publisher propulsion_target_pose_pub = n.advertise<goldo_msgs::RobotPose>("propulsion/target_pose", 1000, true);
  
  ros::Publisher motors_enable_pub = n.advertise<std_msgs::Bool>("motors/enable", 1000);
  ros::Publisher motors_pwm_left_pub = n.advertise<std_msgs::Float32>("motors/pwm_left", 1000);
  ros::Publisher motors_pwm_right_pub = n.advertise<std_msgs::Float32>("motors/pwm_right", 1000);
  ros::Publisher gpio_pub = n.advertise<std_msgs::UInt32>("stm32/gpio", 1000);
  ros::Publisher sensors_pub = n.advertise<std_msgs::UInt32>("stm32/sensors", 1000);
  
  ros::ServiceServer service1 = n.advertiseService("stm32/motors/set_enable", set_motors_enable);
  ros::ServiceServer service5 = n.advertiseService("stm32/motors/set_pwm", motors_set_pwm);
  ros::ServiceServer service2 = n.advertiseService("stm32/propulsion/set_enable", set_propulsion_enable);
  ros::ServiceServer service3 = n.advertiseService("stm32/propulsion/rotation", set_propulsion_enable);
  ros::ServiceServer service4 = n.advertiseService("stm32/propulsion/translation", set_propulsion_enable);
  ros::ServiceServer service6 = n.advertiseService("stm32/propulsion/point_to", propulsion_point_to);
  ros::ServiceServer service7 = n.advertiseService("stm32/propulsion/move_to", propulsion_move_to);
  ros::ServiceServer service8 = n.advertiseService("stm32/propulsion/execute_trajectory", propulsion_execute_trajectory);

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
  
  goldo::PropulsionControllerConfig propulsion_controller_config;
  memset(&propulsion_controller_config, 0, sizeof(propulsion_controller_config));
  
  
  goldo::PropulsionLowLevelControllerConfig llc;
  goldo::PIDConfig pid1;
  goldo::PIDConfig pid2;
  
	pid1.period=1e-3f;
	pid1.kp=5;
	pid1.ki=1;
	pid1.kd=0;
	pid1.feed_forward=0.5f;
	pid1.lim_iterm=0.5;
	pid1.lim_dterm=0;
	pid1.min_output=-1.0f;
	pid1.max_output=1.0f;
	
	pid2.period=1e-3f;
	pid2.kp=0.1;
	pid2.ki=0;
	pid2.kd=0;
	pid2.feed_forward=0.01f;
	pid2.lim_iterm=0.5;
	pid2.lim_dterm=0;
	pid2.min_output=-1.0f;
	pid2.max_output=1.0f;
		
  llc.speed_pid_config = pid1;
  llc.longi_pid_config = pid1;
  llc.yaw_rate_pid_config = pid2;
  llc.yaw_pid_config = pid2;
  
  propulsion_controller_config.low_level_config_static = llc;
  propulsion_controller_config.low_level_config_cruise = llc;
  propulsion_controller_config.low_level_config_rotate = llc;
  
  propulsion_controller_config.lookahead_distance = 0.1f;
  propulsion_controller_config.lookahead_time = 0.1f;
  propulsion_controller_config.static_pwm_limit = 1.0f;
  propulsion_controller_config.cruise_pwm_limit = 1.0f;
  
  
  
  
  propulsion_controller.setConfig(propulsion_controller_config);
  
  
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
	  
	  {
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
	  }
	//std_msgs::String msg;
	//msg.data = "hello world";	
	//raw_message_pub.publish(msg);
	
	if(tick % 100 == 0)
	{
		std_msgs::UInt32 msg;
		msg.data = static_cast<uint32_t>(propulsion_controller.state());
		propulsion_state_pub.publish(msg);
		
		{
			auto odometry_pose = propulsion_controller.targetPose();
			
			  goldo_msgs::RobotPose pose;
			  pose.position.x = odometry_pose.position.x;
			  pose.position.y = odometry_pose.position.y;
			  pose.yaw = odometry_pose.yaw;
			  pose.speed = odometry_pose.speed;
			  pose.yaw_rate = odometry_pose.yaw_rate;
			  pose.acceleration = 0;
			  pose.angular_acceleration = 0;
			  
			propulsion_target_pose_pub.publish(pose);
		}
		
		{
			std_msgs::Bool msg;
			msg.data = robot_simulator.getMotorsEnable();
			motors_enable_pub.publish(msg);
		}
		{
			std_msgs::Float32 msg;
			msg.data = std::get<0>(robot_simulator.getMotorsPwm());
			motors_pwm_left_pub.publish(msg);
		}
		{
			std_msgs::Float32 msg;
			msg.data = std::get<1>(robot_simulator.getMotorsPwm());
			motors_pwm_right_pub.publish(msg);
		}
		
	}
	
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
