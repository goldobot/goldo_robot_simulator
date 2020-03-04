#include "robot_simulator.hpp"

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288
#endif

using namespace goldo;


void RobotSimulator::setConfig(const RobotSimulatorConfig& config)
{
	m_config = config;
}

bool RobotSimulator::getMotorsEnable() const noexcept
{
	return m_motors_enable;
}

void RobotSimulator::setMotorsEnable(bool enable)
{
	m_motors_enable = enable;
}

std::tuple<float, float> RobotSimulator::getMotorsPwm()
{
	return std::make_tuple(m_left_pwm, m_right_pwm);	
}
void RobotSimulator::setMotorsPwm(float left, float right)
{
	m_left_pwm = left;
	m_right_pwm = right;	
}

std::tuple<uint16_t, uint16_t> RobotSimulator::readEncoders() const noexcept
{
	return std::make_tuple(m_left_encoder, m_right_encoder);	
}
void RobotSimulator::doStep()
{
	// Simple formula for speed and yaw, with perfect motor controllers.
	float target_speed = 0.5 * (m_left_pwm + m_right_pwm) * m_config.speed_coeff;
	float target_yaw_rate = (m_right_pwm - m_left_pwm) * m_config.speed_coeff / m_config.wheels_spacing;
	
	if(!m_motors_enable)
	{
		target_speed = 0;
		target_yaw_rate = 0;
	}

	// Low pass filter
	target_speed = target_speed * 0.5 + 0.5 * m_speed;
	target_yaw_rate = target_yaw_rate * 0.5 + 0.5 * m_yaw_rate;

	//Limit acceleration
	float acceleration_limit = 2;
	if(target_speed > m_speed + 1e-3 * acceleration_limit)
	{
		m_speed = m_speed + 1e-3 * acceleration_limit;
	} else if(target_speed < m_speed - 1e-3 * acceleration_limit)
	{
		m_speed = m_speed - 1e-3 * acceleration_limit;
	} else
	{
		m_speed = target_speed;
	}

	m_yaw_rate = target_yaw_rate;

	// Update robot pose
	double d_yaw = 1e-3 * m_yaw_rate;
	double d_trans = 1e-3 * m_speed;

	m_x += cos(m_yaw + 0.5 * d_yaw) * d_trans;
	m_y += sin(m_yaw + 0.5 * d_yaw) * d_trans;
	m_yaw += d_yaw;

	// Clamp yaw
	while (m_yaw > M_PI)
	{
		m_yaw -= 2 * M_PI;
	}
	while (m_yaw < -M_PI)
	{
		m_yaw += 2 * M_PI;
	}

	// Update encoders
	m_left_encoder_delta += (d_trans - 0.5 * d_yaw * m_config.encoders_spacing) * m_config.encoders_counts_per_m;
	m_right_encoder_delta += (d_trans + 0.5 * d_yaw * m_config.encoders_spacing) * m_config.encoders_counts_per_m;

	int dcounts_left = static_cast<int>(std::trunc(m_left_encoder_delta));
	m_left_encoder_delta -= dcounts_left;

	int dcounts_right = static_cast<int>(std::trunc(m_right_encoder_delta));
	m_right_encoder_delta -= dcounts_right;

	m_left_encoder += dcounts_left;
	m_right_encoder += dcounts_right;

	if (m_left_encoder >= 8192)
	{
		m_left_encoder -= 8192;
	}
	if (m_left_encoder < 0)
	{
		m_left_encoder += 8192;
	}

	if (m_right_encoder >= 8192)
	{
		m_right_encoder -= 8192;
	}
	if (m_right_encoder < 0)
	{
		m_right_encoder += 8192;
	}
}
