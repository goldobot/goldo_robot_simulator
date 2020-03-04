#pragma once
#include <tuple>
#include <cstdint>

namespace goldo
{
	struct RobotSimulatorConfig
	{
		float speed_coeff; // wheel speed = speed_coeff * pwm
		float wheels_spacing; // wheels spacing
		float encoders_spacing;
		float encoders_counts_per_m;
	};

	class RobotSimulator
	{
	public:
		RobotSimulator() = default;
		void doStep();
		void setConfig(const RobotSimulatorConfig& config);
		bool getMotorsEnable() const noexcept;
		void setMotorsEnable(bool enable);
		std::tuple<float, float> getMotorsPwm();
		void setMotorsPwm(float left, float right);
		std::tuple<uint16_t, uint16_t> readEncoders() const noexcept;
		
	private:

		RobotSimulatorConfig m_config;

		double m_x{0};
		double m_y{0};
		double m_yaw{0};
		double m_speed{0};
		double m_yaw_rate{0};

		bool m_motors_enable{false};
		float m_left_pwm{0};
		float m_right_pwm{0};

		double m_left_encoder_delta{0};
		double m_right_encoder_delta{0};

		int m_left_encoder{0};
		int m_right_encoder{0};
	};
}
