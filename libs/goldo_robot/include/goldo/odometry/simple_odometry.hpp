#pragma once
#include <cstdint>
#include "goldo/geometry/geometry.hpp"
#include "goldo/odometry/odometry_config.hpp"

namespace goldo
{
    class SimpleOdometry
    {
    public:
    	SimpleOdometry();

		uint16_t leftEncoderValue() const noexcept;
		uint16_t rightEncoderValue() const noexcept;
		const RobotPose& pose() const noexcept;

		const OdometryConfig& config() const noexcept;
		void setConfig(const OdometryConfig& config);
		void setPose(const RobotPose& pose);
		
		void reset(uint16_t left_encoder, uint16_t right_encoder);
		void update(uint16_t left_encoder, uint16_t right_encoder);

		//! \brief Update current pose to be on specified line, with yaw normal to line.
		//! Used for repositioning on table borders
		void measureLineNormal(Vector2D normal, float distance);
		void measurePerpendicularPoint(float angle, float offset, Vector2D point);
        
    private:
		uint16_t m_left_encoder;
		uint16_t m_right_encoder;
		int m_left_accumulator;
		int m_right_accumulator;
        RobotPose m_pose;
        OdometryConfig m_config;

        double m_x;
        double m_y;
        float m_yaw;
        float m_speed_coeff_1;
        float m_speed_coeff_2;
        float m_acceleration_coeff_1;
        float m_acceleration_coeff_2;

        void updatePose(float dx, float dtheta, float dt);
    };
}
