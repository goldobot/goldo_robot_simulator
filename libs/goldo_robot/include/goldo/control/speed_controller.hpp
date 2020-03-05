#pragma once

namespace goldo
{
	class SpeedController
	{
		public:
		
		
		
		float currentSpeed();		
		void setTargetSpeed(float speed);
		
		
		
		
		private:
			float m_target_speed;
			float m_current_speed;
			float m_max_acceleration;
			float m_max_decceleration;
			float m_max_jerk;		
	};	
}