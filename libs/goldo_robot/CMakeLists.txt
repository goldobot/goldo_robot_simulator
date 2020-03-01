add_library(goldo_robot STATIC)
target_sources(goldo_robot PRIVATE
  src/geometry/trajectory.cpp
  src/odometry/simple_odometry.cpp
  src/control/pid_controller.cpp
  src/control/propulsion_controller.cpp
  src/control/propulsion_low_level_controller.cpp
  src/control/speed_controller.cpp
  src/control/trapezoidal_speed_profile.cpp
  )
  
target_include_directories(goldo_robot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)