cmake_minimum_required(VERSION 3.5)
project(position_control)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

install(PROGRAMS
  nodes/kalman_filter.py
  nodes/ranges_debugger.py
  nodes/setpoint.py
  nodes/yaw_controller.py
  nodes/x_controller_kalman.py
  nodes/x_controller_vision.py
  nodes/y_controller_vision.py
  nodes/y_controller_kalman.py
  nodes/z_controller_vision.py
  nodes/z_controller_kalman.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
