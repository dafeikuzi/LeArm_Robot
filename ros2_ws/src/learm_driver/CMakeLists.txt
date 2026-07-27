cmake_minimum_required(VERSION 3.8)
project(learm_driver)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(arm_serial_control REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CalibrationMovePwm.srv"
  "srv/GetEncoder.srv"
  "srv/MoveJoints.srv"
  "srv/MovePose.srv"
  "srv/GetArmStatus.srv"
  "srv/SetGripper.srv"
)

add_library(learm_protocol src/protocol.cpp)
target_include_directories(learm_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(learm_driver_node src/learm_driver_node.cpp)
target_include_directories(learm_driver_node PRIVATE include)
ament_target_dependencies(learm_driver_node arm_serial_control rclcpp std_msgs std_srvs)
target_link_libraries(learm_driver_node learm_protocol)
set_target_properties(learm_driver_node PROPERTIES INSTALL_RPATH "$ORIGIN")
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(learm_driver_node "${cpp_typesupport_target}")

install(TARGETS learm_protocol learm_driver_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_link_libraries(test_protocol learm_protocol)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
