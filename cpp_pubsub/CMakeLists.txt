cmake_minimum_required(VERSION 3.5)
project(cpp_pubsub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

ament_export_dependencies(rosidl_default_generators)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Stringss.srv"
)

add_executable(talker src/publisher.cpp)
ament_target_dependencies(talker rclcpp std_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

add_executable(listener src/subscriber.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

add_executable(clienttt src/client.cpp)
ament_target_dependencies(clienttt rclcpp std_msgs)

rosidl_target_interfaces(talker 
                        ${PROJECT_NAME} "rosidl_typesupport_cpp")

rosidl_target_interfaces(clienttt
                        ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  talker
  listener
  clienttt
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()