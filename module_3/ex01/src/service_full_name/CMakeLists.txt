cmake_minimum_required(VERSION 3.5)
project(service_full_name)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(srv_files
  "srv/SummFullName.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
)

add_executable(service_name src/service_name.cpp)
ament_target_dependencies(service_name rclcpp)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(service_name "${cpp_typesupport_target}")

add_executable(client_name src/client_name.cpp)
ament_target_dependencies(client_name rclcpp)
target_link_libraries(client_name "${cpp_typesupport_target}")

install(TARGETS
  service_name
  client_name
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
