cmake_minimum_required(VERSION 3.8)
project(serial_com_stm32)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)

find_package(Threads REQUIRED)

add_executable(send_msg src/send_msg.cpp)

ament_target_dependencies("send_msg"
  "rclcpp"
  "interfaces"
)

target_link_libraries("send_msg"
  Threads::Threads
)

install(TARGETS
  send_msg
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
