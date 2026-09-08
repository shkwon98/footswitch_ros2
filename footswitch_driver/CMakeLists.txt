cmake_minimum_required(VERSION 3.12.0)
project(footswitch_driver)

list(INSERT CMAKE_MODULE_PATH 0 "${PROJECT_SOURCE_DIR}/cmake")

find_package(ament_cmake REQUIRED)
find_package(footswitch_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(HIDAPI REQUIRED COMPONENTS hidraw)

include_directories(include ${HIDAPI_INCLUDE_DIRS})

set(ROS_DEPENDENCIES footswitch_msgs rclcpp rclcpp_components)

add_library(footswitch_driver src/footswitch_driver.cpp)
target_compile_features(footswitch_driver PUBLIC cxx_std_20)

ament_target_dependencies(footswitch_driver ${ROS_DEPENDENCIES})

target_link_libraries(footswitch_driver ${HIDAPI_LIBRARIES})

add_executable(footswitch_driver_node src/footswitch_driver_node.cpp)

target_link_libraries(footswitch_driver_node footswitch_driver)

install(
  TARGETS footswitch_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS footswitch_driver_node RUNTIME DESTINATION lib/${PROJECT_NAME})
install(FILES ../LICENSE DESTINATION share/${PROJECT_NAME})
install(FILES ../scripts/19-footswitch.rules DESTINATION share/${PROJECT_NAME}/udev)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
