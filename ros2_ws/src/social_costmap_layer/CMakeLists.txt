cmake_minimum_required(VERSION 3.8)
project(social_costmap_layer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(human_interfaces REQUIRED)

# The cost function has no ROS dependency, so it is built and tested on its own.
add_library(social_cost SHARED src/social_cost.cpp)
target_include_directories(social_cost PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

add_library(${PROJECT_NAME} SHARED src/social_layer.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(${PROJECT_NAME} social_cost)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  nav2_costmap_2d
  pluginlib
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  human_interfaces)

pluginlib_export_plugin_description_file(nav2_costmap_2d social_layer.xml)

install(TARGETS social_cost ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(FILES social_layer.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_social_cost test/test_social_cost.cpp)
  target_link_libraries(test_social_cost social_cost)

  find_package(ament_cmake_clang_format REQUIRED)
  ament_clang_format(CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/.clang-format)
endif()

ament_package()
