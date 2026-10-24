cmake_minimum_required(VERSION 3.8)
project(lidar_dock_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(include)

set(DETECTOR_SOURCES
  src/config.cpp
  src/detection_filter.cpp
  src/landmark_matcher.cpp
  src/line_extractor.cpp
  src/model.cpp
  src/nav2_pose_publisher.cpp
  src/pose_estimator.cpp
  src/region_splitter.cpp
  src/scan_processor.cpp
  src/detector_node.cpp
)

add_executable(lidar_dock_detector_node ${DETECTOR_SOURCES})
target_link_libraries(lidar_dock_detector_node Eigen3::Eigen)
ament_target_dependencies(
  lidar_dock_detector_node
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  visualization_msgs
)

install(TARGETS lidar_dock_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(test_detection_filter
    test/test_detection_filter.cpp
    src/detection_filter.cpp
  )
  if(TARGET test_detection_filter)
    target_link_libraries(test_detection_filter Eigen3::Eigen)
    ament_target_dependencies(test_detection_filter rclcpp)
  endif()
  ament_add_gtest(test_config_validation
    test/test_config_validation.cpp
    src/config.cpp
    src/model.cpp
  )
  if(TARGET test_config_validation)
    target_link_libraries(test_config_validation Eigen3::Eigen)
    ament_target_dependencies(test_config_validation rclcpp)
  endif()
endif()

ament_package()
