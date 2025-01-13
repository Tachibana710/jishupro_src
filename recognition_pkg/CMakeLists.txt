cmake_minimum_required(VERSION 3.8)
project(recognition_pkg)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_BUILD_TYPE Release)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

file(GLOB_RECURSE SRC_FILES ${CMAKE_SOURCE_DIR}/src/*/*/*/*.cpp)
include_directories(${CMAKE_SOURCE_DIR}/src/myCNN/include)
add_executable(recog_node src/myCNN/src/main.cpp ${SRC_FILES})
install(TARGETS recog_node DESTINATION lib/${PROJECT_NAME})

find_package(Eigen3 REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(my_msgs REQUIRED)

# target_link_libraries(recog_node Eigen3::Eigen rclcpp sensor_msgs std_msgs)
ament_target_dependencies(recog_node rclcpp sensor_msgs std_msgs my_msgs Eigen3)

target_link_libraries(recog_node Eigen3::Eigen)

ament_export_dependencies(rosidl_default_runtime)

install(
  PROGRAMS
  scripts/recognition_node.py  # インストールするPythonスクリプト
  scripts/camera_calib.py
DESTINATION lib/${PROJECT_NAME}  # インストール先ディレクトリ
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
