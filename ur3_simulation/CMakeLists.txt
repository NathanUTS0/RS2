cmake_minimum_required(VERSION 3.8)
project(ur3_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

add_executable(maze_solver src/maze_solver.cpp)

# Modern CMake way to set include directories
target_include_directories(maze_solver PUBLIC ${OpenCV_INCLUDE_DIRS})

# Linking dependencies
ament_target_dependencies(maze_solver rclcpp sensor_msgs cv_bridge OpenCV moveit_ros_planning_interface gazebo_ros)

# OpenCV linking
target_link_libraries(maze_solver ${OpenCV_LIBS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
maze_solver
DESTINATION lib/${PROJECT_NAME}
)


ament_package()
