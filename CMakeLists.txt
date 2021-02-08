cmake_minimum_required(VERSION 3.5)
project(navigation_1)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(osqp REQUIRED)

include_directories(
  /usr/include/eigen3
  include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(generate_map src/generate_map.cpp)
ament_target_dependencies(generate_map rclcpp std_msgs nav_msgs)

add_executable(a_star src/A_star.cpp)
ament_target_dependencies(a_star rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs)

add_executable(opt_path_in_gallerys src/opt_path_in_gallerys.cpp)
ament_target_dependencies(opt_path_in_gallerys rclcpp std_msgs nav_msgs geometry_msgs visualization_msgs sensor_msgs)
target_link_libraries(opt_path_in_gallerys osqp::osqp)

add_executable(opt_path_in_gallerys2 src/opt_path_in_gallerys2.cpp)
ament_target_dependencies(opt_path_in_gallerys2 rclcpp std_msgs nav_msgs geometry_msgs visualization_msgs sensor_msgs)
target_link_libraries(opt_path_in_gallerys2 osqp::osqp)

install(TARGETS
  generate_map
  a_star
  opt_path_in_gallerys
  opt_path_in_gallerys2
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
