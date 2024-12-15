cmake_minimum_required(VERSION 3.8)
project(lunatic_exchange)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(lunatic_exchange src/lunatic_exchange.cpp)
target_include_directories(lunatic_exchange PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(lunatic_exchange PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  lunatic_exchange
  "moveit_ros_planning_interface"
  "rclcpp"
)


# Install all files to share/
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/ )
install(DIRECTORY meshes  DESTINATION share/${PROJECT_NAME}/ )
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})


# Binary:
install(TARGETS lunatic_exchange DESTINATION lib/${PROJECT_NAME})

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

