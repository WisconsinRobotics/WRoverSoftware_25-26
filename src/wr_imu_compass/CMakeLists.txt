cmake_minimum_required(VERSION 3.8)
project(wr_imu_compass)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Threads REQUIRED)
find_package(SDL2 REQUIRED)
find_package(phoenix6 REQUIRED) # Ensure this is the correct package name
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(compass src/compass.cpp)
ament_target_dependencies(compass rclcpp std_msgs phoenix6)


target_include_directories(compass PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(compass PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS compass
  DESTINATION lib/${PROJECT_NAME})

  # Link against the correct libraries
target_link_libraries(compass phoenix6 Threads::Threads ${SDL2_LIBRARIES})

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
