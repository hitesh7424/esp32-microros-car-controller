cmake_minimum_required(VERSION 3.8)
project(esp32_car)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# Python integration
find_package(ament_cmake_python REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)



add_executable(hello src/hello.cpp)
add_executable(camera_node src/camera_node.cpp)
add_executable(wheel_speed_controller src/wheel_speed_controller.cpp)

target_include_directories(hello PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(hello PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(camera_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${OpenCV_INCLUDE_DIRS}
)

target_include_directories(wheel_speed_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_compile_features(wheel_speed_controller PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(wheel_speed_controller rclcpp geometry_msgs)

target_compile_features(camera_node PUBLIC c_std_99 cxx_std_17)
target_link_libraries(camera_node ${OpenCV_LIBS} curl)
ament_target_dependencies(camera_node rclcpp sensor_msgs image_transport)

install(TARGETS 
  hello
  wheel_speed_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Python package and scripts
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/controller.py
  scripts/joystick_to_wheels.py
  scripts/wheel_speed_publisher.py
  DESTINATION lib/${PROJECT_NAME}
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
