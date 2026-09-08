cmake_minimum_required(VERSION 3.8)
project(quadrotor_gnc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================================
# Dependencies
# ============================================================

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)


# ============================================================
# Generate custom ROS 2 interfaces
# ============================================================

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Measurement.msg"
  "msg/EstimatedState.msg"
  "msg/Control.msg"
)


# ============================================================
# Get C++ typesupport target
# ============================================================

rosidl_get_typesupport_target(
  cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)


# ============================================================
# Kalman Filter library
# ============================================================

add_library(kalman_filter
  src/kalman_filter.cpp
)

target_include_directories(kalman_filter PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(
  kalman_filter
  Eigen3::Eigen
)


# ============================================================
# LQR Controller library
# ============================================================

add_library(lqr_controller
  src/lqr_controller.cpp
)

target_include_directories(lqr_controller PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(
  lqr_controller
  Eigen3::Eigen
)


# ============================================================
# Estimator node
# ============================================================

add_executable(estimator_node
  src/estimator_node.cpp
)

ament_target_dependencies(
  estimator_node
  rclcpp
)

add_dependencies(
  estimator_node
  ${PROJECT_NAME}__rosidl_generator_cpp
)

target_link_libraries(
  estimator_node
  "${cpp_typesupport_target}"
  kalman_filter
)

set_target_properties(
  estimator_node
  PROPERTIES
    INSTALL_RPATH "$ORIGIN"
)


# ============================================================
# Controller node
# ============================================================

add_executable(controller_node
  src/controller_node.cpp
)

ament_target_dependencies(
  controller_node
  rclcpp
)

add_dependencies(
  controller_node
  ${PROJECT_NAME}__rosidl_generator_cpp
)

target_link_libraries(
  controller_node
  "${cpp_typesupport_target}"
  lqr_controller
)

set_target_properties(
  controller_node
  PROPERTIES
    INSTALL_RPATH "$ORIGIN"
)


# ============================================================
# Plant node
# ============================================================

add_executable(plant_node
  src/plant_node.cpp
)

ament_target_dependencies(
  plant_node
  rclcpp
)

add_dependencies(
  plant_node
  ${PROJECT_NAME}__rosidl_generator_cpp
)

target_link_libraries(
  plant_node
  "${cpp_typesupport_target}"
  Eigen3::Eigen
)


# ============================================================
# Install executables and libraries
# ============================================================

install(
  TARGETS
    estimator_node
    controller_node
    plant_node
    kalman_filter
    lqr_controller
  DESTINATION lib/${PROJECT_NAME}
)


# ============================================================
# Install launch files
# ============================================================

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


# ============================================================
# Install config files
# ============================================================

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)


# ============================================================
# Finalize package
# ============================================================

# ============================================================
# Tests
# ============================================================

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_lqr_controller
    test/test_lqr_controller.cpp
  )

  target_include_directories(test_lqr_controller PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  target_link_libraries(test_lqr_controller
    lqr_controller
    Eigen3::Eigen
  )
endif()

ament_package()