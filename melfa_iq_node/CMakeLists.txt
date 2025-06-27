cmake_minimum_required(VERSION 3.8)
project(melfa_iq_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(melfa_msgs REQUIRED)
find_package(melfa_iq_msgs REQUIRED)
find_package(std_msgs REQUIRED)


add_executable(iq_ src/iq_.cpp)
target_include_directories(iq_ PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(iq_ PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(iq_ ${THIS_PACKAGE_INCLUDE_DEPENDS} melfa_msgs melfa_iq_msgs rclcpp)


install(TARGETS iq_
DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)

ament_lint_auto_find_test_dependencies()
endif()

ament_package()
