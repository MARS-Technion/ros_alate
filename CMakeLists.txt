cmake_minimum_required(VERSION 3.8)
project(ros_alate)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ros_alate_interfaces REQUIRED)

########################## Additional Packages #########################

# Add Boost
set(BOOST_ROOT ${CMAKE_FIND_ROOT_PATH} CACHE PATH "Hint to where the proper Boost is installed")
find_package(Boost 1.71.0 REQUIRED COMPONENTS "system" "serialization" "thread" "filesystem" "date_time")
if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
else()
  message(FATAL_ERROR "Could not find boost!")
endif()

#Add NeMALA
find_library(NeMALA_LIBRARY NAMES libNeMALA.a libNeMALA.so PATHS "/usr/local/lib/")
include_directories("/usr/local/include/")

################################# Libraries #############################

set (EXTRA_LIBS ${EXTRA_LIBS} ${Boost_LIBRARIES}  ${NeMALA_LIBRARY})

#############################################################

add_executable(adapter src/ros_alate_adapter.cpp src/adapter.cpp)
ament_target_dependencies(
	adapter
	rclcpp
	geometry_msgs
	ros_alate_interfaces
)
target_link_libraries (adapter ${EXTRA_LIBS})

target_include_directories(adapter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(adapter PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS adapter
  DESTINATION lib/${PROJECT_NAME})

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
