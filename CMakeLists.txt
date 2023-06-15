#
# cmake rules for naiad_chassis
# Author: LiuChuansen
# Date: 20230512
# 
# 
cmake_minimum_required(VERSION 3.10)

project(naiad_chassis VERSION 0.1)

set(RULES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/rules)
set(LOCAL_LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs)
set(LOCAL_SRCS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

include(${RULES_DIR}/ws_common.cmake)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  #add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-Wall -Wextra)
endif()

if (WS_ROS)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(naiad_interfaces REQUIRED PATHS ${WS_INSTALL_PREFIX})

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
endif()

## ROS环境下，安装目录由ROS指定, 非ROS环境下，需自行设定默认安装目录
if (NOT WS_ROS)
set(CMAKE_INSTALL_PREFIX ${WS_INSTALL_PREFIX})
endif()

find_package(libuv REQUIRED PATHS ${WS_INSTALL_PREFIX})
if (NOT WS_ROS)
find_package(spdlog REQUIRED PATHS ${WS_INSTALL_PREFIX})
endif()
find_package(fmt REQUIRED PATHS ${WS_INSTALL_PREFIX})
find_package(docopt REQUIRED PATHS ${WS_INSTALL_PREFIX})

find_package(common REQUIRED PATHS ${WS_INSTALL_PREFIX})

message(STATUS "=> found spdlog : ${spdlog_VERSION}")
message(STATUS "=> found libuv  : ${libuv_VERSION}")
message(STATUS "=> found docopt : ${docopt_VERSION}")
message(STATUS "=> found fmt    : ${fmt_VERSION}")
message(STATUS "=> found common : ${common_VERSION}")

## set default include folder
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${WS_INSTALL_PREFIX}/include)

if (WS_ROS)
include_directories(    
    ${ROS_ROOT_DIR}/include)
endif()

## add local libs dir
link_directories(${WS_INSTALL_PREFIX}/lib)
## add libs/sacp
add_subdirectory(${LOCAL_LIBS_DIR}/sacp)

if (WS_ROS)
# add src 
add_subdirectory(src)
endif()

# add test
add_subdirectory(test)

if (WS_ROS)
ament_package()
endif()
