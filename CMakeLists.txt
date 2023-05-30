
#
# cmake rules for nos_common
# Author: LiuChuansen
# Date: 20230512
# 


cmake_minimum_required(VERSION 3.13)

project(nos_chassis)

message(STATUS "=> enter project ${PROJECT_NAME}")
message(STATUS "=> PROJECT_SOURCE_DIR=${PROJECT_SOURCE_DIR}")
message(STATUS "=> PROJECT_BINARY_DIR=${PROJECT_BINARY_DIR}")
message(STATUS "=> CMAKE_BINARY_DIR=${CMAKE_BINARY_DIR}")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

## General rules for all nos targets
if (NOT TOPDIR)
get_filename_component(TOPDIR ${PROJECT_SOURCE_DIR}/../ ABSOLUTE)
endif()
if (NOT NOS_INSTALL_PREFEX)
set(NOS_INSTALL_PREFEX      ${TOPDIR}/install/${CMAKE_SYSTEM_PROCESSOR})
endif()

## 本地安装目录
set(LOCAL_INSTALL_DIR       ${NOS_INSTALL_PREFEX})
## 本地的依赖库
set(LOCAL_LIBS_DIR          ${PROJECT_SOURCE_DIR}/libs)

message(STATUS "=> TOPDIR=${TOPDIR}")
message(STATUS "=> NOS_INSTALL_PREFEX=${NOS_INSTALL_PREFEX}")
message(STATUS "=> LOCAL_INSTALL_DIR=${LOCAL_INSTALL_DIR}")
## End 

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "-> Setting build type to 'RelWithDebInfo'")
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()


# CMAKE_PREFIX_PATH是一个环境变量，用于指定CMake可以搜索软件或库的目录列表。
# 它可以被find_package()、find_program()、find_library()、find_file()和find_path()等命令使用。
# 这些命令将根据自己的文档添加bin、lib或include等子目录。CMAKE_PREFIX_PATH可以包含单个前缀或多个前缀，
# UNIX上用冒号分隔，Windows上用分号分隔。默认情况下，它为空。它旨在由项目设置。
set(CMAKE_PREFIX_PATH ${NOS_INSTALL_PREFEX})

## 查找需要的库
find_package(libuv REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)
find_package(docopt REQUIRED)

## 安装目录
set(CMAKE_INSTALL_PREFIX ${LOCAL_INSTALL_DIR})

## set default include folder
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${NOS_INSTALL_PREFEX}/include
    )

## add local libs dir
link_directories(${NOS_INSTALL_PREFEX}/lib)

## add libs
add_subdirectory(libs/sacp)

## add src
add_subdirectory(src)

## add test
add_subdirectory(test)
