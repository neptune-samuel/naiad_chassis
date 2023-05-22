
cmake_minimum_required(VERSION 3.10)

project(nos_chassis)

message(STATUS "=> enter project ${PROJECT_NAME}")
message(STATUS "=> PROJECT_SOURCE_DIR=${PROJECT_SOURCE_DIR}")
message(STATUS "=> PROJECT_BINARY_DIR=${PROJECT_BINARY_DIR}")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

## 设定一些目录
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

## 配置NOS顶层目录
get_filename_component(TOPDIR ${PROJECT_SOURCE_DIR}/../ ABSOLUTE)
## nos_common 目录
set(NOS_COMMON_DIR ${TOPDIR}/nos_common)
set(NOS_COMMON_INSTALL_DIR ${NOS_COMMON_DIR}/local-install/${CMAKE_SYSTEM_PROCESSOR})
set(NOS_COMMON_LIBS_INSTALL_DIR ${NOS_COMMON_DIR}/libs-install/${CMAKE_SYSTEM_PROCESSOR})


message(STATUS "=> set TOPDIR=${TOPDIR}")
message(STATUS "=> set NOS_COMMON_DIR=${NOS_COMMON_DIR}")
message(STATUS "=> set NOS_COMMON_INSTALL_DIR=${NOS_COMMON_INSTALL_DIR}")
message(STATUS "=> set NOS_COMMON_LIBS_INSTALL_DIR=${NOS_COMMON_LIBS_INSTALL_DIR}")
message(STATUS "=> get NOS_BUILD=${NOS_BUILD}")

## 设置本地安装目录
set(LOCAL_INSTALL_DIR ${PROJECT_SOURCE_DIR}/local-install/${CMAKE_SYSTEM_PROCESSOR})
message(STATUS "=> Local install at ${LOCAL_INSTALL_DIR}")

# CMAKE_PREFIX_PATH是一个环境变量，用于指定CMake可以搜索软件或库的目录列表。
# 它可以被find_package()、find_program()、find_library()、find_file()和find_path()等命令使用。
# 这些命令将根据自己的文档添加bin、lib或include等子目录。CMAKE_PREFIX_PATH可以包含单个前缀或多个前缀，
# UNIX上用冒号分隔，Windows上用分号分隔。默认情况下，它为空。它旨在由项目设置。
set(CMAKE_PREFIX_PATH ${NOS_COMMON_INSTALL_DIR} ${NOS_COMMON_LIBS_INSTALL_DIR})

## 查找需要的库
find_package(libuv REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)
find_package(docopt REQUIRED)

## 设定一些公共的环境变量
set(CMAKE_INSTALL_PREFIX ${LOCAL_INSTALL_DIR})

## set default include folder
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${NOS_COMMON_INSTALL_DIR}/include
    ${NOS_COMMON_LIBS_INSTALL_DIR}/include
    )
## add local libs dir
link_directories(${NOS_COMMON_INSTALL_DIR}/lib
    ${NOS_COMMON_LIBS_INSTALL_DIR}/lib
    )

## add src
add_subdirectory(src)
