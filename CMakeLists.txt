
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

## 设定可执行文件名称
set(EXECUTABLE_NAME ${PROJECT_NAME})

## 配置NOS顶层目录
get_filename_component(TOPDIR ${PROJECT_SOURCE_DIR}/../ ABSOLUTE)
## nos_common 目录
set(NOS_COMMON_DIR ${TOPDIR}/nos_common)
message(STATUS "=> set TOPDIR=${TOPDIR}")

## 查找依赖的库
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)
find_library(LIB_UV uv REQUIRED)
find_package(docopt COMPONENTS CXX REQUIRED)
include_directories(${DOCOPT_INCLUDE_DIRS})

## 找到noscommon
message(STATUS "=> get NOS_BUILD=${NOS_BUILD}")

if (NOS_BUILD)
# 在NOS BUILD目录下，需要CMAKE决定依赖
set(LIB_NOSCOMMON noscommon)
else()
find_library(LIB_NOSCOMMON noscommon PATHS ${NOS_COMMON_DIR}/build/lib)
endif()

message(STATUS "=> find LIB_NOSCOMMON=${LIB_NOSCOMMON}")

## set default include folder
include_directories(${PROJECT_SOURCE_DIR}/include  ${NOS_COMMON_DIR}/include)

add_executable(${EXECUTABLE_NAME} src/main.cpp)

target_link_libraries(${EXECUTABLE_NAME} spdlog::spdlog ${LIB_NOSCOMMON} ${LIB_UV})


add_executable(test_docopt src/test_docopt.cpp)
target_link_libraries(test_docopt docopt)


## set install 
install(TARGETS ${EXECUTABLE_NAME} DESTINATION bin)
