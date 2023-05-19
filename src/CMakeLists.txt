
## 设定可执行文件名称
set(EXECUTABLE_NAME ${PROJECT_NAME})

find_library(LIB_NOSCOMMON noscommon PATHS ${NOS_COMMON_INSTALL_DIR}/lib)
message(STATUS "=> find LIB_NOSCOMMON=${LIB_NOSCOMMON}")

add_executable(${EXECUTABLE_NAME} main.cpp)
target_link_libraries(${EXECUTABLE_NAME} spdlog::spdlog uv docopt ${LIB_NOSCOMMON})

## set install 
install(TARGETS ${EXECUTABLE_NAME} DESTINATION bin)
