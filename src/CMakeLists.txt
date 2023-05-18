
## 设定可执行文件名称
set(EXECUTABLE_NAME ${PROJECT_NAME})

if (NOS_BUILD)
# 在NOS BUILD目录下，需要CMAKE决定依赖
set(LIB_NOSCOMMON noscommon)
else()
find_library(LIB_NOSCOMMON noscommon PATHS ${NOS_COMMON_INSTALL_DIR}/lib)
endif()

message(STATUS "=> find LIB_NOSCOMMON=${LIB_NOSCOMMON}")

add_executable(${EXECUTABLE_NAME} main.cpp)
target_link_libraries(${EXECUTABLE_NAME} spdlog::spdlog uv ${LIB_NOSCOMMON})

#add_executable(test_docopt src/test_docopt.cpp)
#target_link_libraries(test_docopt docopt)

## set install 
install(TARGETS ${EXECUTABLE_NAME} DESTINATION bin)
