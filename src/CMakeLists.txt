
## 设定可执行文件名称
set(EXECUTABLE_NAME ${PROJECT_NAME})

add_executable(${EXECUTABLE_NAME} main.cpp sacp_client.cpp)
target_include_directories(${EXECUTABLE_NAME} PRIVATE ${LOCAL_LIBS_DIR}/sacp/include)
target_link_libraries(${EXECUTABLE_NAME} uv_a docopt_s fmt::fmt sacp common)
add_dependencies(${EXECUTABLE_NAME} sacp)

## set install 
install(TARGETS ${EXECUTABLE_NAME} DESTINATION bin)
