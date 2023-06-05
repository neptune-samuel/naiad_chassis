
## 设定可执行文件名称
set(CHASSIS_NAME ${PROJECT_NAME})

add_executable(${CHASSIS_NAME} main.cpp sacp_client.cpp)
target_include_directories(${CHASSIS_NAME} PRIVATE ${LOCAL_LIBS_DIR}/sacp/include)
target_link_libraries(${CHASSIS_NAME} uv_a docopt_s fmt::fmt sacp common)
add_dependencies(${CHASSIS_NAME} sacp)

## set install 
install(TARGETS ${CHASSIS_NAME} DESTINATION bin)
