
## 设定可执行文件名称
set(CHASSIS_NAME ${PROJECT_NAME})

add_executable(${CHASSIS_NAME} main.cpp sacp_client.cpp node_manager.cpp)
target_include_directories(${CHASSIS_NAME} PRIVATE ${LOCAL_LIBS_DIR}/sacp/include)
add_dependencies(${CHASSIS_NAME} sacp)
ament_target_dependencies(${CHASSIS_NAME} rclcpp naiad_interfaces)
target_link_libraries(${CHASSIS_NAME} fmt::fmt sacp common uv_a)

#rosidl_target_interfaces(my_node ${PROJECT_NAME} "rosidl_typesupport_cpp")


## set install 
install(TARGETS ${CHASSIS_NAME} 
    RUNTIME DESTINATION lib/${PROJECT_NAME})
