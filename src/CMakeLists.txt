
## 设定可执行文件名称
set(CHASSIS_NAME ${PROJECT_NAME})

set(ROBOT_N1_SRCS 
    robot_n1/attribute_helper.cpp
    robot_n1/controller_request.cpp 
    robot_n1/fogbox_request.cpp 
    robot_n1/ledlight_request.cpp 
    robot_n1/lifter_request.cpp 
    robot_n1/powerbox_request.cpp 
    robot_n1/pumpbox_request.cpp 
    robot_n1/pushbox_request.cpp)

add_executable(${CHASSIS_NAME} main.cpp sacp_client.cpp node_chassis.cpp node_manager.cpp ${ROBOT_N1_SRCS})
target_include_directories(${CHASSIS_NAME} PRIVATE ${LOCAL_LIBS_DIR}/sacp/include ${LOCAL_SRCS_DIR})
add_dependencies(${CHASSIS_NAME} sacp)
ament_target_dependencies(${CHASSIS_NAME} rclcpp naiad_interfaces)
target_link_libraries(${CHASSIS_NAME} fmt::fmt sacp common uv_a)

#rosidl_target_interfaces(my_node ${PROJECT_NAME} "rosidl_typesupport_cpp")


## set install 
install(TARGETS ${CHASSIS_NAME} 
    RUNTIME DESTINATION lib/${PROJECT_NAME})
