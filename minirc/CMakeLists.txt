## 设定可执行文件名称
set(MINIRC_NAME naiad_minirc)

add_executable(${MINIRC_NAME} 
    main.cpp 
    node_minirc.cpp)
ament_target_dependencies(${MINIRC_NAME} rclcpp naiad_interfaces)
target_link_libraries(${MINIRC_NAME} fmt::fmt sacp common uv_a)

## set install 
install(TARGETS ${MINIRC_NAME} 
    RUNTIME DESTINATION lib/${PROJECT_NAME})
