
# test target
add_executable(test_sacp test_sacp.cpp)
target_include_directories(test_sacp PRIVATE ${LOCAL_LIBS_DIR}/sacp/include)
target_link_libraries(test_sacp fmt::fmt sacp common)
add_dependencies(test_sacp sacp)

add_executable(sacp_client test_sacpclient.cpp ${LOCAL_LIBS_DIR}/sacp_client/src/sacp_client.cpp)
target_include_directories(sacp_client PRIVATE ${LOCAL_LIBS_DIR}/sacp/include ${LOCAL_LIBS_DIR}/sacp_client/include)
target_link_libraries(sacp_client fmt::fmt sacp common uv_a)
add_dependencies(sacp_client sacp)

