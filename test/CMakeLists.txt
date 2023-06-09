
# test target
add_executable(test_sacp test_sacp.cpp)
target_include_directories(test_sacp PRIVATE ${LOCAL_LIBS_DIR}/sacp/include)
target_link_libraries(test_sacp fmt::fmt sacp common)
add_dependencies(test_sacp sacp)

add_executable(test_sacpclient test_sacpclient.cpp ${LOCAL_LIBS_DIR}/sacp_client/src/sacp_client.cpp)
target_include_directories(test_sacpclient PRIVATE ${LOCAL_LIBS_DIR}/sacp/include ${LOCAL_LIBS_DIR}/sacp_client/include)
#target_link_libraries(test_sacpclient docopt_s fmt::fmt sacp common uv_a)
target_link_libraries(test_sacpclient fmt::fmt sacp common uv_a)
add_dependencies(test_sacpclient sacp)

