
## test target
add_executable(test_sacp test_sacp.cpp)
target_include_directories(test_sacp PRIVATE ${LOCAL_LIBS_DIR}/sacp/include)
target_link_libraries(test_sacp fmt::fmt sacp common)
add_dependencies(test_sacp sacp)

add_executable(test_sacpclient test_sacp_client.cpp ${LOCAL_SRCS_DIR}/sacp_client.cpp)
target_include_directories(test_sacpclient PRIVATE ${LOCAL_LIBS_DIR}/sacp/include)
target_link_libraries(test_sacpclient uv_a docopt_s fmt::fmt sacp common)
add_dependencies(test_sacpclient sacp)

