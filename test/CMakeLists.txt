
## test target
add_executable(test_sacp test_sacp.cpp)
target_include_directories(test_sacp PRIVATE ${LOCAL_LIBS_DIR}/sacp/include)
target_link_libraries(test_sacp ${LIB_NOSCOMMON} ${LIB_LOGGER} fmt::fmt sacp)
add_dependencies(test_sacp sacp)


