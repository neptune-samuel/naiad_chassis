
## 设定可执行文件名称
set(EXECUTABLE_NAME ${PROJECT_NAME})

#find_library(LIB_LOGGER logger PATHS ${WS_LIBRARY_PREFIX}/lib)
find_library(LIB_NOSCOMMON noscommon PATHS ${WS_LIBRARY_PREFIX}/lib)
#message(STATUS "=> find LIB_LOGGER=${LIB_LOGGER}")
message(STATUS "=> find LIB_NOSCOMMON=${LIB_NOSCOMMON}")

add_executable(${EXECUTABLE_NAME} main.cpp sacp_client.cpp)
target_include_directories(${EXECUTABLE_NAME} PRIVATE ${LOCAL_LIBS_DIR}/sacp/include)
target_link_libraries(${EXECUTABLE_NAME} uv_a docopt_s ${LIB_NOSCOMMON} fmt::fmt sacp)
target_compile_definitions()
add_dependencies(${EXECUTABLE_NAME} sacp)

## set install 
install(TARGETS ${EXECUTABLE_NAME} DESTINATION bin)
