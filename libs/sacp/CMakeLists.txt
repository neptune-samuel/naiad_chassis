
## 设定可执行文件名称
set(LIBRARY_NAME sacp)

set(LIBSACP_DIR ${PROJECT_SOURCE_DIR}/libs/libsacp)

add_library(${LIBRARY_NAME} 
    ${LIBSACP_DIR}/portable.c 
    ${LIBSACP_DIR}/sacp_frame.c
    src/attribute.cpp 
    src/frame.cpp 
    src/stream.cpp 
    )

target_compile_definitions(${LIBRARY_NAME} PRIVATE __no_framework__ __build_arch64__)
target_include_directories(${LIBRARY_NAME} PRIVATE ${LOCAL_LIBS_DIR} include)

## set install 
install(TARGETS ${LIBRARY_NAME} DESTINATION lib)
