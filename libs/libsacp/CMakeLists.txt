
## 设定可执行文件名称
set(LIBRARY_NAME sacp)

add_library(${LIBRARY_NAME} portable.c sacp_frame.c)
target_compile_definitions(${LIBRARY_NAME} PUBLIC __no_framework__)

## set install 
install(TARGETS ${LIBRARY_NAME} DESTINATION lib)

