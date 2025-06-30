cmake_minimum_required(VERSION 3.10)
project(uart_parse_packet)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置输出目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# 添加可执行文件
add_executable(uart_parse_packet protocol.cpp serial_port.cpp test_protocol.cpp)

# Windows系统特定设置
if(WIN32)
    # 链接Windows特定库
    target_link_libraries(test_protocol winmm)
endif()

# 安装目标
install(TARGETS test_protocol DESTINATION bin)

# 打印信息
message(STATUS "CMAKE_SYSTEM_NAME: ${CMAKE_SYSTEM_NAME}")
message(STATUS "CMAKE_SYSTEM_VERSION: ${CMAKE_SYSTEM_VERSION}")
message(STATUS "CMAKE_CXX_COMPILER_ID: ${CMAKE_CXX_COMPILER_ID}")
message(STATUS "CMAKE_CXX_COMPILER_VERSION: ${CMAKE_CXX_COMPILER_VERSION}") 