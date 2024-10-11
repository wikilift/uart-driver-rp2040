cmake_minimum_required(VERSION 3.13)

# Define the UartDriver library
add_library(UartDriver STATIC
    src/uart_driver.cpp
)


target_include_directories(UartDriver PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${GLOBAL_INCLUDE_PATH}  
)

# Link dependencies for UartDriver
target_link_libraries(UartDriver
    pico_stdlib
    hardware_uart
    FreeRTOS-Kernel-Heap4
)
