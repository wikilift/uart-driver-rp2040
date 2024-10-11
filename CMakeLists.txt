cmake_minimum_required(VERSION 3.12)

set(ProjectName uart_example)

add_executable(${ProjectName}
    examples/main.cpp
)

add_subdirectory(uart_driver)


target_include_directories(${ProjectName} PRIVATE
    ${GLOBAL_INCLUDE_PATH}  
    ${FREERTOS_KERNEL_PATH}/include
    ${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040
)

target_link_libraries(${ProjectName}
    UartDriver
    pico_stdlib
)

pico_enable_stdio_uart(${ProjectName} 1)
pico_enable_stdio_usb(${ProjectName} 1)

pico_add_extra_outputs(${ProjectName})
