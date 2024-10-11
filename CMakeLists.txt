add_executable(${ProjectName}
    examples/main.cpp
    uart_driver/src/uart_driver.cpp 
   
)

target_include_directories(${ProjectName} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${FREERTOS_KERNEL_PATH}/include
    ${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)


target_link_libraries(${ProjectName} 
    pico_stdlib 
    FreeRTOS-Kernel-Heap4 
    hardware_uart
    hardware_dma
    )

pico_enable_stdio_uart(${ProjectName} 1)
pico_enable_stdio_usb(${ProjectName} 1)

pico_add_extra_outputs(${ProjectName})