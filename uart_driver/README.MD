
# RP2040 UART Driver with FreeRTOS Support 🚀

This project provides a `UartDriver` class for managing UART communication on the RP2040 microcontroller using the Pico SDK. It offers interrupt-based UART handling, configurable UART parameters, and support for FreeRTOS tasks. This library is ideal for applications that require low-latency UART communication with event-based management.

## Cloning the Project

To get started, clone the project repository from GitHub:


```bash
git clone https://github.com/username/rp2040-uart-driver.git
cd rp2040-uart-driver 
```



## Requirements 📚

-   **Pico SDK**: The Raspberry Pi Pico SDK, which provides essential libraries and utilities for RP2040 development.
-   **FreeRTOS**: This library is designed to be used with FreeRTOS for managing tasks and handling UART events.
-   **GNU Arm Embedded Toolchain**: Necessary for compiling the firmware for the RP2040.
-   **CMake**: Used to generate build files for the project.

Ensure that your environment is set up with the correct paths for the Pico SDK and FreeRTOS. A typical setup might look like:
```bash
export PICO_SDK_PATH=/path/to/pico-sdk
export FREERTOS_KERNEL_PATH=/path/to/FreeRTOS-Kernel
```
In CMakeLists must be add this lie:
```cmake
pico_enable_stdio_uart(${ProjectName} 1)
```
## Features ✨

-   **Configurable UART Communication**: Supports setting baud rate, parity, data bits, and stop bits.
-   **Interrupt-Based Handling**: Uses interrupts to detect and handle UART events like data reception, frame errors, or overflows.
-   **FreeRTOS Integration**: Easily integrates with FreeRTOS for event-driven UART communication using task notifications.
-   **Circular Buffer Management**: Efficient data buffering for reliable UART data reception.
-   **Dynamic Baud Rate Adjustment**: Allows changing the baud rate at runtime.

## Building the Project 🛠️

Before using the library, compile the project manually. You can do this in one of two ways:

### Using `make` from the Command Line
```bash
mkdir -p build
cd build
cmake ..
make
```

### Using the Raspberry Pi Pico Extension in VSCode

-   Open the root folder of the project as your workspace.
-   Use the "Build" button from the Pico SDK extension toolbar.
-   Make sure the Raspberry Pi Pico SDK and `CMakeLists.txt` are correctly set up in your environment.

## Integration Guide 📖

### Including the Library in Your Project

To include the `UartDriver` library in your own project, add the path to the library in your `CMakeLists.txt`:

1.  Copy the `uart_driver` folder to your project's library directory.
2.  Include the `UartDriver` path in your `CMakeLists.txt`:

```cmake

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/path_to_uart_driver)
```

3.  Link the library to your executable:
```cmake
target_link_libraries(your_executable_name
    pico_stdlib
    hardware_uart
    FreeRTOS-Kernel
    UartDriver
)
```

### Library API 📜

The `UartDriver` class provides the following methods:

-   **Constructor**: `UartDriver(const uart_config_t &config)`
-   **Install Driver**: `bool install()`
-   **Deinitialize Driver**: `void deinit()`
-   **Send Data**: `void write(const uint8_t *data, size_t length)`
-   **Set Baud Rate**: `void set_baud_rate(uint baud_rate)`
-   **Read Data**: `size_t read_data(uint8_t *buffer, size_t buffer_size)`
-   **Get Event Type**: `uart_event_type_t get_event_type()`
-   **Flush Buffer**: `void flush()`

## Flashing the Firmware 📦

To flash the `.uf2` firmware file to the Raspberry Pi Pico, copy the file to the mounted Pico directory or use a flash script like:

```bash
cp build/your_firmware.uf2 /media/your_user/RPI-RP2/
```

For automatic flashing, you can use a [flash script ](https://github.com/wikilift/rp2040-freertos-cpp-starter)  described in the `rp2040-freertos-cpp-starter` project.

## Usage Example 👨‍💻
```cpp
#include "uart_driver.h"
#include "FreeRTOS.h"
#include "task.h"

uart_config_t uart_config = {
    .uart = uart0,
    .baud_rate = 115200,
    .tx_pin = 0,
    .rx_pin = 1,
    .data_bits = 8,
    .stop_bits = 1,
    .parity = UART_PARITY_NONE,
    .task_handle = nullptr,
    .timer_interval_ms = 10
};

UartDriver *uart_driver;

void uart_task(void *pvParameters)
{
    uint8_t buffer[UartDriver::BUFFER_SIZE];
	uint32_t  id_counter  =  0;
    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        size_t bytes_read = uart_driver->read_data(buffer, sizeof(buffer));
        if (bytes_read > 0)
        {
            printf("Data received: ");
            for (size_t i = 0; i < bytes_read; ++i)
            {
                printf("%c", buffer[i]);
            }
            printf("\n");
            char  response_buffer[50];

			snprintf(response_buffer, sizeof(response_buffer), "Received from RP2040 - ID: %u\n", id_counter);

			// Send the response message back via UART

			uart_driver->write(reinterpret_cast<const  uint8_t  *>(response_buffer), strlen(response_buffer));
			// Increment the ID counter for the next message

			id_counter++;
        }
    }
}

int main()
{
    stdio_init_all();

    TaskHandle_t uartTaskHandle;
    xTaskCreate(uart_task, "UART Task", 1024, NULL, tskIDLE_PRIORITY + 1, &uartTaskHandle);

    uart_config.task_handle = uartTaskHandle;
    uart_driver = new UartDriver(uart_config);
    if (!uart_driver->install())
    {
        printf("Failed to install UART driver.\n");
        return -1;
    }

    vTaskStartScheduler();
    return 0;
}
```

## License 📝

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for more details.