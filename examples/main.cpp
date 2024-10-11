/**
 * @file main.cpp
 * @brief Example application using the UartDriver class with FreeRTOS.
 *
 * This example demonstrates the use of the `UartDriver` class for managing UART
 * communication using FreeRTOS tasks. It includes an interrupt-based UART driver
 * with a FreeRTOS timer for handling data reception and event notifications.
 *
 * The application consists of two tasks:
 * 1. `uart_task`: Handles UART events such as data reception and error detection.
 * 2. `led_task`: A simple task to toggle an LED on and off.
 *
 * The program sets up the UART driver, creates the tasks, and starts the FreeRTOS scheduler.
 *
 * @author
 * Daniel Giménez
 * @date 2024-10-11
 *
 * @license MIT License
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "uart_driver.h"
#include <string.h>

/**
 * @brief Task to toggle the onboard LED.
 *
 * This task toggles the onboard LED (PICO_DEFAULT_LED_PIN) at a frequency of 200 ms.
 * It serves as a basic task for visual indication of task scheduling.
 *
 * @param pvp Unused parameter.
 */
void led_task(void *pvp);

/**
 * @brief UART configuration structure.
 *
 * This configuration sets up the UART0 instance with a baud rate of 115200,
 * TX pin 0, RX pin 1, and 8 data bits, 1 stop bit, and no parity.
 * The FreeRTOS task handle is assigned later during task creation.
 */
uart_config_t uart_config = {
    .uart = uart0,              /**< UART instance (e.g., uart0) */
    .baud_rate = 115200,        /**< UART baud rate */
    .tx_pin = 0,                /**< TX pin number */
    .rx_pin = 1,                /**< RX pin number */
    .data_bits = 8,             /**< Number of data bits */
    .stop_bits = 1,             /**< Number of stop bits */
    .parity = UART_PARITY_NONE, /**< No parity */
    .task_handle = nullptr,     /**< FreeRTOS task handle, assigned later */
    .timer_interval_ms = 10     /**< Timer interval in milliseconds */
};

/**
 * @brief Pointer to the UART driver instance.
 *
 * This pointer is used to reference the `UartDriver` object throughout the application.
 */
UartDriver *uart_driver = nullptr;

/**
 * @brief FreeRTOS task for handling UART communication.
 *
 * This task waits for notifications from the UART driver and processes events such as
 * data reception, break conditions, and UART errors. Depending on the event type, it
 * prints the appropriate message or processes received data.
 *
 * @param pvParameters Unused parameter.
 */
void uart_task(void *pvParameters)
{
    uint8_t buffer[UartDriver::BUFFER_SIZE];
    uint32_t id_counter = 0;
    for (;;)
    {
        // Wait indefinitely for a notification from the UART interrupt or timer.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Get the current event type from the UART driver
        uart_event_type_t event_type = uart_driver->get_event_type();

        switch (event_type)
        {
        case UART_DATA:
        {
            // Read available data from the UART buffer
            size_t bytes_read = uart_driver->read_data(buffer, sizeof(buffer));

            if (bytes_read > 0)
            {
                // Print the received data as a string
                printf("Data received (%u bytes): ", (unsigned int)bytes_read);
                for (size_t i = 0; i < bytes_read; ++i)
                {
                    printf("%c", buffer[i]);
                }
                printf("\n");
                // Create a formatted response with the ID counter
                char response_buffer[50];
                snprintf(response_buffer, sizeof(response_buffer), "Received from RP2040 - ID: %u\n", id_counter);

                // Send the response message back via UART
                uart_driver->write(reinterpret_cast<const uint8_t *>(response_buffer), strlen(response_buffer));

                // Increment the ID counter for the next message
                id_counter++;
            }
            break;
        }
        case UART_BREAK:
            printf("UART BREAK detected\n");
            break;
        case UART_PARITY_ERROR:
            printf("Parity error on UART\n");
            break;
        case UART_FRAME_ERROR:
            printf("Frame error on UART\n");
            break;
        case UART_OVERFLOW:
            printf("Buffer overflow on UART\n");
            break;
        default:
            break;
        }
    }
}

/**
 * @brief Main function to initialize the UART driver and start FreeRTOS.
 *
 * This function initializes the stdio, sets up the UART driver with the configuration
 * defined in `uart_config`, and creates the FreeRTOS tasks (`uart_task` and `led_task`).
 * After creating the tasks, it starts the FreeRTOS scheduler to begin task execution.
 *
 * @return int Return 0 if successful, -1 if the UART driver installation fails.
 */
extern "C" int main()
{
    // Initialize the stdio for UART output (e.g., printf)
    stdio_init_all();
    sleep_ms(5000); // Delay for 5 seconds to stabilize the system before starting
    printf("Starting FreeRTOS scheduler\n");

    printf("Installing the UART driver\n");

    // Create the UART task and store its handle in `uartTaskHandle`
    TaskHandle_t uartTaskHandle;
    xTaskCreate(uart_task, "UART Task", 1024, NULL, tskIDLE_PRIORITY + 1, &uartTaskHandle);

    // Assign the task handle to the UART configuration
    uart_config.task_handle = uartTaskHandle;

    // Instantiate the UART driver object
    uart_driver = new UartDriver(uart_config);
    if (!uart_driver->install())
    {
        printf("Error installing driver\n");
        return -1;
    }

    // Create a simple LED task for toggling the onboard LED
    xTaskCreate(led_task, "LED Task", 256, NULL, tskIDLE_PRIORITY + 1, NULL);
    printf("UART driver successfully installed\n");

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();
    return 0;
}

/**
 * @brief FreeRTOS task to toggle the onboard LED.
 *
 * This task toggles the onboard LED (PICO_DEFAULT_LED_PIN) at a frequency of 200 ms.
 * It serves as a simple visual indicator to show that FreeRTOS tasks are running.
 *
 * @param pvp Unused parameter.
 */
void led_task(void *pvp)
{
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    while (true)
    {
        gpio_put(LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(200));
        gpio_put(LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
