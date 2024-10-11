#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "uart_driver/include/uart_driver.h"
#include <string.h>


void led_task(void *pvp);

uart_config_t uart_config = {
    .uart = uart0,
    .baud_rate = 115200,
    .tx_pin = 0,
    .rx_pin = 1,
    .data_bits = 8,
    .stop_bits = 1,
    .parity = UART_PARITY_NONE,
    .task_handle = nullptr,
    .timer_interval_ms = 10};

UartDriver *uart_driver = nullptr;

void uart_task(void *pvParameters)
{
    uint8_t buffer[UartDriver::BUFFER_SIZE];

    for (;;)
    {

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        uart_event_type_t event_type = uart_driver->get_event_type();

        switch (event_type)
        {
        case UART_DATA:
        {

            size_t bytes_read = uart_driver->read_data(buffer, sizeof(buffer));

            if (bytes_read > 0)
            {
                printf("Data recv (%u bytes): ", (unsigned int)bytes_read);
                for (size_t i = 0; i < bytes_read; ++i)
                {
                    printf("%c", buffer[i]);
                }
                printf("\n");
            }
            break;
        }
        case UART_BREAK:
            printf("UART BREAK detectec\n");
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




extern "C" int main()
{
    stdio_init_all();
    sleep_ms(5000);
    printf("Starting FreeRTOS scheduler\n");

    printf("Instalando el driver UART\n");

    TaskHandle_t uartTaskHandle;
    xTaskCreate(uart_task, "UART Task", 1024, NULL, tskIDLE_PRIORITY + 1, &uartTaskHandle);

    uart_config.task_handle = uartTaskHandle;

    uart_driver = new UartDriver(uart_config);
    if (!uart_driver->install())
    {
        printf("Error installing driver\n");
        return -1;
    }

    xTaskCreate(led_task, "LED Task", 256, NULL, tskIDLE_PRIORITY + 1, NULL);
    printf("UART driver succesfully installed\n");

    vTaskStartScheduler();
    return 0;
}


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
