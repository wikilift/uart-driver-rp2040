/**
 * @file uart_driver.h
 * @brief UartDriver class for handling UART communication with interrupt-based management.
 *
 *
 *
 *
 * @author Daniel Giménez
 * @date 2024-10-11
 * @license MIT License
 *
 * @par License:
 *
 * MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "uart_driver/include/uart_driver.h"
#include <cstring>
#include <stdio.h>

UartDriver *UartDriver::instances_[2] = {nullptr};

UartDriver::UartDriver(const uart_config_t &config)
    : config_(config), event_type_(UART_UNKNOWN), rx_head_(0), rx_tail_(0)
{
}

UartDriver::~UartDriver()
{
    deinit();
}

bool UartDriver::install()
{
    uart_init(config_.uart, config_.baud_rate);
    uart_set_format(config_.uart, config_.data_bits, config_.stop_bits, config_.parity);
    uart_set_fifo_enabled(config_.uart, true);
    gpio_set_function(config_.tx_pin, GPIO_FUNC_UART);
    gpio_set_function(config_.rx_pin, GPIO_FUNC_UART);

    int uart_index = (config_.uart == uart0) ? 0 : 1;
    instances_[uart_index] = this;

    if (uart_index == 0)
    {
        irq_set_exclusive_handler(UART0_IRQ, uart0_irq_handler);
        irq_set_enabled(UART0_IRQ, true);
    }
    else
    {
        irq_set_exclusive_handler(UART1_IRQ, uart1_irq_handler);
        irq_set_enabled(UART1_IRQ, true);
    }

    uart_set_irq_enables(config_.uart, true, false);                                                                                            // Interrupción RX
    uart_get_hw(config_.uart)->imsc |= (UART_UARTIMSC_OEIM_BITS | UART_UARTIMSC_BEIM_BITS | UART_UARTIMSC_PEIM_BITS | UART_UARTIMSC_FEIM_BITS); // Interrupciones de errores
    xTimer = xTimerCreate(
        "UARTTimer",
        pdMS_TO_TICKS(config_.timer_interval_ms),
        pdFALSE,
        this,
        uart_timeout_callback);

    if (xTimer == nullptr)
    {
        printf("Imposible to create timer, aborting");
        return false;
    }

    return true;
}

void UartDriver::deinit()
{

    uart_set_irq_enables(config_.uart, false, false);
    uart_get_hw(config_.uart)->imsc &= ~(UART_UARTIMSC_BEIM_BITS | UART_UARTIMSC_PEIM_BITS | UART_UARTIMSC_FEIM_BITS);
    int uart_index = (config_.uart == uart0) ? 0 : 1;
    irq_set_enabled((uart_index == 0) ? UART0_IRQ : UART1_IRQ, false);
    instances_[uart_index] = nullptr;
    uart_deinit(config_.uart);
}

void UartDriver::write(const uint8_t *data, size_t length)
{
    if (!uart_is_writable(config_.uart))
    {
        printf("Error: Write is not available");
    }
    else
    {
        printf("%s", reinterpret_cast<const char *>(data));

        uart_write_blocking(config_.uart, data, length);
    }
}

void UartDriver::flush()
{
    taskENTER_CRITICAL();
    rx_head_ = 0;
    rx_tail_ = 0;
    event_type_ = UART_UNKNOWN;
    taskEXIT_CRITICAL();
}

void UartDriver::set_baud_rate(uint baud_rate)
{

    config_.baud_rate = baud_rate;
    uart_set_baudrate(config_.uart, baud_rate);
}

size_t UartDriver::read_data(uint8_t *buffer, size_t buffer_size)
{
    size_t data_size;

    taskENTER_CRITICAL();
    if (rx_head_ >= rx_tail_)
    {
        data_size = rx_head_ - rx_tail_;
    }
    else
    {
        data_size = BUFFER_SIZE - rx_tail_ + rx_head_;
    }

    size_t bytes_to_read = (data_size < buffer_size) ? data_size : buffer_size;

    for (size_t i = 0; i < bytes_to_read; ++i)
    {
        buffer[i] = rx_buffer_[(rx_tail_ + i) % BUFFER_SIZE];
    }

    rx_tail_ = (rx_tail_ + bytes_to_read) % BUFFER_SIZE;
    taskEXIT_CRITICAL();

    return bytes_to_read;
}

uart_event_type_t UartDriver::get_event_type()
{
    return event_type_;
}

void UartDriver::uart0_irq_handler()
{
    UartDriver *driver = instances_[0];
    if (driver)
    {
        uart_irq_handler(driver, uart0, uart_get_hw(uart0));
    }
}

void UartDriver::uart1_irq_handler()
{
    UartDriver *driver = instances_[1];
    if (driver)
    {
        uart_irq_handler(driver, uart1, uart_get_hw(uart1));
    }
}

void UartDriver::uart_irq_handler(UartDriver *driver, uart_inst_t *uart_inst, uart_hw_t *uart_hw)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    uint32_t status = uart_hw->mis;
    uart_hw->icr = status;

    driver->event_type_ = UART_DATA;

    uint32_t error_status = status & (UART_UARTMIS_OEMIS_BITS | UART_UARTMIS_BEMIS_BITS | UART_UARTMIS_PEMIS_BITS | UART_UARTMIS_FEMIS_BITS);

    if (error_status)
    {
        switch (error_status)
        {
        case UART_UARTMIS_OEMIS_BITS:
            driver->event_type_ = UART_OVERFLOW;
            break;
        case UART_UARTMIS_BEMIS_BITS:
            driver->event_type_ = UART_BREAK;
            break;
        case UART_UARTMIS_PEMIS_BITS:
            driver->event_type_ = UART_PARITY_ERROR;
            break;
        case UART_UARTMIS_FEMIS_BITS:
            driver->event_type_ = UART_FRAME_ERROR;
            break;
        default:
            driver->event_type_ = UART_UNKNOWN;
            break;
        }
    }

    while (uart_is_readable(uart_inst))
    {
        uint8_t ch = uart_getc(uart_inst);

        size_t next_head = (driver->rx_head_ + 1) % BUFFER_SIZE;
        if (next_head != driver->rx_tail_)
        {
            driver->rx_buffer_[driver->rx_head_] = ch;
            driver->rx_head_ = next_head;
        }
        else
        {
            driver->event_type_ = UART_OVERFLOW;
            if (driver->config_.task_handle != NULL)
            {
                vTaskNotifyGiveFromISR(driver->config_.task_handle, &xHigherPriorityTaskWoken);
            }

            if (driver->xTimer != NULL)
            {
                xTimerStopFromISR(driver->xTimer, &xHigherPriorityTaskWoken);
            }

            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            return;
        }
    }

    if (driver->xTimer != NULL)
    {
        xTimerStartFromISR(driver->xTimer, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void UartDriver::uart_timeout_callback(TimerHandle_t xTimer)
{
    UartDriver *driver = static_cast<UartDriver *>(pvTimerGetTimerID(xTimer));

    if (driver->config_.task_handle != NULL)
    {
        xTaskNotifyGive(driver->config_.task_handle);
    }
}
