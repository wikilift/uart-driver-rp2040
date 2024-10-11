/**
 * @file uart_driver.h
 * @brief UartDriver class for handling UART communication with interrupt-based management.
 *
 * This header file defines the UartDriver class, which provides an interface for UART communication
 * on the RP2040 microcontroller using the Pico SDK. The driver supports configuration of UART parameters,
 * such as baud rate, parity, and stop bits, and manages data transmission and reception through circular buffers.
 *
 * The class includes interrupt handlers for UART events (data received, errors, etc.) and a repeating timer
 * for periodic checks. It is designed to be used with FreeRTOS, allowing integration with RTOS tasks for
 * efficient asynchronous communication.
 *
 * The driver also supports custom task notifications to signal events and buffer management, making it
 * suitable for real-time applications that require responsive and low-latency UART communication.
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
#pragma once

#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "FreeRTOS.h"
#include "timers.h"
#include "task.h"

/**
 * @brief UART event types for handling different UART conditions.
 */
typedef enum
{
    UART_DATA,         /**< Data successfully received */
    UART_BREAK,        /**< Break condition detected */
    UART_PARITY_ERROR, /**< Parity error detected */
    UART_FRAME_ERROR,  /**< Frame error detected */
    UART_OVERFLOW,     /**< Buffer overflow error detected */
    UART_UNKNOWN       /**< Unknown event occurred */
} uart_event_type_t;

/**
 * @brief Configuration structure for UART initialization.
 */
typedef struct
{
    uart_inst_t *uart;        /**< UART instance (e.g., uart0, uart1) */
    uint baud_rate;           /**< Baud rate for UART communication (e.g., 115200, 9600) */
    uint tx_pin;              /**< TX pin number */
    uint rx_pin;              /**< RX pin number */
    uint data_bits;           /**< Number of data bits (e.g., 8) */
    uint stop_bits;           /**< Number of stop bits (e.g., 1) */
    uart_parity_t parity;     /**< Parity configuration (e.g., UART_PARITY_NONE) */
    TaskHandle_t task_handle; /**< Task handle for receiving notifications */
    int timer_interval_ms;    /**< Timer interval in milliseconds */
} uart_config_t;

/**
 * @brief UART driver class to manage UART communication.
 *
 * This class provides methods to initialize, configure, and manage UART communication
 * using DMA and interrupts. It supports circular buffer management, data sending, and
 * receiving, along with FreeRTOS task notifications.
 */
class UartDriver
{
public:
    /**
     * @brief Constructor to initialize the UART driver with the given configuration.
     * @param config Configuration structure for UART.
     */
    UartDriver(const uart_config_t &config);

    /**
     * @brief Destructor to deinitialize and free resources.
     */
    ~UartDriver();

    /**
     * @brief Install and initialize the UART driver.
     *
     * This function configures the UART hardware, sets up interrupts, and initializes the circular buffer.
     * @return true if installation is successful, false otherwise.
     */
    bool install();

    /**
     * @brief Deinitialize and uninstall the UART driver.
     *
     * This function disables UART interrupts, releases resources, and deinitializes the UART hardware.
     */
    void deinit();

    /**
     * @brief Send data through UART.
     * @param data Pointer to the data to be sent.
     * @param length Length of the data to be sent.
     */
    void write(const uint8_t *data, size_t length);

    /**
     * @brief Set a new baud rate for UART communication.
     *
     * This function allows dynamic adjustment of the baud rate.
     * @param baud_rate The new baud rate to be configured.
     */
    void set_baud_rate(uint baud_rate);

    /**
     * @brief Read data from the circular buffer.
     *
     * Reads the available data in the buffer into the provided buffer.
     * @param buffer Pointer to the buffer where data will be stored.
     * @param buffer_size Size of the buffer.
     * @return Number of bytes read.
     */
    size_t read_data(uint8_t *buffer, size_t buffer_size);

    /**
     * @brief Get the current UART event type.
     *
     * Returns the current event type based on the detected UART condition (e.g., data received, parity error).
     * @return The type of UART event that occurred.
     */
    uart_event_type_t get_event_type();

    static const size_t BUFFER_SIZE = 1024; /**< Constant defining the circular buffer size. */

    /**
     * @brief Flush the UART driver buffer.
     *
     * This method clears the internal circular buffer, resetting the read and write indices.
     * It can be used to discard any data currently stored in the buffer.
     */
    void flush(void);

private:
    uart_config_t config_;                  /**< UART configuration settings */
    volatile uart_event_type_t event_type_; /**< Current UART event type */

    /**< Timer for handling periodic checks on UART data. */
    TimerHandle_t xTimer;

    /**< Static array to hold driver instances (for multiple UARTs). */
    static UartDriver *instances_[2];

    /**< Circular buffer for storing received data. */
    uint8_t rx_buffer_[BUFFER_SIZE];
    /**< Write index for the circular buffer. */
    size_t rx_head_;
    /**< Read index for the circular buffer. */
    size_t rx_tail_;

    /**
     * @brief UART0 interrupt handler.
     *
     * Handles UART0-specific interrupts and processes data accordingly.
     */
    static void uart0_irq_handler();

    /**
     * @brief UART1 interrupt handler.
     *
     * Handles UART1-specific interrupts and processes data accordingly.
     */
    static void uart1_irq_handler();

    /**
     * @brief Generic UART interrupt handler.
     *
     * This function handles UART interrupts and processes the received data,
     * updating the event type and managing the circular buffer.
     * @param driver Pointer to the UART driver instance.
     * @param uart_inst UART instance (e.g., uart0 or uart1).
     * @param uart_hw Hardware pointer to the UART structure.
     */
    static void uart_irq_handler(UartDriver *driver, uart_inst_t *uart_inst, uart_hw_t *uart_hw);

    /**
     * @brief Timer callback for checking UART data.
     *
     * This callback function is triggered at regular intervals to handle data reception and event processing.
     * @param rt Pointer to the repeating timer structure.
     * @return true if the callback should be repeated, false to stop.
     */
    static void uart_timeout_callback(TimerHandle_t xTimer);

    /**< Copy constructor (deleted to prevent copying). */
    UartDriver(const UartDriver &) = delete;
    /**< Assignment operator (deleted to prevent copying). */
    UartDriver &operator=(const UartDriver &) = delete;
};
