// Include standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
// Include Pico libraries
#include "pico/stdlib.h"
#include "pico/divider.h"
#include "pico/multicore.h"
// Include hardware libraries
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
#include "hardware/spi.h"
#include "hardware/adc.h"
#include "hardware/uart.h"
#include "hardware/pwm.h"

// Include Stepper Motor Library
#include "libs/stepper.h"

// Include GPS Library
#include "libs/gps.h"

#define FRAME_RATE 33000

#define PWM_OUT 15
#define SYSTEM_CLK_KHZ 150000
#define CLK_DIV 250.0f

// --- Buffer Configuration ---
#define NMEA_BUFFER_SIZE 256
char nmea_buffer[NMEA_BUFFER_SIZE];
int buffer_index = 0;
bool new_sentence_ready = false;

#define GPS_RX 13
#define GPS_TX 12

#define UART_ID uart0
#define BAUD_RATE 9600

// Stepper Motor Control Pins
#define PITCH_DIR 16
#define AZIM_DIR 14
#define MOTOR_AZIM 15
#define MOTOR_PITCH 17

uint8_t nmea_ready = 0;

void on_uart_rx() {
    // Only read one byte at a time
    while (uart_is_readable(UART_ID)) {
        char c = uart_getc(UART_ID);
        // Add character to buffer if not full
        if (buffer_index < NMEA_BUFFER_SIZE - 1) {
            nmea_buffer[buffer_index++] = c;
            // Check for end of sentence (e.g., newline character \n or \r)
            if (c == '\n') {
                nmea_buffer[buffer_index] = '\0'; // Null-terminate the string
                parseNMEA(nmea_buffer);          // Process non-blocking data
                buffer_index = 0;                // Reset buffer
                break; // Exit the while loop
            }
        } else {
            // Buffer overflow, reset index
            buffer_index = 0;
        }
    }
}

void initGPS() {
    // Set up our UART with the specified baud rate
    uart_init(UART_ID, BAUD_RATE);

    // Set the GPIO pins for the UART function
    gpio_set_function(GPS_TX, GPIO_FUNC_UART);
    gpio_set_function(GPS_RX, GPIO_FUNC_UART);

    // Disable hardware flow control (typical for GPS modules)
    uart_set_hw_flow(UART_ID, false, false);

    // Set data format (8 bits, no parity, 1 stop bit - standard for NMEA)
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);

    // Set up the interrupt handler
    int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;

    // And set up and enable the interrupt
    irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
    irq_set_enabled(UART_IRQ, true);

    // Enable the UART to fire an interrupt on receive data ready
    uart_set_irq_enables(UART_ID, true, false);

    printf("UART initialized on GPIO%d (RX) at %d baud.\n", GPS_RX, BAUD_RATE);   
}



int main(void) {
    set_sys_clock_khz(150000, true) ;
    // initialize stio
    stdio_init_all();

    // GPS Init
    initGPS();

    // Stepper Init
    initSteppers();

    stepMotor(10000, MOTOR_AZIM);
    stepMotor(-10000, MOTOR_AZIM);
    stepMotor(10000, MOTOR_PITCH);
    stepMotor(-10000, MOTOR_PITCH);
}