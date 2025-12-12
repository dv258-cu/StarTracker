/**
 * Main file for StarTracker
 * tracker.c
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdlib.h>
#include "magnetometer.h"
#include "libs/mpu6050.h"
#include "libs/stepper.h"
#include "libs/gps.h"
#include "libs/vga16_graphics_v2.h"
#include "orientation.h"
#include "libs/pt_cornell_rp2040_v1_4.h"

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

#define PI 3.14159265358979323846

// Global variables for sensor data 
static magnetometer magnetometer_data;
static imu_data imu_data_global;
static orientation current_orientation;
static const float dt = 0.05f;  // 50ms = 0.05 seconds

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
#define AZIMUTH_DIR 14
#define BOTTOM_MOTOR 15
#define MOTOR_PITCH 17
#define MOTOR_ROTATE_DIR 19
#define MOTOR_ROTATE_STEP 18

// Global variables for stepping gimble 
float magnetic_north = 0.0f;
float heading;
float heading_difference;
int steps_to_do = 0;
float steps_per_degree = 1600.0f / 360.0f;  // ~4.444 steps per degree
const float DEADBAND = 5.0f;  // error degree tolerance
const int MAX_STEPS = 100;    // ~22.5 degrees per iteration (6% of full rotation)

// protothread to display compass like approaching magnetic north on vga screen 
static PT_THREAD (protothread_vga(struct pt *pt)){
    PT_BEGIN(pt);
    
    static short center_x = 320; 
    static short center_y = 240;  
    static short compass_radius = 150;  
    static float heading_rad;
    static short needle_x, needle_y;
    static char heading_str[20];
    static char error_str[20];
    static char color_indicator;
    
    while(1) {
        // Clear compass area (draw background circle)
        fillCircle(center_x, center_y, compass_radius + 5, BLACK);
        drawCircle(center_x, center_y, compass_radius, WHITE);
        
        // cardinal directions
        drawChar(center_x - 3, center_y - compass_radius - 15, 'N', RED, BLACK, 2);
        drawChar(center_x + compass_radius + 10, center_y - 3, 'E', WHITE, BLACK, 2);
        drawChar(center_x - 3, center_y + compass_radius + 10, 'S', WHITE, BLACK, 2);
        drawChar(center_x - compass_radius - 15, center_y - 3, 'W', WHITE, BLACK, 2);
        
        // Draw tick marks every 30 degrees
        for (int i = 0; i < 12; i++) {
            float angle_rad = (i * 30.0f) * PI / 180.0f;
            short x1 = center_x + (short)((compass_radius - 10) * sinf(angle_rad));
            short y1 = center_y - (short)((compass_radius - 10) * cosf(angle_rad));
            short x2 = center_x + (short)(compass_radius * sinf(angle_rad));
            short y2 = center_y - (short)(compass_radius * cosf(angle_rad));
            drawLine(x1, y1, x2, y2, WHITE);
        }
        
        heading_rad = (heading - 90.0f) * PI / 180.0f;
        
        // Draw compass needle pointing to current heading (red)
        needle_x = center_x + (short)(compass_radius * 0.8f * cosf(heading_rad));
        needle_y = center_y - (short)(compass_radius * 0.8f * sinf(heading_rad));
        drawLine(center_x, center_y, needle_x, needle_y, RED);
        
        // Draw small circle at center
        fillCircle(center_x, center_y, 5, RED);
        
        // Draw arrow pointing to magnetic north (green)
        short north_x = center_x + (short)(compass_radius * 0.6f * cosf(-PI/2));  // -90° = North
        short north_y = center_y - (short)(compass_radius * 0.6f * sinf(-PI/2));
        drawLine(center_x, center_y, north_x, north_y, GREEN);
        
        // Color indicator based on error from magnetic north
        if (fabsf(heading_difference) < 5.0f) {
            color_indicator = GREEN;  
        } else if (fabsf(heading_difference) < 15.0f) {
            color_indicator = YELLOW;  
        } else {
            color_indicator = RED;  
        }
        
        // Draw status box at bottom
        fillRect(10, 450, 300, 25, BLACK);
        drawRect(10, 450, 300, 25, WHITE);
        
        // Display heading value
        sprintf(heading_str, "Heading: %.1f", heading);
        setCursor(15, 455);
        setTextColor(WHITE);
        writeString(heading_str);
        
        // Display error from magnetic north
        sprintf(error_str, "Error: %.1f", heading_difference);
        setCursor(200, 455);
        setTextColor(color_indicator);
        writeString(error_str);
        
        // Draw indicator bar showing alignment (0-100%)
        short bar_width = (short)(280 * (1.0f - fabsf(heading_difference) / 180.0f));
        if (bar_width < 0) bar_width = 0;
        fillRect(15, 430, bar_width, 10, color_indicator);
        drawRect(15, 430, 280, 10, WHITE);
        
        PT_YIELD_usec(50000);  // Update every 50ms
    }
    
    PT_END(pt);
}



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

int main(void) 
{
    set_sys_clock_khz(150000, true) ;
    stdio_init_all();
    initVGA();

    // GPS
    initGPS();
    initSteppers();
    initMagnetometer();

    magnetometer_data = magnetometer_calibrate();
        
    heading = magnetometer_get_filtered_heading(magnetometer_data);
    
    printf("heading: %f\n", heading);
    
    // take shortest path to magnetic north
    heading_difference = heading - magnetic_north;
    if (heading_difference > 180.0f) {
        heading_difference = heading_difference - 360.0f;
    } else if (heading_difference < -180.0f) {
        heading_difference = heading_difference + 360.0f;
    }

    if (fabsf(heading_difference) > DEADBAND) { 
        steps_to_do = (int)(heading_difference * steps_per_degree);
    }

    stepMotor(steps_to_do, MOTOR_PITCH);
    
    /**
    use the magnetometer to drive the stepper motors to magnetic north 
    stepmotor takes 1600 steps per revolution
    */
    // while (true){

    //     magnetometer_data = magnetometer_calibrate();
        
    //     heading = magnetometer_get_filtered_heading(magnetometer_data);
        
    //     printf("heading: %f\n", heading);
        
    //     // take shortest path to magnetic north
    //     heading_difference = heading - magnetic_north;
    //     if (heading_difference > 180.0f) {
    //         heading_difference = heading_difference - 360.0f;
    //     } else if (heading_difference < -180.0f) {
    //         heading_difference = heading_difference + 360.0f;
    //     }

    //     if (fabsf(heading_difference) > DEADBAND) { 
    //         steps_to_do = (int)(heading_difference * steps_per_degree);
            
    //         // Limit steps to prevent overshoot and allow feedback during movement
    //         // if (steps_to_do > MAX_STEPS) steps_to_do = MAX_STEPS;
    //         // else if (steps_to_do < -MAX_STEPS) steps_to_do = -MAX_STEPS;

    //         stepMotor(steps_to_do, MOTOR_PITCH);
    //     } 

    //     // sleep_ms(50);  // Control loop delay
    // }



}
