// Include the VGA grahics library
#include "vga16_graphics_v2.h"
#include "vga_graphics_v3.h"
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

// Stepper Motor Control Pins
#define PITCH_DIR 16
#define AZIM_DIR 14
#define MOTOR_AZIM 15
#define MOTOR_PITCH 17
#define MOTOR_ROTATE_DIR 19
#define MOTOR_ROTATE_STEP 18

#define SYSTEM_CLK_KHZ 150000
#define CLK_DIV 250.0f




void resetMotors() {
    gpio_put(AZIM_DIR, 1);
    gpio_put(PITCH_DIR, 1);
    gpio_put(MOTOR_ROTATE_DIR, 1);
}


void initSteppers() {
    gpio_init(MOTOR_AZIM);
    gpio_init(MOTOR_PITCH);
    gpio_init(AZIM_DIR);
    gpio_init(PITCH_DIR);
    gpio_init(MOTOR_ROTATE_DIR);
    gpio_init(MOTOR_ROTATE_STEP);


    gpio_set_dir(MOTOR_PITCH, GPIO_OUT);
    gpio_set_dir(MOTOR_AZIM, GPIO_OUT);
    gpio_set_dir(AZIM_DIR, GPIO_OUT);
    gpio_set_dir(PITCH_DIR, GPIO_OUT);
    gpio_set_dir(MOTOR_ROTATE_DIR, GPIO_OUT);
    gpio_set_dir(MOTOR_ROTATE_STEP, GPIO_OUT);
    
    
    resetMotors();
}

// Step Once
void step(int motor) {
    sleep_ms(1);
    gpio_put(motor, 1);
    sleep_us(1);
    gpio_put(motor, 0);
}

void stepMotor(int num_steps, int motor) {

    if (num_steps < 0) {
            gpio_put(AZIM_DIR, 0);
            gpio_put(PITCH_DIR, 0);
            gpio_put(MOTOR_ROTATE_DIR, 0);
    }
    else {
        gpio_put(AZIM_DIR, 1);
        gpio_put(PITCH_DIR, 1);
        gpio_put(MOTOR_ROTATE_DIR, 1);
    }
    
    for (int i = 0; i < abs(num_steps); i++) {
        step(motor);
    } 

    resetMotors();
    
}