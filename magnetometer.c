/**
 * Tony Kariuki (akk85@cornell.edu)
 * 
 * magnetometer.c
 *
 */

#include <hardware/i2c.h>
#include <hardware/gpio.h>
#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "magnetometer.h"

// Full 3x3 soft-iron (ellipsoid → sphere) calibration
typedef struct {
    float hard_iron_bias[3];      
    float soft_iron_matrix[3][3]; 
} magnetometer_calibration;


static const magnetometer_calibration calibration_data = {
    .hard_iron_bias = {
        -338.261184f, 
        -355.552345f, 
        555.445243f
    },
    
    .soft_iron_matrix = {
        {0.000628f, -0.000012f, 0.000006f},
        {-0.000012f, 0.000631f, -0.000000f},
        {0.000006f, -0.000000f, 0.000657f}
    }
};

// Reset/recover I2C bus if it's stuck (internal helper)
static void i2c_bus_recover(void) {
    // Deinitialize I2C
    i2c_deinit(I2C_CHAN);
    
    // Set pins as GPIO temporarily
    gpio_set_function(SDA_PIN, GPIO_FUNC_NULL);
    gpio_set_function(SCL_PIN, GPIO_FUNC_NULL);
    
    // Configure as outputs and drive high
    gpio_init(SDA_PIN);
    gpio_init(SCL_PIN);
    gpio_set_dir(SDA_PIN, GPIO_OUT);
    gpio_set_dir(SCL_PIN, GPIO_OUT);
    gpio_put(SDA_PIN, 1);
    gpio_put(SCL_PIN, 1);
    
    // Clock recovery - send 9 clock pulses while SDA is high
    for (int pulse_count = 0; pulse_count < 9; pulse_count++) {
        gpio_put(SCL_PIN, 0);
        sleep_us(10);
        gpio_put(SCL_PIN, 1);
        sleep_us(10);
    }
    
    // Set SDA high, then SCL high (idle state)
    gpio_put(SDA_PIN, 1);
    gpio_put(SCL_PIN, 1);
    sleep_ms(10);
}

// I2C configuration
void magnetometer_init(void) {
    i2c_bus_recover();
    
    i2c_init(I2C_CHAN, I2C_BAUD_RATE);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
    
    sleep_ms(10);  // Let bus stabilize
}


// ---- Low-level helpers ----
static bool i2c_write(uint8_t register_address, uint8_t register_value) {
    uint8_t write_buffer[2] = {register_address, register_value};
    return i2c_write_blocking(I2C_CHAN, MAG_ADDRESS, write_buffer, 2, false) == 2;
}

// Read N bytes from the magnetometer
static bool i2c_read(uint8_t register_address, uint8_t *read_buffer, size_t byte_count) {
    if (i2c_write_blocking(I2C_CHAN, MAG_ADDRESS, &register_address, 1, true) != 1) {
        return false;
    }
    return i2c_read_blocking(I2C_CHAN, MAG_ADDRESS, read_buffer, byte_count, false) == (int)byte_count;
}

// ---- Raw data unpack (20-bit two's complement) ----
static inline int32_t unpack20(uint8_t high_byte, uint8_t mid_byte, uint8_t low_nibble) {
    int32_t unpacked_value = ((int32_t)high_byte << 12) | ((int32_t)mid_byte << 4) | (low_nibble & 0x0F);
    if (unpacked_value & (1 << 19)) unpacked_value -= (1 << 20);  // sign-extend
    return unpacked_value;
}



// Raw magnetometer values (20-bit signed integers)
typedef struct {
    int32_t x;
    int32_t y;
    int32_t z;
} magnetometer_raw;

static magnetometer_raw magnetometer_read_raw(void) {
    magnetometer_raw result = {0, 0, 0};

    uint8_t control_value = MMC5603_CTRL0_AUTO_SR_EN | MMC5603_CTRL0_TM_M;
    if (!i2c_write(MMC5603_REG_CTRL0, control_value)) {
        return result;  // Return zeros if write fails
    }

    sleep_ms(20);
    
    uint8_t status_register = 0;
    const int max_poll_attempts = 500;  // ~100ms timeout
    bool measurement_complete = false;
    
    for (int poll_count = 0; poll_count < max_poll_attempts; poll_count++) {
        if (!i2c_read(MMC5603_REG_STATUS, &status_register, 1)) {
            return result;  // Return zeros if read fails
        }
        
        if ((status_register & MMC5603_STATUS_MEAS_DONE) != 0) {
            measurement_complete = true;
            break;
        }
        
        sleep_us(200);
    }

    uint8_t raw_data_bytes[9];
    if (!i2c_read(MMC5603_REG_XOUT0, raw_data_bytes, 9)) {
        return result;  // Return zeros if read fails
    }

    result.x = unpack20(raw_data_bytes[0], raw_data_bytes[1], raw_data_bytes[6]);
    result.y = unpack20(raw_data_bytes[2], raw_data_bytes[3], raw_data_bytes[7]);
    result.z = unpack20(raw_data_bytes[4], raw_data_bytes[5], raw_data_bytes[8]);

    // Clear TM_M bit 
    i2c_write(MMC5603_REG_CTRL0, MMC5603_CTRL0_AUTO_SR_EN);

    return result;
}

// Full 3x3 soft-iron calibration (sphere)
magnetometer magnetometer_calibrate(void) {
    magnetometer result = {0.0f, 0.0f, 0.0f};
    
    magnetometer_raw raw = magnetometer_read_raw();

    // Convert raw values to microtesla (float) datasheet scale: 0.0625 mG/LSB = 0.00625 µT/LSB
    float raw_magnetic_x = raw.x * 0.00625f;
    float raw_magnetic_y = raw.y * 0.00625f;
    float raw_magnetic_z = raw.z * 0.00625f;
    
    // Remove hard-iron bias 
    float centered_x = raw_magnetic_x - calibration_data.hard_iron_bias[0];
    float centered_y = raw_magnetic_y - calibration_data.hard_iron_bias[1];
    float centered_z = raw_magnetic_z - calibration_data.hard_iron_bias[2];

    // Apply 3x3 whitening matrix: corrected = W * (raw - bias)
    result.x = calibration_data.soft_iron_matrix[0][0]*centered_x + 
               calibration_data.soft_iron_matrix[0][1]*centered_y + 
               calibration_data.soft_iron_matrix[0][2]*centered_z;
    result.y = calibration_data.soft_iron_matrix[1][0]*centered_x + 
               calibration_data.soft_iron_matrix[1][1]*centered_y + 
               calibration_data.soft_iron_matrix[1][2]*centered_z;
    result.z = calibration_data.soft_iron_matrix[2][0]*centered_x + 
               calibration_data.soft_iron_matrix[2][1]*centered_y + 
               calibration_data.soft_iron_matrix[2][2]*centered_z;

    return result;
}

// get heading in degrees x and y values from magnetometer_calibrate data 
float magnetometer_get_heading(float magnetic_x, float magnetic_y) {
    float heading = atan2f(magnetic_x, magnetic_y) * 180.0f / PI;
    if (heading < 0) heading += 360.0f;
    return heading;
}
