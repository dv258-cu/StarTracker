/**
 * Tony Kariuki (akk85@cornell.edu)
 * 
 * magnetometer.c
 *
 */

#include <stdint.h>
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
    // Hard iron bias in raw LSB units (calibrated values)
    .hard_iron_bias = {
        -8.0f,
        -8.0f,
        -8.0f
    },
    
    .soft_iron_matrix = {
        {1.000000f, 0.0f, 0.0f},
        {0.0f, 1.000000f, 0.0f},
        {0.0f, 0.0f, 1.000000f}
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
void initMagnetometer(void) {
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

    // Apply hard-iron bias in raw LSB units (before conversion to microtesla)
    float centered_x_lsb = (float)raw.x - calibration_data.hard_iron_bias[0];
    float centered_y_lsb = (float)raw.y - calibration_data.hard_iron_bias[1];
    float centered_z_lsb = (float)raw.z - calibration_data.hard_iron_bias[2];

    // Apply soft-iron matrix in LSB units
    float corrected_x_lsb = calibration_data.soft_iron_matrix[0][0]*centered_x_lsb + 
                             calibration_data.soft_iron_matrix[0][1]*centered_y_lsb + 
                             calibration_data.soft_iron_matrix[0][2]*centered_z_lsb;
    float corrected_y_lsb = calibration_data.soft_iron_matrix[1][0]*centered_x_lsb + 
                             calibration_data.soft_iron_matrix[1][1]*centered_y_lsb + 
                             calibration_data.soft_iron_matrix[1][2]*centered_z_lsb;
    float corrected_z_lsb = calibration_data.soft_iron_matrix[2][0]*centered_x_lsb + 
                             calibration_data.soft_iron_matrix[2][1]*centered_y_lsb + 
                             calibration_data.soft_iron_matrix[2][2]*centered_z_lsb;

    // Convert calibrated LSB values to microtesla
    // Datasheet scale: 0.0625 mG/LSB = 0.00625 µT/LSB
    result.x = corrected_x_lsb * 0.00625f;
    result.y = corrected_y_lsb * 0.00625f;
    result.z = corrected_z_lsb * 0.00625f;

    return result;
}

// get heading in degrees x and y values from magnetometer_calibrate data 
float magnetometer_get_heading(float magnetic_x, float magnetic_y) {
    float heading = atan2f(magnetic_x, magnetic_y) * 180.0f / PI;
    if (heading < 0) {
        heading += 360.0f;
    }
    return heading;
}


// Helper function to normalize angle difference (handles 360° wrap-around)
static float normalize_angle_diff(float angle1, float angle2) {
    float diff = angle2 - angle1;
    if (diff > 180.0f) {
        diff -= 360.0f;
    }
    if (diff < -180.0f) {
        diff += 360.0f;
    }
    return diff;
}

// Filter magnetometer values and calculate smoothed heading
float magnetometer_get_filtered_heading(magnetometer mag_data) {
    // Static variables for filtering (persist across function calls)
    static float filtered_mag_x = 0.0f;
    static float filtered_mag_y = 0.0f;
    static bool first_mag = true;
    
    static float smoothed_heading = 0.0f;
    static bool first_heading = true;
    
    // Filter magnetometer X and Y values to reduce noise and jumps
    if (first_mag) {
        filtered_mag_x = mag_data.x;
        filtered_mag_y = mag_data.y;
        first_mag = false;
    } else {
        // Exponential moving average for magnetometer values
        // Use heavy smoothing (0.2 factor) to reduce jumps
        float mag_filter = 0.2f;
        filtered_mag_x = filtered_mag_x * (1.0f - mag_filter) + mag_data.x * mag_filter;
        filtered_mag_y = filtered_mag_y * (1.0f - mag_filter) + mag_data.y * mag_filter;
    }
    
    // Get heading from filtered magnetometer values
    float raw_heading = magnetometer_get_heading(filtered_mag_x, filtered_mag_y);
    
    // Apply smoothing to heading
    if (first_heading) {
        smoothed_heading = raw_heading;
        first_heading = false;
    } else {
        float diff = normalize_angle_diff(smoothed_heading, raw_heading);
        
        // Adaptive smoothing: more smoothing for small changes, faster for large
        float adaptive_factor = 0.5f;  // Default moderate smoothing
        if (fabsf(diff) < 1.0f) {
            adaptive_factor = 0.3f;  // Heavy smoothing for noise
        } else if (fabsf(diff) > 5.0f) {
            adaptive_factor = 0.7f;  // Fast response for rotation
        }
        
        smoothed_heading += diff * adaptive_factor;
        if (smoothed_heading < 0.0f) {
            smoothed_heading += 360.0f;
        }
        if (smoothed_heading >= 360.0f) {
            smoothed_heading -= 360.0f;
        }
    }
    
    return smoothed_heading;
}

