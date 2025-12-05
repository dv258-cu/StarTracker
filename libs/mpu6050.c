/**
 * Hunter Adams (vha3@cornell.edu)
 * 
 *
 */

#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "mpu6050.h"

// Initialize I2C bus for MPU6050
void imu_init(void) {
    i2c_init(IMU_I2C_CHAN, IMU_I2C_BAUD_RATE);
    gpio_set_function(IMU_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(IMU_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(IMU_SDA_PIN);
    gpio_pull_up(IMU_SCL_PIN);
    sleep_ms(10);  // Let bus stabilize
}
 
 void mpu6050_reset() {
     // Two byte reset. First byte register, second byte data
     // There are a load more options to set up the device in
     // different ways that could be added here
     uint8_t buf[] = {0x6B, 0x00};
     i2c_write_blocking(IMU_I2C_CHAN, IMU_ADDRESS, buf, 2, false);

     // Set gyro sample rate (set to 1KHz, same as accel)
     uint8_t gyro_rate[] = {0x19, 0b00000111} ;
     i2c_write_blocking(IMU_I2C_CHAN, IMU_ADDRESS, gyro_rate, 2, false);

     // Configure the Gyro range (+/- 250 deg/s)
     uint8_t gyro_settings[] = {0x1b, 0b00000000} ;
     i2c_write_blocking(IMU_I2C_CHAN, IMU_ADDRESS, gyro_settings, 2, false);

     // Configure the Accel range (+/- 2g's)
     uint8_t accel_settings[] = {0x1c, 0b00000000} ;
     i2c_write_blocking(IMU_I2C_CHAN, IMU_ADDRESS, accel_settings, 2, false);

     // Configure interrupt pin
     uint8_t pin_settings[] = {0x37, 0b00010000} ;
     i2c_write_blocking(IMU_I2C_CHAN, IMU_ADDRESS, pin_settings, 2, false);

     // Configure data ready interrupt
     uint8_t int_config[] = {0x38, 0x01} ;
     i2c_write_blocking(IMU_I2C_CHAN, IMU_ADDRESS, int_config, 2, false);
 }

void mpu6050_read_raw(fix15 accel[3], fix15 gyro[3]) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];
    int16_t temp_accel, temp_gyro ;

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_blocking(IMU_I2C_CHAN, IMU_ADDRESS, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(IMU_I2C_CHAN, IMU_ADDRESS, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        temp_accel = (buffer[i<<1] << 8 | buffer[(i<<1) + 1]);
        accel[i] = temp_accel ;
        accel[i] <<= 2 ; // convert to g's (fixed point)
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    i2c_write_blocking(IMU_I2C_CHAN, IMU_ADDRESS, &val, 1, true);
    i2c_read_blocking(IMU_I2C_CHAN, IMU_ADDRESS, buffer, 6, false);  // False - finished with bus

    for (int i = 0; i < 3; i++) {
        temp_gyro = (buffer[i<<1] << 8 | buffer[(i<<1) + 1]);
        gyro[i] = temp_gyro ;
        gyro[i] = multfix15(gyro[i], 500<<16) ; // deg/sec
    }
}

// Read IMU data and return as float values
imu_data imu_read(void) {
    imu_data result = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    
    fix15 accel[3], gyro[3];
    mpu6050_read_raw(accel, gyro);
    
    // Convert fixed-point to float
    result.accel_x = fix2float15(accel[0]);
    result.accel_y = fix2float15(accel[1]);
    result.accel_z = fix2float15(accel[2]);
    result.gyro_x = fix2float15(gyro[0]);
    result.gyro_y = fix2float15(gyro[1]);
    result.gyro_z = fix2float15(gyro[2]);
    
    return result;
}


