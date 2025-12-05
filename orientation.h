/**
 * Orientation sensor fusion
 * Combines magnetometer and IMU data using complementary filter
 */

#ifndef ORIENTATION_H
#define ORIENTATION_H

#include <stdbool.h>

// Orientation angles (Euler angles)
typedef struct {
    float roll;   
    float pitch;  
    float yaw;    
} orientation;

void orientation_init(void);


void orientation_calibrate_gyro(float gyro_x, float gyro_y, float gyro_z);

// Update orientation using magnetometer and IMU data
bool orientation_update(
    float accel_x, float accel_y, float accel_z,
    float gyro_x, float gyro_y, float gyro_z,
    float mag_x, float mag_y, float mag_z,
    float dt);

// Get current orientation
orientation orientation_get(void);

#endif // ORIENTATION_H

