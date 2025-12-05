/**
 * Orientation sensor fusion using complementary filter
 * Combines accelerometer, gyroscope, and magnetometer data
 */

#include <math.h>
#include "orientation.h"


#define M_PI 3.14159265358979323846

// Complementary filter coefficient (0.0 = trust gyro, 1.0 = trust accel)
#define ALPHA_STATIONARY 0.1f  // 10% accelerometer when stationary (stronger correction)
#define ALPHA_MOVING 0.02f     // 2% accelerometer when moving

// Low-pass filter for gyro (reduces noise)
#define GYRO_LPF_ALPHA 0.8f  // 80% old value, 20% new value

// Gyro deadband threshold (deg/s) - values below this are treated as zero
#define GYRO_DEADBAND 0.5f

// Motion detection threshold (deg/s) - if gyro magnitude below this, consider stationary
#define MOTION_THRESHOLD 5.0f

// Truly stationary threshold (deg/s) - below this, uses sensors directly (no filter)
#define TRULY_STATIONARY_THRESHOLD 4.0f

// Gyro bias calibration
#define CALIBRATION_SAMPLES 100
static float gyro_bias_x = 0.0f;
static float gyro_bias_y = 0.0f;
static float gyro_bias_z = 0.0f;
static int calibration_count = 0;
static bool calibration_done = false;

// Current orientation
static orientation current_orientation = {0.0f, 0.0f, 0.0f};   // holds my current orientation

// Filtered gyro values (for noise reduction)
static float filtered_gyro_x = 0.0f;        // holds my filtered gyro values updated every call 
static float filtered_gyro_y = 0.0f;
static float filtered_gyro_z = 0.0f;

// Previous accelerometer values for stability detection
static float prev_accel_x = 0.0f;
static float prev_accel_y = 0.0f;
static float prev_accel_z = 0.0f;
static bool accel_initialized = false;       // flag to check if accelerometer changed if false it's likely stationary

// Initialize orientation filter
void orientation_init(void) {                // I call this in my main file before starting to feed data 
    current_orientation.roll = 0.0f;
    current_orientation.pitch = 0.0f;
    current_orientation.yaw = 0.0f;
    filtered_gyro_x = 0.0f;
    filtered_gyro_y = 0.0f;
    filtered_gyro_z = 0.0f;
    gyro_bias_x = 0.0f;
    gyro_bias_y = 0.0f;
    gyro_bias_z = 0.0f;
    calibration_count = 0;
    calibration_done = false;
}

// Calibrate gyro bias (call when device is stationary)
void orientation_calibrate_gyro(float gyro_x, float gyro_y, float gyro_z) {
    if (calibration_count < CALIBRATION_SAMPLES) {
        gyro_bias_x += gyro_x;
        gyro_bias_y += gyro_y;
        gyro_bias_z += gyro_z;
        calibration_count++;
    } else if (!calibration_done) {
        // Average the samples to get zero_offset for each axis 
        gyro_bias_x /= CALIBRATION_SAMPLES;
        gyro_bias_y /= CALIBRATION_SAMPLES;
        gyro_bias_z /= CALIBRATION_SAMPLES; 
        calibration_done = true;                             // flag to avoid recalibrating in subsequent calls
    }
}

// Calculate pitch and roll from accelerometer (when stationary)
static void accel_to_angles(float accel_x, float accel_y, float accel_z, 
                            float *pitch, float *roll) {
    // Calculate pitch and roll from accelerometer
    *pitch = atan2f(-accel_x, sqrtf(accel_y * accel_y + accel_z * accel_z)) * 180.0f / M_PI;
    *roll = atan2f(accel_y, accel_z) * 180.0f / M_PI;
}


// Update orientation using complementary filter
bool orientation_update(float accel_x, float accel_y, float accel_z,
                       float gyro_x, float gyro_y, float gyro_z,
                       float mag_x, float mag_y, float mag_z,
                       float dt) { 

    // Subtract gyro bias if calibrated
    if (calibration_done) {
        gyro_x -= gyro_bias_x;
        gyro_y -= gyro_bias_y;
        gyro_z -= gyro_bias_z;
    }
    
    // Apply deadband to very small gyro values (treat as zero)
    if (fabsf(gyro_x) < GYRO_DEADBAND) gyro_x = 0.0f;
    if (fabsf(gyro_y) < GYRO_DEADBAND) gyro_y = 0.0f;
    if (fabsf(gyro_z) < GYRO_DEADBAND) gyro_z = 0.0f;
    
    // Low-pass filter gyro to reduce noise
    filtered_gyro_x = GYRO_LPF_ALPHA * filtered_gyro_x + (1.0f - GYRO_LPF_ALPHA) * gyro_x;
    filtered_gyro_y = GYRO_LPF_ALPHA * filtered_gyro_y + (1.0f - GYRO_LPF_ALPHA) * gyro_y;
    filtered_gyro_z = GYRO_LPF_ALPHA * filtered_gyro_z + (1.0f - GYRO_LPF_ALPHA) * gyro_z;
    
    // Detect if device is stationary (low gyro magnitude)
    float gyro_magnitude = sqrtf(filtered_gyro_x * filtered_gyro_x + 
                                 filtered_gyro_y * filtered_gyro_y + 
                                 filtered_gyro_z * filtered_gyro_z);
    bool is_stationary = (gyro_magnitude < MOTION_THRESHOLD);
    
    // Check accelerometer stability (if values aren't changing much, device is stationary)
    float accel_change = 0.0f;
    if (accel_initialized) {
        float dx = accel_x - prev_accel_x;
        float dy = accel_y - prev_accel_y;
        float dz = accel_z - prev_accel_z;
        accel_change = sqrtf(dx*dx + dy*dy + dz*dz);
    } else {
        accel_initialized = true;
    }
    prev_accel_x = accel_x;
    prev_accel_y = accel_y;
    prev_accel_z = accel_z;
    
    // Truly stationary if gyro is low AND accelerometer is stable
    bool truly_stationary = (is_stationary && 
                            gyro_magnitude < TRULY_STATIONARY_THRESHOLD &&
                            accel_change < 0.05f);  // Accel change < 0.05g
    
    // Use stronger accelerometer correction when stationary    
    float alpha;
    if (is_stationary){
        alpha = ALPHA_STATIONARY;
    } else {
        alpha = ALPHA_MOVING;
    }
    
    // Calculate angles from accelerometer (pitch and roll)
    float accel_pitch, accel_roll;
    accel_to_angles(accel_x, accel_y, accel_z, &accel_pitch, &accel_roll);
    
    // Integrate gyro to get angle change
    float gyro_pitch_delta = filtered_gyro_y * dt;
    float gyro_roll_delta = filtered_gyro_x * dt;
    float gyro_yaw_delta = filtered_gyro_z * dt;
    
    // if our device is stationary we trust the accelerometer and magnetometer for roll and pitch 
    if (truly_stationary) {
        current_orientation.roll  = accel_roll;
        current_orientation.pitch = accel_pitch;
    } else {
        // When moving, use complementary filter for smooth tracking
        current_orientation.roll = (1.0f - alpha) * (current_orientation.roll + gyro_roll_delta) + 
                                   alpha * accel_roll;
        current_orientation.pitch = (1.0f - alpha) * (current_orientation.pitch + gyro_pitch_delta) + 
                                    alpha * accel_pitch;
    }
    
    // Yaw (heading) from magnetometer
    float yaw_from_mag = atan2f(mag_x, mag_y) * 180.0f / M_PI; 

    if (yaw_from_mag < 0) yaw_from_mag += 360.0f;
    
    // if our device is stationary then we trust the magnetometer completely for yaw 
    if (truly_stationary) {
        current_orientation.yaw = yaw_from_mag;
    } else {
        // When moving, use complementary filter
        float yaw_alpha;
        if (is_stationary){
            yaw_alpha = 0.2f;  
        } else {
            yaw_alpha = ALPHA_MOVING;
        }
        current_orientation.yaw = (1.0f - yaw_alpha) * (current_orientation.yaw + gyro_yaw_delta) + 
                                  yaw_alpha * yaw_from_mag;
    }
    
    // Normalize yaw to 0-360

    if (current_orientation.yaw < 0) {
        current_orientation.yaw += 360.0f;
    }

    if (current_orientation.yaw >= 360.0f) {
        current_orientation.yaw -= 360.0f;
    }
    return true;
}

// Get current orientation (roll, pitch, yaw)
orientation orientation_get(void) {
    return current_orientation;           
}

