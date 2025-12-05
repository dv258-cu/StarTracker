/**
 * Main file for StarTracker
 * tracker.c
 *
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "magnetometer.h"
#include "libs/mpu6050.h"
#include "orientation.h"
#include "libs/pt_cornell_rp2040_v1_4.h"
#include "libs/vga16_graphics_v2.h"
#include "libs/vga_graphics_v3.h"

#define PI 3.14159265358979323846

// Global variables for sensor data 
static magnetometer magnetometer_data;
static imu_data imu_data_global;
static orientation current_orientation;
static const float dt = 0.05f;  // 50ms = 0.05 seconds

// Display buffer for text
static char display_buffer[100];
static char compass_buffer[100];

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

// Calibrate gyro bias (collect samples while device is stationary)
static void calibrate_gyro(void) {
    printf("Calibrating gyro keep device still for 5 seconds\r\n");
    for (int i = 0; i < 100; i++) {
        imu_data imu_value = imu_read();
        orientation_calibrate_gyro(imu_value.gyro_x, imu_value.gyro_y, imu_value.gyro_z);
        sleep_ms(50);
    }
    printf("Calibration complete!\n");
}

// Draw compass circle 
static void draw_compass_circle(int center_x, int center_y, int radius, float heading) {

    int text_area_height = 30;  
    fillRect(center_x - radius - 5, center_y - radius - 5, 
             (radius + 5) * 2, (radius + 5) * 2 + text_area_height, BLACK);
    
    drawCircle(center_x, center_y, radius, WHITE);
    
    // North (0 degrees, top)
    drawLine(center_x, center_y - radius, 
             center_x, center_y - radius + 10, WHITE);
    setCursor(center_x - 3, center_y - radius - 12);
    writeString("N");
    
    // East (90 degrees, right)
    drawLine(center_x + radius, center_y, 
             center_x + radius - 10, center_y, WHITE);
    setCursor(center_x + radius + 5, center_y - 3);
    writeString("E");
    
    // South (180 degrees, bottom)
    drawLine(center_x, center_y + radius, 
             center_x, center_y + radius - 10, WHITE);
    setCursor(center_x - 3, center_y + radius + 12);
    writeString("S");
    
    // West (270 degrees, left)
    drawLine(center_x - radius, center_y, 
             center_x - radius + 10, center_y, WHITE);
    setCursor(center_x - radius - 8, center_y - 3);
    writeString("W");
    
    // Draw degree markers every 30 degrees
    for (int deg = 0; deg < 360; deg += 30) {
        float angle_rad = (deg * PI / 180.0f);
        int x1 = center_x + (int)((radius - 5) * sin(angle_rad));
        int y1 = center_y - (int)((radius - 5) * cos(angle_rad));
        int x2 = center_x + (int)(radius * sin(angle_rad));
        int y2 = center_y - (int)(radius * cos(angle_rad));
        drawLine(x1, y1, x2, y2, WHITE);
    }
    
    float adjusted_angle = (90.0f - heading) * PI / 180.0f;
    
    // Calculate arrow endpoint on circle
    int arrow_x = center_x + (int)((radius - 5) * cos(adjusted_angle));
    int arrow_y = center_y - (int)((radius - 5) * sin(adjusted_angle));
    
    drawLine(center_x, center_y, arrow_x, arrow_y, YELLOW);
    
    // Draw arrowhead (small triangle)
    float arrow_angle1 = adjusted_angle + 0.3f; // ~17 degrees offset
    float arrow_angle2 = adjusted_angle - 0.3f;
    int arrowhead_length = 10;
    int arrow_x1 = arrow_x + (int)(arrowhead_length * cos(arrow_angle1));
    int arrow_y1 = arrow_y - (int)(arrowhead_length * sin(arrow_angle1));
    int arrow_x2 = arrow_x + (int)(arrowhead_length * cos(arrow_angle2));
    int arrow_y2 = arrow_y - (int)(arrowhead_length * sin(arrow_angle2));
    
    drawLine(arrow_x, arrow_y, arrow_x1, arrow_y1, YELLOW);
    drawLine(arrow_x, arrow_y, arrow_x2, arrow_y2, YELLOW);
    drawLine(arrow_x1, arrow_y1, arrow_x2, arrow_y2, YELLOW);
    
    // Draw center point
    drawCircle(center_x, center_y, 2, YELLOW);
    
    // Clear and display heading value below circle
    // Clear a rectangle for the text to avoid overwriting issues
    int text_x = center_x - 50;
    int text_y = center_y + radius + 15;
    int text_w = 100;
    int text_h = 15;
    fillRect(text_x, text_y, text_w, text_h, BLACK);
    
    setCursor(text_x + 5, text_y);
    sprintf(compass_buffer, "Heading: %5.1f", heading);
    writeString(compass_buffer);
}

// VGA Display Thread
static PT_THREAD (protothread_display(struct pt *pt))
{
    PT_BEGIN(pt);
    
    static int y_position;
    static int line_height = 20;

    static int data_box_x = 10;
    static int data_box_y = 10;
    static int data_box_w = 150;
    static int data_box_h = 150;
    
    static int circle_center_x = 450;
    static int circle_center_y = 200;
    static int circle_radius = 120;
    
    // Heading smoothing filter (exponential moving average)
    static float smoothed_heading = 0.0f;
    static bool first_heading = true;
    const float smoothing_factor = 0.5f; 
    
    // Clear screen initially
    fillRect(0, 0, 640, 480, BLACK);
    
    while(1) {
        // Read sensors
        magnetometer_data = magnetometer_calibrate();
        imu_data_global = imu_read();
        
        // Update orientation using sensor fusion
        orientation_update(imu_data_global.accel_x, imu_data_global.accel_y, imu_data_global.accel_z,
                         imu_data_global.gyro_x, imu_data_global.gyro_y, imu_data_global.gyro_z,
                         magnetometer_data.x, magnetometer_data.y, magnetometer_data.z,
                         dt);
        
        current_orientation = orientation_get();
        
        fillRect(data_box_x + 2, data_box_y + 2, data_box_w - 4, data_box_h - 4, BLACK);
        
        drawRect(data_box_x, data_box_y, data_box_w, data_box_h, WHITE);
        
        setTextColor(WHITE);
        setTextSize(1);
        setTextWrap(0);
        
        setCursor(data_box_x + 5, data_box_y + 5);
        writeString("StarTracker Sensor Data");
        
        y_position = data_box_y + 30;
        
        // Display Orientation (Euler angles)
        setCursor(data_box_x + 5, y_position);
        writeString("Orientation (deg):");
        y_position += line_height;
        setCursor(data_box_x + 15, y_position);
        sprintf(display_buffer, "Roll:  %6.1f", current_orientation.roll);
        writeString(display_buffer);
        y_position += line_height;
        setCursor(data_box_x + 15, y_position);
        sprintf(display_buffer, "Pitch: %6.1f", current_orientation.pitch);
        writeString(display_buffer);
        y_position += line_height;
        setCursor(data_box_x + 15, y_position);
        sprintf(display_buffer, "Yaw:   %6.1f", current_orientation.yaw);
        writeString(display_buffer);
        
        float raw_heading = current_orientation.yaw;
        
        // Normalize yaw to 0-360 range if needed
        if (raw_heading < 0.0f) {
            raw_heading += 360.0f;
        }
        if (raw_heading >= 360.0f) {
            raw_heading -= 360.0f;
        }
        
        // Light smoothing filter for the fused heading (it's already smoothed by orientation system)
        if (first_heading) {
            smoothed_heading = raw_heading;
            first_heading = false;
        } else {
            float diff = normalize_angle_diff(smoothed_heading, raw_heading);
            
            // Light smoothing since orientation system already does fusion
            float adaptive_factor = 0.2f;  // Light smoothing
            if (fabsf(diff) > 5.0f) {
                adaptive_factor = 0.5f;  // Faster response for larger changes
            }
            
            smoothed_heading += diff * adaptive_factor;
            if (smoothed_heading < 0.0f) {
                smoothed_heading += 360.0f;
            }
            if (smoothed_heading >= 360.0f) {
                smoothed_heading -= 360.0f;
            }
        }
        
        float heading = smoothed_heading;
        
        draw_compass_circle(circle_center_x, circle_center_y, circle_radius, heading);
        
        // Print heading to serial
        printf("Heading: %5.1f\r\n", heading);

        
        PT_YIELD_usec(50000);
    }
    
    PT_END(pt);
}

int main(void) 
{
    stdio_init_all();
    sleep_ms(2000); 

    initVGA();

    // Initialize the sensors 
    magnetometer_init();
    imu_init();
    mpu6050_reset();

    orientation_init();
    
    calibrate_gyro();
    
    pt_add_thread(protothread_display);
    
    pt_schedule_start;
}
