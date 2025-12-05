/**
 * Hunter Adams (vha3@cornell.edu)
 * 
 *
 */

 #define IMU_ADDRESS 0x68
 #define IMU_I2C_CHAN i2c1
 #define IMU_SDA_PIN  10 // yellow - white 
 #define IMU_SCL_PIN  11 // orange - green 
 #define IMU_I2C_BAUD_RATE 400000
 
 // Fixed point data type
 typedef signed int fix15 ;
 #define multfix15(a,b) ((fix15)(((( signed long long)(a))*(( signed long long)(b)))>>16)) 
 #define float2fix15(a) ((fix15)((a)*65536.0f)) // 2^16
 #define fix2float15(a) ((float)(a)/65536.0f) 
 #define int2fix15(a) ((a)<<16)
 #define fix2int15(a) ((a)>>16)
 #define divfix(a,b) ((fix15)(((( signed long long)(a) << 16 / (b)))))
 // Parameter values
 #define oneeightyoverpi 3754936
 #define zeropt001 65
 #define zeropt999 65470
 #define zeropt01 655
 #define zeropt99 64880
 #define zeropt1 6553
 #define zeropt9 58982
 
// IMU data structure
typedef struct {
    float accel_x;  // Acceleration X (g's)
    float accel_y;  // Acceleration Y (g's)
    float accel_z;  // Acceleration Z (g's)
    float gyro_x;   // Gyro X (deg/s)
    float gyro_y;   // Gyro Y (deg/s)
    float gyro_z;   // Gyro Z (deg/s)
} imu_data;

// VGA primitives - usable in main
void imu_init(void);
void mpu6050_reset(void);
void mpu6050_read_raw(fix15 accel[3], fix15 gyro[3]);
imu_data imu_read(void);