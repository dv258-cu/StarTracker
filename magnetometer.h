/**
 * Tony Kariuki (akk85@cornell.edu)
 * 
 * magnetometer.h
 *
 */

/* 3 axisMagnetometer 
* -+ 30 Gauss
* I2C address 0x30
SCL = GPIO 5 (GP5)
SDA = GPIO 4 (GP4)
*/
#include <stdbool.h>
// Macros for fixed-point arithmetic (faster than floating point)
typedef signed int fix15 ;
#define multfix15(a,b) ((fix15)(((( signed long long)(a))*(( signed long long)(b)))>>16)) 
#define float2fix15(a) ((fix15)((a)*65536.0f)) // 2^16
#define fix2float15(a) ((float)(a)/65536.0f) 
#define int2fix15(a) ((a)<<16)
#define fix2int15(a) ((a)>>16)
#define divfix(a,b) ((fix15)(((( signed long long)(a) << 16 / (b)))))


// ==== MMC5603 Register Map (subset we need) ====
// Output registers (auto-increment starting at 0x00)
#define MMC5603_REG_XOUT0       0x00   // X[19:12]
#define MMC5603_REG_STATUS      0x18   // status bits (Meas done = bit1)
#define MMC5603_REG_ODR         0x1A   // output data rate (not used yet)
#define MMC5603_REG_CTRL0       0x1B   // control (trigger single measure, etc.)
#define MMC5603_REG_CTRL1       0x1C   // control (continuous mode etc., not yet)
#define MMC5603_REG_CTRL2       0x1D   // control (set/reset etc., not yet)
#define MMC5603_REG_PRODUCTID   0x39   // should read 0x10 on MMC5603


// ==== Bit masks we’ll use ====
#define MMC5603_STATUS_MEAS_DONE   0x02  // STATUS bit1: magnetic measurement complete
#define MMC5603_CTRL0_TM_M         0x20  // CTRL0 bit5: trigger one magnetic measurement
#define MMC5603_CTRL0_AUTO_SR_EN   0x01  // CTRL0 bit0: enable auto set/reset



#define MAG_ADDRESS 0x30  // Primary address, some boards use 0x60
#define MAG_ADDRESS_ALT 0x60  // Alternative address for MMC5603
#define I2C_CHAN       i2c0
#define SDA_PIN        4   // GP4 = I2C0 SDA
#define SCL_PIN        5   // GP5 = I2C0 SCL
#define I2C_BAUD_RATE  400000  

#define PI 3.14159265358979323846

// Calibrated magnetic field values
typedef struct {
    float x;  
    float y;  
    float z;  
} magnetometer;


// I2C and magnetometer functions
void magnetometer_init(void);
magnetometer magnetometer_calibrate(void);
float magnetometer_get_heading(float magnetic_x, float magnetic_y);