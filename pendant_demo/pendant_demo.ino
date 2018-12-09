// This is the demo program for this bluetooth-neopixel pendant.
// Author: Yang Liu, 12/04/2018.

// Bluetooth command format: keyword + space + argument(optional)
// Singular commands
    // "rainbow", rainbow mode
// Combinational commands:
    // "brightness ##", 1 min brightness, 0 max brightness, 255 just below max
    // "all color_name", set all neopixels to same color
    // "led# color_name", addressing neopixels individuallly, number from 0 to 9
    // "gravity color_name", gravity mode
    // "shade color_name", shade mode
// Supported color names:
// white, black, maroon, brown, olive, teal, navy, red, orange, yellow, lime
// green, cyan, blue, pruple, magenta, grey, pink, apricot, beige, mint, lavender

// The circuit board shares the same body frame with the IMU.
// If facing the PCB, x axis points to the right,
// y axis points up to the switch, z axis points out from the board.

// List of distinct colors:
// https://sashat.me/2017/01/11/list-of-20-simple-distinct-colors/

#include <Adafruit_NeoPixel.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

#define NEOPIXEL_PIN 10
#define NEOPIXEL_NUM 10
#define MPU6050_ADDRESS 0x68
#define BUFFER_LEN 20

// MPU6050 variables
MPU6050 mpu6050;
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
float ax, ay, az;       // Stores the real accel value in g's
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
float gx, gy, gz;       // Stores the real gyro value in degrees per seconds
float aRes = 2.0/32768.0;
float gRes = 250.0/32768.0;
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // vector to hold quaternion
float roll, pitch, yaw;  // in degrees
// for quaternion update
float GyroMeasError = PI * (40.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;  // compute beta
float GyroMeasDrift = PI * (2.0f / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
float deltat = 0.0f;
float p_vect[3];

// Neopixel variables
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NEOPIXEL_NUM, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
uint32_t color_white = pixels.Color(255,255,255);
uint32_t color_black = pixels.Color(0,0,0);
uint32_t color_maroon = pixels.Color(128,0,0);
uint32_t color_brown = pixels.Color(170,110,40);
uint32_t color_olive = pixels.Color(128,128,0);
uint32_t color_teal = pixels.Color(0,128,128);
uint32_t color_navy = pixels.Color(0,0,128);
uint32_t color_red = pixels.Color(230,25,75);
uint32_t color_orange = pixels.Color(245,130,48);
uint32_t color_yellow = pixels.Color(255,255,25);
uint32_t color_lime = pixels.Color(210,245,60);
uint32_t color_green = pixels.Color(60,180,75);
uint32_t color_cyan = pixels.Color(70,240,240);
uint32_t color_blue = pixels.Color(0,130,200);
uint32_t color_purple = pixels.Color(145,30,180);
uint32_t color_magenta = pixels.Color(240,50,230);
uint32_t color_grey = pixels.Color(128,128,128);
uint32_t color_pink = pixels.Color(250,190,190);
uint32_t color_apricot = pixels.Color(255,215,180);
uint32_t color_beige = pixels.Color(255,250,200);
uint32_t color_mint = pixels.Color(170,255,195);
uint32_t color_lavender = pixels.Color(170,190,255);
uint8_t brightness;
uint32_t pixel_colors[NEOPIXEL_NUM];
int rainbow_index = 0;

// bluetooth control variables
char buffer[BUFFER_LEN];
uint8_t buffer_index;
enum MODE {ADDRESSING_MODE, RAINBOW_MODE, GRAVITY_MODE, SHADE_MODE};
MODE mode;

// flow control
unsigned long time_last, time_now;  // microsecond
unsigned long period = 20000;  // period for main loop
// debug print control
unsigned long debug_period = 100000;
uint8_t debug_freq = (uint8_t)((float)debug_period/period);
uint8_t debug_count = 0;

void setup() {
    Serial.begin(9600);  // USB
    Serial1.begin(9600);  // bluetooth

    // mpu6050
    Wire.begin();  // join I2C bus
    calibrateGyro();
    mpu6050.initialize();

    // neopixels
    mode = ADDRESSING_MODE;
    brightness = 50;
    set_all(&color_orange);
    pixels.setBrightness(brightness);
    pixels.show();

    time_last = micros();
}

void loop() {
    time_now = micros();
    if ((time_now - time_last) > period) {
        deltat = (time_now - time_last)/1000000.0f;
        time_last = time_now;

        // read raw accel/gyro measurements from device
        mpu6050.getMotion6(&accelCount[0], &accelCount[1], &accelCount[2],
            &gyroCount[0], &gyroCount[1], &gyroCount[2]);
        ax = (float)accelCount[0] * aRes;  // get actual g value
        ay = (float)accelCount[1] * aRes;
        az = (float)accelCount[2] * aRes;
        gx = (float)gyroCount[0] * gRes;  // get actual gyro value
        gy = (float)gyroCount[1] * gRes;
        gz = (float)gyroCount[2] * gRes;

        // quaternion update
        MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.f, gy*PI/180.f, gz*PI/180.f);
        // roll = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
        // pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
        // yaw = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
        // roll *= 180.0f / PI;
        // pitch *= 180.0f / PI;
        // yaw *= 180.0f / PI;

        // projection of pointing-up unit vector in inertial frame to IMU's body frame
        p_vect[0] = 2*(q[1]*q[3] - q[0]*q[2]);
        p_vect[1] = 2*(q[0]*q[1] + q[2]*q[3]);
        p_vect[2] = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];

        // // debug print
        // debug_count = debug_count + 1;
        // if (debug_count > debug_freq) {
        //     debug_count = 0;
        //     // Serial.print(ax); Serial.print("\t");
        //     // Serial.print(ay); Serial.print("\t");
        //     // Serial.print(az); Serial.print("\t");
        //     // Serial.print(gx); Serial.print("\t");
        //     // Serial.print(gy); Serial.print("\t");
        //     // Serial.println(gz);
        //     // Serial.print(roll); Serial.print("\t");
        //     // Serial.print(pitch); Serial.print("\t");
        //     // Serial.println(yaw);
        //     Serial.print(p_vect[0]); Serial.print("\t");
        //     Serial.print(p_vect[1]); Serial.print("\t");
        //     Serial.println(p_vect[2]);
        // }

        // read bluetooth command
        for (uint8_t i=0; i<BUFFER_LEN; i++) {
            buffer[i] = 0;
        }
        buffer_index = 0;
        while (Serial1.available() && buffer_index < BUFFER_LEN) {
            buffer[buffer_index] = Serial1.read();
            if (buffer[buffer_index] == '\n') {
                break;
            }
            buffer_index++;
        }
        if (buffer_index != 0 && buffer_index != BUFFER_LEN &&
            buffer[buffer_index] == '\n') {
            // proceed if buffer didn't overflow and complete command is received
            // look for the first space
            int8_t space_pos = -1;
            for (uint8_t i=0; i<buffer_index; i++) {
                if (buffer[i] == ' ') {
                    space_pos = i;
                    break;
                }
            }
            if (space_pos == 0 || space_pos == buffer_index-1) {
                // invalid if space is the first or last character
                return;
            }
            // match the command
            if (space_pos == -1) {
                // space not found, attempt to match the singular commands
                // list all singular commands in below
                if (match_cmd(buffer, buffer_index, "rainbow")) {
                    mode = RAINBOW_MODE;
                    set_all(&color_black);
                }
            }
        }

        // update pixel color variables
        if (mode == RAINBOW_MODE) {
            Serial.println("in rainbow mode");
            for (uint8_t i=0; i<NEOPIXEL_NUM; i++) {
                pixels.setPixelColor(i, pixel_wheel((i * 256 / 25 + rainbow_index) & 255));
            }
        }

//        Serial.println("updating neopixel colors");
//        pixels.setBrightness(brightness);
//        pixels.show();
    }
}

// check if input string match the command
uint8_t match_cmd(char* input, uint8_t input_len, char* cmd) {
    if (input_len != strlen(cmd)) {
        return 0;
    }
    for (uint8_t i=0; i<input_len; i++) {
        if (*(input+i) != *(cmd+i)) {
            return 0;
        }
    }
    return 1;
}

// set all neopixels to one color
void set_all(uint32_t* color) {
    for (uint8_t i=0; i<NEOPIXEL_NUM; i++) {
        pixels.setPixelColor(i, &color);
    }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t pixel_wheel(byte WheelPos) {
    WheelPos = 255 - WheelPos;
    if(WheelPos < 85)
    {
        return pixels.Color(255 - WheelPos * 3, 0, WheelPos * 3);
    }
    else if(WheelPos < 170)
    {
        WheelPos -= 85;
        return pixels.Color(0, WheelPos * 3, 255 - WheelPos * 3);
    }
    else
    {
        WheelPos -= 170;
        return pixels.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
    }
}

// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration and rotation rate to produce a quaternion-based estimate of relative
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz)
{
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];         // short name local variable for readability
    float norm;                                               // vector norm
    float f1, f2, f3;                                         // objetive funcyion elements
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
    float qDot1, qDot2, qDot3, qDot4;
    float hatDot1, hatDot2, hatDot3, hatDot4;
    float gerrx, gerry, gerrz, gbiasx, gbiasy, gbiasz;        // gyro bias error

    // Auxiliary variables to avoid repeated arithmetic
    float _halfq1 = 0.5f * q1;
    float _halfq2 = 0.5f * q2;
    float _halfq3 = 0.5f * q3;
    float _halfq4 = 0.5f * q4;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    float _2q1q3 = 2.0f * q1 * q3;
    float _2q3q4 = 2.0f * q3 * q4;

    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Compute the objective function and Jacobian
    f1 = _2q2 * q4 - _2q1 * q3 - ax;
    f2 = _2q1 * q2 + _2q3 * q4 - ay;
    f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - az;
    J_11or24 = _2q3;
    J_12or23 = _2q4;
    J_13or22 = _2q1;
    J_14or21 = _2q2;
    J_32 = 2.0f * J_14or21;
    J_33 = 2.0f * J_11or24;

    // Compute the gradient (matrix multiplication)
    hatDot1 = J_14or21 * f2 - J_11or24 * f1;
    hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
    hatDot3 = J_12or23 * f2 - J_33 *f3 - J_13or22 * f1;
    hatDot4 = J_14or21 * f1 + J_11or24 * f2;

    // Normalize the gradient
    norm = sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
    hatDot1 /= norm;
    hatDot2 /= norm;
    hatDot3 /= norm;
    hatDot4 /= norm;

    // Compute estimated gyroscope biases
    gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
    gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
    gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;

    // Compute and remove gyroscope biases
    gbiasx += gerrx * deltat * zeta;
    gbiasy += gerry * deltat * zeta;
    gbiasz += gerrz * deltat * zeta;
    gx -= gbiasx;
    gy -= gbiasy;
    gz -= gbiasz;

    // Compute the quaternion derivative
    qDot1 = -_halfq2 * gx - _halfq3 * gy - _halfq4 * gz;
    qDot2 =  _halfq1 * gx + _halfq3 * gz - _halfq4 * gy;
    qDot3 =  _halfq1 * gy - _halfq2 * gz + _halfq4 * gx;
    qDot4 =  _halfq1 * gz + _halfq2 * gy - _halfq3 * gx;

    // Compute then integrate estimated quaternion derivative
    q1 += (qDot1 -(beta * hatDot1)) * deltat;
    q2 += (qDot2 -(beta * hatDot2)) * deltat;
    q3 += (qDot3 -(beta * hatDot3)) * deltat;
    q4 += (qDot4 -(beta * hatDot4)) * deltat;

    // Normalize the quaternion
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
    norm = 1.0f/norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;
}

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
    Wire.beginTransmission(address);  // Initialize the Tx buffer
    Wire.write(subAddress);           // Put slave register address in Tx buffer
    Wire.write(data);                 // Put data in Tx buffer
    Wire.endTransmission();           // Send the Tx buffer
}

uint8_t readByte(uint8_t address, uint8_t subAddress)
{
    uint8_t data; // `data` will store the register data     
    Wire.beginTransmission(address);         // Initialize the Tx buffer
    Wire.write(subAddress);                  // Put slave register address in Tx buffer
    Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
    Wire.requestFrom(address, (uint8_t) 1);  // Read one byte from slave register address 
    data = Wire.read();                      // Fill Rx buffer with result
    return data;                             // Return data read from slave register
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
    Wire.beginTransmission(address);   // Initialize the Tx buffer
    Wire.write(subAddress);            // Put slave register address in Tx buffer
    Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
    uint8_t i = 0;
    Wire.requestFrom(address, count);  // Read bytes from slave register address 
    while (Wire.available()) {
        dest[i++] = Wire.read(); }         // Put read results in the Rx buffer
}

// calibrate only gyro in mpu6050, by accumulating readings and taking average
// the device should be static along the time
// reason not calibrate accelerometer is that device orientation is not upright
void calibrateGyro()
{
    uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
    uint16_t ii, packet_count, fifo_count;
    int32_t gyro_bias[3] = {0, 0, 0};

    // reset device, reset all registers, clear gyro and accelerometer bias registers
    writeByte(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
    delay(100);  

    // get stable time source
    // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
    writeByte(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x01);  
    writeByte(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_2, 0x00); 
    delay(200);

    // Configure device for bias calculation
    writeByte(MPU6050_ADDRESS, MPU6050_RA_INT_ENABLE, 0x00);   // Disable all interrupts
    writeByte(MPU6050_ADDRESS, MPU6050_RA_FIFO_EN, 0x00);      // Disable FIFO
    writeByte(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x00);   // Turn on internal clock source
    writeByte(MPU6050_ADDRESS, MPU6050_RA_I2C_MST_CTRL, 0x00); // Disable I2C master
    writeByte(MPU6050_ADDRESS, MPU6050_RA_USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
    writeByte(MPU6050_ADDRESS, MPU6050_RA_USER_CTRL, 0x0C);    // Reset FIFO and DMP
    delay(15);

    // Configure MPU6050 gyro and accelerometer for bias calculation
    writeByte(MPU6050_ADDRESS, MPU6050_RA_CONFIG, 0x01);      // Set low-pass filter to 188 Hz
    writeByte(MPU6050_ADDRESS, MPU6050_RA_SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
    writeByte(MPU6050_ADDRESS, MPU6050_RA_GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    writeByte(MPU6050_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

    uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
    uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
    writeByte(MPU6050_ADDRESS, MPU6050_RA_USER_CTRL, 0x40);   // Enable FIFO  
    writeByte(MPU6050_ADDRESS, MPU6050_RA_FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 1024 bytes in MPU-6050)
    delay(80); // accumulate 80 samples in 80 milliseconds = 960 bytes

    // At end of sample accumulation, turn off FIFO sensor read
    writeByte(MPU6050_ADDRESS, MPU6050_RA_FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
    readBytes(MPU6050_ADDRESS, MPU6050_RA_FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
    fifo_count = ((uint16_t)data[0] << 8) | data[1];
    packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

    for (ii = 0; ii < packet_count; ii++) {
        // int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
        int16_t gyro_temp[3] = {0, 0, 0};
        readBytes(MPU6050_ADDRESS, MPU6050_RA_FIFO_R_W, 12, &data[0]); // read data for averaging
        // accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
        // accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
        // accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;    
        gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
        gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
        gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

        // accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
        // accel_bias[1] += (int32_t) accel_temp[1];
        // accel_bias[2] += (int32_t) accel_temp[2];
        gyro_bias[0]  += (int32_t) gyro_temp[0];
        gyro_bias[1]  += (int32_t) gyro_temp[1];
        gyro_bias[2]  += (int32_t) gyro_temp[2];
    }
    // accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    // accel_bias[1] /= (int32_t) packet_count;
    // accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;

    // if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
    // else {accel_bias[2] += (int32_t) accelsensitivity;}

    // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
    data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
    data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
    data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
    data[3] = (-gyro_bias[1]/4)       & 0xFF;
    data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
    data[5] = (-gyro_bias[2]/4)       & 0xFF;

    // Push gyro biases to hardware registers
    writeByte(MPU6050_ADDRESS, MPU6050_RA_XG_OFFS_USRH, data[0]);// might not be supported in MPU6050
    writeByte(MPU6050_ADDRESS, MPU6050_RA_XG_OFFS_USRL, data[1]);
    writeByte(MPU6050_ADDRESS, MPU6050_RA_YG_OFFS_USRH, data[2]);
    writeByte(MPU6050_ADDRESS, MPU6050_RA_YG_OFFS_USRL, data[3]);
    writeByte(MPU6050_ADDRESS, MPU6050_RA_ZG_OFFS_USRH, data[4]);
    writeByte(MPU6050_ADDRESS, MPU6050_RA_ZG_OFFS_USRL, data[5]);
}

