#include <Adafruit_NeoPixel.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

#define NEOPIXEL_PIN 10
#define NEOPIXEL_NUM 10

MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

void setup() {
    Wire.begin();
    Serial.begin(9600);

    // initialize MPU6050
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();
    // verify IMU connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
}

void loop() {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    Serial.print("a/g:\t");
    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.print(az); Serial.print("\t");
    Serial.print(gx); Serial.print("\t");
    Serial.print(gy); Serial.print("\t");
    Serial.println(gz);
    delay(100);
}

