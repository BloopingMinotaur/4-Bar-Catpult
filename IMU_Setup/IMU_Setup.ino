#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// MPU-6050 setup
Adafruit_MPU6050 mpu;
const float accelerometerSensitivity = 8192; // Sensitivity for ±4g range

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Initialize MPU-6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void loop() {
  // Read accelerometer and gyroscope data
  sensors_event_t a_event, g_event, temp_event;
  mpu.getEvent(&a_event, &g_event, &temp_event);

  // Print accelerometer values in Gs
  Serial.print("Accel X: "); Serial.print(a_event.acceleration.x);
  Serial.print(" Accel Y: "); Serial.print(a_event.acceleration.y);
  Serial.print(" Accel Z: "); Serial.println(a_event.acceleration.z);

  // Wait for a short time before the next reading
  delay(100);
}



