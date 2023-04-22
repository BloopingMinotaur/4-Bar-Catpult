#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// MPU-6050 setup
Adafruit_MPU6050 mpu;
const float accelerometerSensitivity = 2048; // Sensitivity for ±16g range

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

  // Convert raw values to Gs
  float accel_x = a_event.acceleration.x ;
  float accel_y = a_event.acceleration.y ;
  float accel_z = a_event.acceleration.z ;

  // Print accelerometer values in Gs
  Serial.print("Accel X: "); Serial.print(accel_x);
  Serial.print(" Accel Y: "); Serial.print(accel_y);
  Serial.print(" Accel Z: "); Serial.println(accel_z);

  // Calculate and print the magnitude of acceleration
  float magnitude = sqrt(pow(accel_x, 2) + pow(accel_y, 2) + pow(accel_z, 2));
  Serial.print("Magnitude: "); Serial.println(magnitude);

  // Wait for a short time before the next reading
  delay(100);
}



