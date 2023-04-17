#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <PID_v1_bc.h>
#include <Encoder.h>

// MPU-6050 setup
Adafruit_MPU6050 mpu;
double ax, ay, az;
double acceleration = 0;

// Motor and encoder setup
const int motorPinA = 8;
const int motorPinB = 9;
const int encoderPinA = 2;
const int encoderPinB = 3;
Encoder encoder(encoderPinA, encoderPinB);

// PID setup
double Setpoint, Input, Output;
double Kp = 2, Ki = 5, Kd = 1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

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
  delay(500);

  // Configure PID
  Setpoint = /* Desired acceleration value */;
  Input = acceleration;
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(10);
  myPID.SetOutputLimits(-255, 255);

  // Motor pins
  pinMode(motorPinA, OUTPUT);
  pinMode(motorPinB, OUTPUT);
}

void loop() {
  // Read accelerometer data
  sensors_event_t a_event, g_event, temp_event;
  mpu.getEvent(&a_event, &g_event, &temp_event);

  // Update acceleration values
  ax = a_event.acceleration.x;
  ay = a_event.acceleration.y;
  az = a_event.acceleration.z;

  // Calculate acceleration magnitude
  acceleration = sqrt(pow(ax, 2) + pow(ay, 2) + pow(az, 2));

  // Update PID input
  Input = acceleration;

  // Compute PID
  myPID.Compute();

  // Control motor
  controlMotor(Output);
  
  // Read encoder value
  long encoderPosition = encoder.read();
}

void controlMotor(int speed) {
  if (speed > 0) {
    digitalWrite(motorPinA, HIGH);
    digitalWrite(motorPinB, LOW);
    analogWrite(5, speed);
  } else {
    digitalWrite(motorPinA, LOW);
    digitalWrite(motorPinB, HIGH);
    analogWrite(5, abs(speed));
  }
}



