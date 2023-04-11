#include <Wire.h>
#include <MPU6050.h>
#include <PID_v1.h>
#include <Encoder.h>

// MPU-6050 setup
MPU6050 mpu;
int16_t ax, ay, az, gx, gy, gz;
double angle = 0;
const double RAD_TO_DEG = 180.0 / PI;

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
  mpu.initialize();
  delay(500);

  // Configure PID
  Setpoint = 0;
  Input = angle;
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(10);
  myPID.SetOutputLimits(-255, 255);

  // Motor pins
  pinMode(motorPinA, OUTPUT);
  pinMode(motorPinB, OUTPUT);
}

void loop() {
  // Read accelerometer and gyroscope data
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Calculate angle from accelerometer data
  angle = atan2(ay, sqrt(pow(ax, 2) + pow(az, 2))) * RAD_TO_DEG;

  // Update PID input
  Input = angle;

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

