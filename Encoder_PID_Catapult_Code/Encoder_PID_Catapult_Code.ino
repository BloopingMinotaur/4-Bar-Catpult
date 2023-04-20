#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <PID_v1_bc.h>
#include <Encoder.h>

// MPU-6050 setup
Adafruit_MPU6050 mpu;
double ax, ay, az;

// Motor and encoder setup
const int motorPinA = 8;
const int motorPinB = 9;
const int encoderPinA = 2;
const int encoderPinB = 3;
Encoder encoder(encoderPinA, encoderPinB);

// Data collection settings
const int encoderCountsPerRevolution = 40 * 40;  // one revolution of the throwing arm
const int maxDataPoints = 1000;
double imuData[maxDataPoints][3];  // Buffer to store IMU data
int dataCounter = 0;
bool dataCollectionComplete = false;
long previousEncoderCount = 0;

// PID setup for constant motor speed
const double desiredRPM = 400;
const double countsPerRevolution = 40;
const double desiredEncoderCountsPerSecond = (desiredRPM / 60) * countsPerRevolution;
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
  Setpoint = desiredEncoderCountsPerSecond;
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(100);
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

  // Read encoder value and calculate counts per second
  long currentEncoderCount = encoder.read();
  long deltaEncoderCount = currentEncoderCount - previousEncoderCount;
  previousEncoderCount = currentEncoderCount;
  Input = deltaEncoderCount;

  // Collect data for one revolution of the throwing arm
  if (!dataCollectionComplete && currentEncoderCount >= encoderCountsPerRevolution) {
    dataCollectionComplete = true;
    Serial.println("Data collection complete.");
  }

  if (!dataCollectionComplete) {
    imuData[dataCounter][0] = ax;
    imuData[dataCounter][1] = ay;
    imuData[dataCounter][2] = az;
    dataCounter++;

    if (dataCounter >= maxDataPoints) {
      dataCollectionComplete = true;
      Serial.println("Data collection complete (buffer full).");
    }
  }

  // Compute PID
  myPID.Compute();

  // Control motor
  controlMotor(Output);

  // Print IMU data
  Serial.print("Accel X: ");
  Serial.print(ax);
  Serial.print(" Accel Y: ");
  Serial.print(ay);
  Serial.print(" Accel Z: ");
  Serial.println(az);
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
