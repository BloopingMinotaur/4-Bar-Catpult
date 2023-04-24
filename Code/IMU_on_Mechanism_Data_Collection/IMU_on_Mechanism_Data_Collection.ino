#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <PID_v1_bc.h>
#include <Encoder.h>

// MPU-6050 setup
Adafruit_MPU6050 mpu;

// Motor and encoder setup
const uint8_t motorPinA = 8;
const uint8_t motorPinB = 9;
const uint8_t encoderPinA = 3;
const uint8_t encoderPinB = 2;
const uint8_t motorPWMPin = 5;
Encoder encoder(encoderPinA, encoderPinB);

// PID setup for constant motor speed
const int desiredRPM = 400;
const int countsPerRevolution = 950;
double Setpoint, Input, Output;
double Kp = 0.5, Ki = 5, Kd = 0.01;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

unsigned long previousMillis = 0;
long previousEncoderCount = 0;
uint16_t revolutions = 0;

const int maxDataPoints = 50;
const uint8_t maxRevolutions = 20;
const uint8_t samplesPerRev = maxDataPoints/maxRevolutions;
const int samplesPerStep = countsPerRevolution / samplesPerRev;
int revolutionCounter = 0;
long datapreviousEncoderCount =0;
int dataIndex = 0;
bool recording = true;
unsigned long dataTimes[maxDataPoints];
double accelDataX[maxDataPoints];
double accelDataY[maxDataPoints];
double accelDataZ[maxDataPoints];


void setup() {
  Serial.begin(19200);
  Wire.begin();

  // Initialize MPU-6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Configure PID
  Setpoint = desiredRPM;
  Serial.print(Setpoint);
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(100);
  myPID.SetOutputLimits(0, 255);

  // Motor pins
  pinMode(motorPinA, OUTPUT);
  pinMode(motorPinB, OUTPUT);
  pinMode(motorPWMPin, OUTPUT);
}

void loop() {
  Serial.println();
  unsigned long currentMillis = millis();
  unsigned long deltaTime = currentMillis - previousMillis;

  // Read accelerometer and gyroscope data
  sensors_event_t a_event, g_event, temp_event;
  mpu.getEvent(&a_event, &g_event, &temp_event);

  // Convert raw values to Gs
  float accel_x = a_event.acceleration.x / 9.81;
  float accel_y = a_event.acceleration.y / 9.81;
  float accel_z = a_event.acceleration.z / 9.81;

  // Print accelerometer values in Gs
  Serial.print("Accel X: ");
  Serial.print(accel_x);
  Serial.print(" Accel Y: ");
  Serial.print(accel_y);
  Serial.print(" Accel Z: ");
  Serial.println(accel_z);

  // Calculate and print the magnitude of acceleration
  float magnitude = sqrt(pow(accel_x, 2) + pow(accel_y, 2) + pow(accel_z, 2));
  Serial.print("Magnitude: ");
  Serial.println(magnitude);

  if (deltaTime >= 100) {
    // Read encoder value and calculate counts per second
    long currentEncoderCount = encoder.read();
    long deltaEncoderCount = currentEncoderCount - previousEncoderCount;
    previousEncoderCount = currentEncoderCount;
    double countsPerSecond = (deltaEncoderCount / (double)deltaTime) * 1000;

    // Calculate current RPM
    Input = (countsPerSecond / countsPerRevolution) * 60;
    Serial.print("Input: ");
    Serial.println(Input);

    // Update revolution counter
    revolutions = currentEncoderCount / countsPerRevolution;
    Serial.print("Revolutions: ");
    Serial.println(revolutions);

    // Record acceleration data
    if (recording && dataIndex < maxDataPoints) {
      int deltaEncoderCount = currentEncoderCount - datapreviousEncoderCount;
      if (deltaEncoderCount >= samplesPerStep) {
        datapreviousEncoderCount = currentEncoderCount;
        dataTimes[dataIndex] = millis();
        accelDataX[dataIndex] = accel_x;
        accelDataY[dataIndex] = accel_y;
        accelDataZ[dataIndex] = accel_z;
        dataIndex++;
      }
    } else if (recording && revolutions >= maxRevolutions) {
      recording = false;
      endRecording();
    }

    // Compute PID
    myPID.Compute();

    // Control motor
    controlMotor(Output);
    Serial.print("PID OUTPUT (PWM): ");
    Serial.println(Output);

    // Print current encoder speed, encoder value, and RPM
    Serial.print("Encoder Speed (counts/s): ");
    Serial.print(countsPerSecond);
    Serial.print(" | Encoder Value: ");
    Serial.print(currentEncoderCount);
    Serial.print(" | RPM: ");
    Serial.println(Input);

    // Update revolution counter
    revolutions = currentEncoderCount / countsPerRevolution;
    Serial.print("Revolutions: ");
    Serial.println(revolutions);
    previousMillis = currentMillis;
  }
}

void controlMotor(int speed) {
  if (speed > 0) {
    digitalWrite(motorPinA, HIGH);
    digitalWrite(motorPinB, LOW);
    analogWrite(motorPWMPin, speed);
  } else {
    digitalWrite(motorPinA, LOW);
    digitalWrite(motorPinB, LOW);
    analogWrite(motorPWMPin, 0);
  }
}

void endRecording() {
  // Print the recorded data
  controlMotor(0);
  Serial.println("Recorded Data:");
  for (int i = 0; i < maxDataPoints; i++) {
    //Serial.print(" Time: ");
    Serial.print(" ");
    Serial.print(dataTimes[i]);
  }
  Serial.println(" ");
  for (int i = 0; i < maxDataPoints; i++) {
    //Serial.print(" Accel X: ");
    Serial.print(" ");
    Serial.print(accelDataX[i]);
  }
  Serial.println(" ");
  for (int i = 0; i < maxDataPoints; i++) {
    //Serial.print(" Accel Y: ");
    Serial.print(" ");
    Serial.print(accelDataY[i]);
  }
  Serial.println(" ");
  for (int i = 0; i < maxDataPoints; i++) {
    //Serial.print(" Accel Y: ");
    Serial.print(" ");
    Serial.print(accelDataZ[i]);
  }
  Serial.println(" ");

  // Reset the data arrays and counters
  memset(dataTimes, 0, sizeof(dataTimes));
  memset(accelDataX, 0, sizeof(accelDataX));
  memset(accelDataY, 0, sizeof(accelDataY));
}
