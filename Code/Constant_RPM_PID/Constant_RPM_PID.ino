#include <PID_v1_bc.h>
#include <Encoder.h>

// Motor and encoder setup
const int motorPinA = 8;
const int motorPinB = 9;
const int encoderPinA = 3;
const int encoderPinB = 2;
Encoder encoder(encoderPinA, encoderPinB);

// PID setup for constant motor speed
const double desiredRPM = 400;
const double countsPerRevolution = 900;
double Setpoint, Input, Output;
double Kp = 0.5, Ki = 5, Kd = 0.01;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

unsigned long previousMillis = 0;
long previousEncoderCount = 0;
int revolutions = 0;
bool motorRunning = true;
unsigned long pauseStartTime = 0;
const unsigned long pauseDuration = 1000; // Pause duration in milliseconds

void setup() {
  Serial.begin(19200);

  // Configure PID
  Setpoint = desiredRPM;
  Serial.print(Setpoint);
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(100);
  myPID.SetOutputLimits(0, 255);

  // Motor pins
  pinMode(motorPinA, OUTPUT);
  pinMode(motorPinB, OUTPUT);
}

void loop() {
  Serial.println();
  unsigned long currentMillis = millis();
  unsigned long deltaTime = currentMillis - previousMillis;

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

    // Check if motor is paused
    if (!motorRunning && (currentMillis - pauseStartTime) >= pauseDuration) {
      motorRunning = true;
      encoder.write(0); // Reset the encoder count
      previousEncoderCount = 0;
    }

    // Compute PID if motor is running
    if (motorRunning) {
      myPID.Compute();
    } else {
      Output = 0;
    }

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

    if (revolutions >= 20 && motorRunning) {
      motorRunning = false;
      pauseStartTime = currentMillis;
    }

    previousMillis = currentMillis;
  }
}

void controlMotor(int speed) {
  if (speed > 0) {
    digitalWrite(motorPinA, HIGH);
    digitalWrite(motorPinB, LOW);
    analogWrite(5, speed);
  } else {
    digitalWrite(motorPinA, LOW);
    digitalWrite(motorPinB, LOW);
    analogWrite(5, 0);
  }
}

