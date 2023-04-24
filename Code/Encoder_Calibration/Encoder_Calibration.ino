#include <Encoder.h>

const uint8_t motorPinA = 8;
const uint8_t motorPinB = 9;
const uint8_t motorPWMPin = 5;
const uint8_t encoderPinA = 3;
const uint8_t encoderPinB = 2;
Encoder encoder(encoderPinA, encoderPinB);

const int motorSpeedPWM = 225;
unsigned long previousMillis = 0;
long previousEncoderCount = 0;

void setup() {
  Serial.begin(19200);
  pinMode(motorPinA, OUTPUT);
  pinMode(motorPinB, OUTPUT);
  pinMode(motorPWMPin, OUTPUT);

  // Start the motor at the specified PWM value
  digitalWrite(motorPinA, HIGH);
  digitalWrite(motorPinB, LOW);
  analogWrite(motorPWMPin, motorSpeedPWM);
}

void loop() {
  unsigned long currentMillis = millis();
  unsigned long deltaTime = currentMillis - previousMillis;

  if (deltaTime >= 1000) { // Update every second
    long currentEncoderCount = encoder.read();
    long deltaEncoderCount = currentEncoderCount - previousEncoderCount;
    previousEncoderCount = currentEncoderCount;

    // Calculate encoder ticks per second
    double ticksPerSecond = deltaEncoderCount / (deltaTime / 1000.0);

    // Print encoder ticks per second and total encoder ticks
    Serial.print("Ticks per second: ");
    Serial.print(ticksPerSecond);
    Serial.print(" | Total encoder ticks: ");
    Serial.println(currentEncoderCount);

    previousMillis = currentMillis;
  }
}
