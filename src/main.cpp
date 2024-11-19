#include <Arduino.h>
#include <Encoder.h>
#include <esp32-hal-gpio.h>

// Define the pins connected to the encoder
#define ENCODER_PIN_A 5
#define ENCODER_PIN_B 19

// Define the pins connected to the stepper driver
#define PUL_PIN 15  // Pulse pin (D15 on Mega or A1 on Uno)
#define DIR_PIN 2   // Direction pin (D2)
#define ENA_PIN 4   // Enable pin (D4)

#define LEFT 1
#define RIGHT 2

// Constants
const int steps_per_rev = 400;  // Effective steps per revolution
const float gear_diameter = 17.0;  // Diameter of the gear in mm
const float circumference = 3.14159 * gear_diameter;  // Circumference of the gear in mm

Encoder* myEnc = nullptr;

void set_direction(int dir) {
  if (dir == LEFT) {
    digitalWrite(DIR_PIN, LOW);
    Serial.println("Set direction: LEFT");
  } else {
    digitalWrite(DIR_PIN, HIGH);
    Serial.println("Set direction: RIGHT");
  }
}

int currentDirection = LEFT;
int speed_us = 5000;


void setup() {
    pinMode(PUL_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(ENA_PIN, OUTPUT);

    digitalWrite(ENA_PIN, LOW); // Enable driver

    Serial.begin(115200);
    delay(1000);

  myEnc = new Encoder(ENCODER_PIN_A, ENCODER_PIN_B);

  // Debug prints to check pin definitions
  Serial.print("Encoder Pin A: ");
  Serial.println(ENCODER_PIN_A);
  Serial.print("Encoder Pin B: ");
  Serial.println(ENCODER_PIN_B);

  // Install the GPIO ISR service if not already installed
  if (gpio_install_isr_service(ESP_INTR_FLAG_IRAM) != ESP_OK) {
    Serial.println("Failed to install GPIO ISR service");
  }

  // Check if the encoder object is initialized correctly
  if (myEnc->read() == 0) {
    Serial.println("Encoder initialized successfully");
  } else {
    Serial.println("Encoder initialization failed");
  }
}

void loop() {
  // // Read the position from the encoder
  long position = myEnc->read();
  Serial.println(position);

  // Calculate angle (assuming 400 pulses per revolution)
  const int pulses_per_revolution = 400; 
  float angle = (position % pulses_per_revolution) * (360.0 / pulses_per_revolution);

  // Handle negative angles (optional for full circle display)
  if (angle < 0) angle += 360;

  // Print to serial
  // Serial.print("Position: ");
  // Serial.print(position);
  // Serial.print(" | Angle: ");
  // Serial.println(angle);
  // delay(100);

  float distanceFrom90 = abs(angle - 90.0);
  if (distanceFrom90 > 30.0) {
    speed_us = 0;  // Stop the motor
  } else {
    speed_us = 5000 * (30.0 - distanceFrom90) / 30.0;  // Proportional speed
  }

  if (angle > 90.0) {
    currentDirection = LEFT;
  } else {
    currentDirection = RIGHT;
  }

  if (speed_us > 0) {
    set_direction(currentDirection);

    digitalWrite(PUL_PIN, HIGH);
    delayMicroseconds(speed_us / 2);
    digitalWrite(PUL_PIN, LOW);
    delayMicroseconds(speed_us / 2);
  }
}
