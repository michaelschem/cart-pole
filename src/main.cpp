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
const int max_rpm = 20;

Encoder* myEnc = nullptr;

void set_direction(int dir) {
  if (dir == LEFT) {
    digitalWrite(DIR_PIN, LOW);
    // Serial.println("Set direction: LEFT");
  } else {
    digitalWrite(DIR_PIN, HIGH);
    // Serial.println("Set direction: RIGHT");
  }
}

int currentDirection = LEFT;
int speed_us = 5000;

unsigned long lastPrintTime = 0;

const int num_readings = 10;
float angle_readings[num_readings];
int read_index = 0;
float total = 0;
float average_angle = 0;

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

  // Initialize all the readings to 0
  for (int i = 0; i < num_readings; i++) {
    angle_readings[i] = 0;
  }
}

void loop() {
  // // Read the position from the encoder
  long position = myEnc->read();
  // Serial.println(position);

  // Calculate angle (assuming 400 pulses per revolution)
  const int pulses_per_revolution = 400; 
  float angle = (position % pulses_per_revolution) * (360.0 / pulses_per_revolution);

  // Handle negative angles (optional for full circle display)
  if (angle < 0) angle += 360;

  // Update the total by subtracting the oldest reading and adding the new one
  total = total - angle_readings[read_index];
  angle_readings[read_index] = angle;
  total = total + angle_readings[read_index];

  // Advance to the next position in the array
  read_index = (read_index + 1) % num_readings;

  // Calculate the average angle
  average_angle = total / num_readings;

  // Determine direction
  const char* directionStr = (currentDirection == LEFT) ? "LEFT" : "RIGHT";

  // Calculate speed in RPM
  float speed_rpm = (speed_us > 0) ? (60.0 * 1000000.0 / (speed_us * steps_per_rev)) : 0;

  // Print direction, speed (RPM), and angle every second
  // unsigned long currentTime = millis();
  // if (currentTime - lastPrintTime >= 1000) {
  //   Serial.print("Direction: ");
  //   Serial.print(directionStr);
  //   Serial.print(" | Speed (RPM): ");
  //   Serial.print(speed_rpm);
  //   Serial.print(" | Angle: ");
  //   Serial.println(average_angle);
  //   lastPrintTime = currentTime;
  // }

  float distanceFrom90 = abs(average_angle - 90.0);
  if (distanceFrom90 > 40.0) {
    speed_us = 0;  // Stop the motor
  } else {
    // Calculate speed_us for the maximum speed defined by max_rpm
    float max_speed_us = 60.0 * 1000000.0 / (max_rpm * steps_per_rev);
    speed_us = max_speed_us * (40.0 - distanceFrom90) / 40.0;
  }

  if (average_angle < 90.0) {
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
