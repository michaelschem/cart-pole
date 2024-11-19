#include <Arduino.h>
#include <Encoder.h>
#include <esp32-hal-gpio.h> // Include the ESP32 GPIO library

// Define the pins connected to the encoder
#define ENCODER_PIN_A 5
#define ENCODER_PIN_B 19

Encoder* myEnc = nullptr;

void setup() {
  Serial.begin(115200);
  Serial.println("Rotary Encoder Test");

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
  Serial.print("Position: ");
  Serial.print(position);
  Serial.print(" | Angle: ");
  Serial.println(angle);

  delay(1000);
}
