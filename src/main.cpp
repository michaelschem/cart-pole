#include <Arduino.h>
#include <Encoder.h>
#include <esp32-hal-gpio.h>
#include <math.h>

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
const int steps_per_rev = 400;
const float max_rpm = 800;
const int max_angle = 30;
const float rod_length = 200.0;  // Length of the rod in mm

unsigned long lastPrintTime = 0;

class MotorController {
public:
    MotorController(int pulPin, int dirPin, int enaPin)
        : pulPin(pulPin), dirPin(dirPin), enaPin(enaPin), currentDirection(LEFT), speed_us(5000) {}

    void setup() {
        pinMode(pulPin, OUTPUT);
        pinMode(dirPin, OUTPUT);
        pinMode(enaPin, OUTPUT);
        digitalWrite(enaPin, LOW); // Enable driver
    }

    void setDirection(int dir) {
        if (dir == LEFT) {
            digitalWrite(dirPin, LOW);
        } else {
            digitalWrite(dirPin, HIGH);
        }
        currentDirection = dir;
    }

    void step() {
        if (speed_us > 0) {
            digitalWrite(pulPin, HIGH);
            delayMicroseconds(speed_us / 2);
            digitalWrite(pulPin, LOW);
            delayMicroseconds(speed_us / 2);
        }
    }

    void setSpeed(int speed_us) {
        this->speed_us = speed_us;
    }

    int getCurrentDirection() const {
        return currentDirection;
    }

private:
    int pulPin;
    int dirPin;
    int enaPin;
    int currentDirection;
    int speed_us;
};

class EncoderReader {
public:
    EncoderReader(int stepsPerRev)
        : stepsPerRev(stepsPerRev), encoder(nullptr) {
    }

    ~EncoderReader() {
        delete encoder;
    }

    void setup(int pinA, int pinB) {
        encoder = new Encoder(pinA, pinB);
        encoder->write(0);
    }

    float getAngle() {
        long position = encoder->read();
        float angle = (position % stepsPerRev) * (360.0 / stepsPerRev);
        return angle;
    }

private:
    Encoder* encoder;
    const int stepsPerRev;
};

MotorController motor(PUL_PIN, DIR_PIN, ENA_PIN);
EncoderReader encoder(steps_per_rev);

void setup() {
    Serial.begin(115200);
    delay(1000);

    motor.setup();
    encoder.setup(ENCODER_PIN_A, ENCODER_PIN_B);

    // Debug prints to check pin definitions
    Serial.print("Encoder Pin A: ");
    Serial.println(ENCODER_PIN_A);
    Serial.print("Encoder Pin B: ");
    Serial.println(ENCODER_PIN_B);

    // Install the GPIO ISR service if not already installed
    // if (gpio_install_isr_service(ESP_INTR_FLAG_IRAM) != ESP_OK) {
    //   Serial.println("Failed to install GPIO ISR service");
    // }

    // Check if the encoder object is initialized correctly
    // if (encoder->read() == 0) {
    //   Serial.println("Encoder initialized successfully");
    // } else {
    //   Serial.println("Encoder initialization failed");
    // }

}

void loop() {
    float angle = encoder.getAngle();
    float head_position = rod_length * sin(radians(angle));  // Calculate the head position of the rod
    float speed_rpm;

    if (abs(head_position) > max_angle) {
        speed_rpm = 0;
    } else {
        speed_rpm = max_rpm * (abs(head_position) / max_angle);
    }

    // Convert RPM to microseconds per step
    long speed_us = ( 0.0 < speed_rpm && speed_rpm < max_rpm ) ? (60.0 * 1000000.0 / (speed_rpm * steps_per_rev)) : 0;

    motor.setSpeed(speed_us);

    if (head_position < 0) {
        motor.setDirection(LEFT);
    } else {
        motor.setDirection(RIGHT);
    }

    motor.step();

    // Print direction, speed (RPM), and angle every second
    unsigned long currentTime = millis();
    if (currentTime - lastPrintTime >= 1000) {
        Serial.print("Direction: ");
        Serial.print(motor.getCurrentDirection() == LEFT ? "LEFT" : "RIGHT");
        Serial.print(" | Speed (RPM): ");
        Serial.print(speed_rpm);
        Serial.print(" | Speed (us): ");
        Serial.print(speed_us);
        Serial.print(" | Angle: ");
        Serial.print(angle);
        Serial.print(" | Head Position: ");
        Serial.println(head_position);
        lastPrintTime = currentTime;
    }
}
