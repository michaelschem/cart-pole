#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#define PUL_PIN 15  // Pulse pin (D15 on Mega or A1 on Uno)
#define DIR_PIN 2   // Direction pin (D2)
#define ENA_PIN 4   // Enable pin (D4)

#define LEFT 1
#define RIGHT 2

char* ssid = "Woodsy";
char* password = "unit103!";

// Constants
const int steps_per_rev = 400;  // Effective steps per revolution
const float gear_diameter = 17.0;  // Diameter of the gear in mm
const float circumference = 3.14159 * gear_diameter;  // Circumference of the gear in mm

WiFiUDP udp;
unsigned int localUdpPort = 8888;  // Local port to listen on
char incomingPacket[255];  // Buffer for incoming packets

void setup() {
  pinMode(PUL_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);

  digitalWrite(ENA_PIN, LOW); // Enable driver

  Serial.begin(115200);
  delay(1000);
  Serial.println("Connecting to WiFi");
  WiFi.disconnect(true);

  IPAddress local_IP(192, 168, 1, 21);
  IPAddress gateway(192, 168, 1, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.config(local_IP, gateway, subnet);

  // Connect to WiFi
  WiFi.begin(ssid, password);
  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 30000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected.");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.print("Subnet Mask: ");
    Serial.println(WiFi.subnetMask());
    Serial.print("Gateway IP: ");
    Serial.println(WiFi.gatewayIP());
  } else {
    Serial.println("\nFailed to connect to WiFi. Restarting...");
    ESP.restart();
  }

  udp.begin(localUdpPort);
  Serial.printf("Now listening for UDP packets on port %d\n", localUdpPort);
}

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

void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(incomingPacket, 255);
    if (len > 0) {
      incomingPacket[len] = 0;  // Null-terminate the string
    }
    float receivedNumber = atof(incomingPacket);
    Serial.printf("Received UDP packet: %f\n", receivedNumber);

    if (receivedNumber > 90.0) {
      currentDirection = LEFT;
    } else {
      currentDirection = RIGHT;
    }
  }

  set_direction(currentDirection);

  digitalWrite(PUL_PIN, HIGH);
  delayMicroseconds(speed_us / 2);
  digitalWrite(PUL_PIN, LOW);
  delayMicroseconds(speed_us / 2);
}
