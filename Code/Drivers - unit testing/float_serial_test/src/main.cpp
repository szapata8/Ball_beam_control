#include <Arduino.h>

void setup() {
  Serial.begin(9600);  // Start serial communication at 9600 baud rate
  while (!Serial) {
    ; // Wait for the serial port to connect, needed for some boards like the Leonardo
  }
}

void loop() {
  if (Serial.available() >= sizeof(float)) {  // Check if there's at least a float's worth of data
    float receivedValue;
    Serial.readBytes((char *)&receivedValue, sizeof(float));  // Read the float value
    Serial.write((char *)&receivedValue, sizeof(float));  // Echo the same float back
  }
}