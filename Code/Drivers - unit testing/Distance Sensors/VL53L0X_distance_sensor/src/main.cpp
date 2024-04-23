#include <Arduino.h>
#include <math.h>
#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
float MEASUREMENT_ERRORS[30] = {0, 0, 0, 0, 0, -0.174, -0.174, -0.325, -0.541, -0.534, -0.638, -0.974, 0.177, -0.959, -0.966, -0.87, -0.801, -0.473, -0.129, 0.306, 1.5, 1.368, 2.5, 2.638, 2.681, 2.681, 2.681};
int total_measurements = 100;
int counter = 0;

float get_measurement_error(float measurement){
	float error = 0;

	// Determine closest integer
	int closest_int = round(measurement);

	// Determine error corresponding to that integer
	error = MEASUREMENT_ERRORS[closest_int-1];

	// Return the corresponding error
	return error;
}

void setup() {
  Serial.begin(115200);

  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }

  Serial.println("Adafruit VL53L0X test.");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
  // power
  Serial.println(F("VL53L0X API Continuous Ranging example\n\n"));

  // start continuous ranging
  lox.startRangeContinuous();
}

void loop() {
	if (lox.isRangeComplete()) {
		Serial.print("Distance in cm: ");
		float measurement_cm = (float)lox.readRange()/10;
		float error = get_measurement_error(measurement_cm);
		measurement_cm = measurement_cm + error;

		Serial.println(measurement_cm);
		delay(100);
		counter++;
	}
	// if (counter == total_measurements){
	// 	counter = 0;
	// 	Serial.println("****************\n");
	// 	delay(20000);
	// }
}