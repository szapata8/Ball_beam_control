#include <Arduino.h>
#include <Wire.h>

#define SENSOR_ADDR  (0x80 >> 1) // 7 highest bits
#define SHIFT_ADDR      0x35
#define DISTANCE_ADDR   0x5E

uint8_t distance_raw[2] = {0};
uint8_t shift = 0;
float distance_cm = 0;
char buf[100];

float MEASUREMENT_ERRORS[40] = {3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,2.76,2,2,2,2,2,2,2,2,1.9,1,1,1}; // error at [array_index] cm from the sensor
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
	Wire.begin();
	Wire.beginTransmission(SENSOR_ADDR);
	Wire.write(byte(SHIFT_ADDR));
	Wire.endTransmission();

	Wire.requestFrom(SENSOR_ADDR, 1);
	if (1 <= Wire.available()){
		shift = Wire.read();
	}
	Serial.print("Read shift bit: ");
  	Serial.println(shift, HEX);
}

void loop() {
	// Read basic measurement
	Wire.beginTransmission(SENSOR_ADDR);
	Wire.write(byte(DISTANCE_ADDR));
	Wire.endTransmission();

	Wire.requestFrom(SENSOR_ADDR, 2);
	if (2 <= Wire.available()){
		distance_raw[0] = Wire.read();
		distance_raw[1] = Wire.read();

		// Print distance in cm
		distance_cm = (distance_raw[0] * 16 + distance_raw[1]) / 16 / (int)pow(2, shift);
		
		float error = get_measurement_error(distance_cm);
		distance_cm = distance_cm + error;
		// sprintf(buf, "Distance %u cm", distance_cm);
		// Serial.println(buf);

		Serial.println(distance_cm);
		counter++;
		// if (counter == total_measurements){
		// 	counter = 0;
		// 	Serial.println("****************\n");
		// 	delay(25000);
		// }
	}
  	else {
		Serial.println("Read error");
	}
	delay(100);
}