#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <RunningAverage.h>

#define PLATFORM_LENGTH 50
#define SENSOR_ADDR  (0x80 >> 1) // 7 highest bits
#define SHIFT_ADDR      0x35
#define DISTANCE_ADDR   0x5E

#define SENSOR1_CALIBRATION 3
#define SENSOR2_CALIBRATION 0
#define RUNNING_AVERAGE_BUFFER_SIZE 30

// Variables sensor 1
uint8_t distance_raw[2] = {0};
uint8_t shift = 0;
uint8_t distance_cm_sensor1 = 0;
char buf[100];
RunningAverage runningAverage_sensor1(RUNNING_AVERAGE_BUFFER_SIZE);

// Variables sensor 2
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
uint16_t distance_cm_sensor2 = 0;
RunningAverage runningAverage_sensor2(RUNNING_AVERAGE_BUFFER_SIZE);

float ball_position = 0;

void read_sensor1(){
	// Read basic measurement
	Wire.beginTransmission(SENSOR_ADDR);
	Wire.write(byte(DISTANCE_ADDR));
	Wire.endTransmission();

	Wire.requestFrom(SENSOR_ADDR, 2);
	if (2 <= Wire.available()){
		distance_raw[0] = Wire.read();
		distance_raw[1] = Wire.read();

		// Print distance in cm
		distance_cm_sensor1 = (distance_raw[0] * 16 + distance_raw[1]) / 16 / (int)pow(2, shift);
		distance_cm_sensor1 = distance_cm_sensor1 + SENSOR1_CALIBRATION;
		// sprintf(buf, "Distance %u cm", distance_cm_sensor1);
		// Serial.println(buf);
	}
  else {
		Serial.println("Read error sensor 1");
		distance_cm_sensor1 = 50;
	}
	runningAverage_sensor1.addValue(distance_cm_sensor1);
}

void read_sensor2(){
	if (lox.isRangeComplete()) {
		uint16_t distance_mm = lox.readRange();
		distance_cm_sensor2 = distance_mm/10;
		distance_cm_sensor2 = distance_cm_sensor2 + SENSOR2_CALIBRATION;
		runningAverage_sensor2.addValue(distance_cm_sensor2);
	}
}

void read_ball_position(){
	// Read sensor 1
	read_sensor1();
	// Read sensor 2
	read_sensor2();

	float average_sensor1 = runningAverage_sensor1.getAverage();
	float average_sensor2 = runningAverage_sensor2.getAverage();
	if ( average_sensor1 < average_sensor2){
		ball_position = average_sensor1;
	}
	else {
		ball_position = PLATFORM_LENGTH - average_sensor2;
	}
	Serial.print("**** ball position: ");
	Serial.print(ball_position);
	Serial.print(" // #1: ");
	Serial.print(average_sensor1);
	Serial.print(" - #2: ");
	Serial.println(average_sensor2);
}

void setup() {
  	Serial.begin(115200);
	// wait until serial port opens for native USB devices
	while (! Serial) {
		delay(1);
	}
	Wire.begin();

	// Setup sensor 1
	runningAverage_sensor1.clear();
	Wire.beginTransmission(SENSOR_ADDR);
	Wire.write(byte(SHIFT_ADDR));
	Wire.endTransmission();

	Wire.requestFrom(SENSOR_ADDR, 1);
	if (1 <= Wire.available())
	{
		shift = Wire.read();
	}
	Serial.print("Read shift bit: ");
  	Serial.println(shift, HEX);

	// Setup sensor 2
	runningAverage_sensor2.clear();
	Serial.println("Adafruit VL53L0X test.");
	if (!lox.begin()) {
		Serial.println(F("Failed to boot VL53L0X"));
		while(1);
	}
	// start continuous ranging
  	lox.startRangeContinuous();
}

void loop() {
	read_ball_position();
	// delay(100);
}