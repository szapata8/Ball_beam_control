#include <Arduino.h>
#include <DRV8834.h>
#include <Wire.h>
#include <CircularBuffer.hpp>
#include <Adafruit_VL53L0X.h>
#include <RunningAverage.h>

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 			200
#define MICROSTEPS_PER_STEP		32
#define RPM 					120
#define DIR 					27
#define STEP 					14
#define M0 						17
#define M1 						16

DRV8834 stepper(MOTOR_STEPS, DIR, STEP, M0, M1);

// =============================== DISTANCE SENSOR VARIABLES
#define PLATFORM_LENGTH 50
#define SENSOR_ADDR  (0x80 >> 1) // 7 highest bits
#define SHIFT_ADDR      0x35
#define DISTANCE_ADDR   0x5E

#define MAIN_SENSOR_SWAP_THRESHOLD 35
#define RUNNING_AVERAGE_BUFFER_SIZE 30
// Variables sensor 1			 {1-----------------10------------------20------------------30-----------}
float MEASUREMENT_ERRORS_1[40] = {3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,2,2,2,2,2,2,2,1.5,1,1,1}; // error at [array_index] cm from the sensor
uint8_t distance_raw[2] = {0};
uint8_t shift = 0;
float distance_cm_sensor1 = 0;
char buf[100];
RunningAverage runningAverage_sensor1(RUNNING_AVERAGE_BUFFER_SIZE);

// Variables sensor 2
float MEASUREMENT_ERRORS_2[30] = {0, 0, 0, 0, 0, -0.174, -0.174, -0.325, -0.541, -0.534, -0.638, -0.974, 0.177, -0.959, -0.966, -0.87, -0.801, -0.473, -0.129, 0.306, 1.5, 1.368, 2.5, 2.638, 2.681, 2.681, 2.681}; // error at [array_index] cm from the sensor
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
float distance_cm_sensor2 = 0;
RunningAverage runningAverage_sensor2(RUNNING_AVERAGE_BUFFER_SIZE);

//---------------------------------------------------------------------------
float ball_position = 0;
float target_position = 25.0;
float error = 0;
float prev_error = 0;
float kp = 3;
float ki = 0.1;
float kd = 40;
float derivative = 0;
CircularBuffer<float, 10> integrator;

#define MAX_STEPS 40
#define MIN_STEPS -40

// ----------------- DISTANCE SENSOR FUNCTIONS
float get_measurement_error(int sensor_number, float measurement){
	float error = 0;
	int closest_int = round(measurement); // Determine closest integer	
	if (sensor_number == 1){
		error = MEASUREMENT_ERRORS_1[closest_int-1]; // Determine error corresponding to that integer
	}
	else {
		error = MEASUREMENT_ERRORS_2[closest_int-1]; // Determine error corresponding to that integer
	}
	return error;
}

void send_distance(){
	// Send current magnetic encoder value (total angle)
	byte floatBytes[sizeof(float)]; // Create a byte array to hold the bytes of the float

	// Copy the bytes of the float into the byte array
	memcpy(floatBytes, &ball_position, sizeof(float));
	
	// Send each byte over the serial connection
	for (int i = 0; i < sizeof(float); i++) {
		Serial.write(floatBytes[i]);
	}
}

void read_distance_sensor1(){
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
		float error = get_measurement_error(1, distance_cm_sensor1);
		distance_cm_sensor1 = distance_cm_sensor1 + error;
		// sprintf(buf, "Distance %u cm", distance_cm_sensor1);
		// Serial.println(buf);
	}
  	else {
		// Serial.println("Read error sensor 1");
		distance_cm_sensor1 = 50;
	}
	runningAverage_sensor1.addValue(distance_cm_sensor1);
}

void read_distance_sensor2(){
	if (lox.isRangeComplete()) {
		uint16_t distance_mm = lox.readRange();
		distance_cm_sensor2 = distance_mm/10;
		float error = get_measurement_error(2, distance_cm_sensor2);
		distance_cm_sensor2 = distance_cm_sensor2 + error;
		runningAverage_sensor2.addValue(distance_cm_sensor2);
	}
}

void read_ball_position(){
	// Read sensor 1
	read_distance_sensor1();
	// Read sensor 2
	read_distance_sensor2();

	float average_sensor1 = runningAverage_sensor1.getAverage();
	float average_sensor2 = runningAverage_sensor2.getAverage();
	if ( average_sensor1 < MAIN_SENSOR_SWAP_THRESHOLD){
		ball_position = average_sensor1;
	}
	else {
		ball_position = PLATFORM_LENGTH - average_sensor2;
	}

	// DEBUGGING FOR DISTANCE MEASUREMENT
	Serial.print("\n\n\t\t\t**** ball position: ");
	Serial.print(ball_position);
	// Serial.print(" // #1: ");
	// Serial.print(average_sensor1);
	// Serial.print(" - #2: ");
	// Serial.print(average_sensor2);
	Serial.println();
}

float buffer_summation(const CircularBuffer<float, 10>& buffer) {
  float sum = 0;  // Variable to hold the sum
  // Loop through all elements in the buffer and add them to the sum
  for (int i = 0; i < buffer.size(); i++) {
    sum += buffer[i];
  }
  return sum;
}

void setup() {
    /*
     * Set target motor RPM.
     */
	Serial.begin(115200); 
    Serial.println("Starting setup...");

	// =========== STEPPER MOTOR SETUP ===========
    stepper.begin(RPM, MICROSTEPS_PER_STEP);
    stepper.enable();

	// =========== DISTANCE SENSOR SETUP ===========
	Wire.begin(); 
	Wire.setClock(800000L); 
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
    // Serial.println("============ point 1");
    // Read current posititon
    read_ball_position();

    // Serial.println("============ point 2");
	error = target_position - ball_position;

	Serial.print("error: ");
	Serial.println(error);

	integrator.push(error);
    derivative = error - prev_error;
    prev_error = error;
    int steps = kp*error + ki*buffer_summation(integrator) + kd*derivative;

    if (steps > MAX_STEPS) { steps = MAX_STEPS;}
    
    if (steps < MIN_STEPS) { steps = MIN_STEPS;}

    Serial.print("\t\tsteps: ");
    Serial.println(steps);

    stepper.move(steps);
    // Serial.println("============ point 3");

    delay(20);
    // Serial.println("============ point 4");
}