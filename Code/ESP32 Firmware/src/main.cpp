#include <Arduino.h>
#include <DRV8834.h>
#include <Wire.h>
#include <CircularBuffer.hpp>

// =============================== STEPPER MOTOR VARIABLES
#define MOTOR_STEPS 			200
#define MICROSTEPS_PER_STEP		32
#define RPM 					120
#define DIR 					27
#define STEP 					14
#define M0 						17
#define M1 						16

DRV8834 stepper(MOTOR_STEPS, DIR, STEP, M0, M1);

// =============================== MAGNETIC ENCODER VARIABLES
int magnetStatus = 0; 						//value of the status register (MD, ML, MH)
int lowbyte; 								//raw angle 7:0
word highbyte; 								//raw angle 7:0 and 11:8
int rawAngle; 								//final raw angle 
float degAngle; 							//raw angle in degrees (360/4096 * [value between 0-4095])
int quadrantNumber, previousquadrantNumber;	//quadrant IDs
float numberofTurns = 0; 					//number of turns
float correctedAngle = 0; 					//tared angle - based on the startup value
float startAngle = 0; 						//starting angle
float totalAngle = 0; 						//total absolute angular displacement
float previoustotalAngle = 0; 				//for the display printing

float real_zero = 0; 						// value where the stage is balanced


// =============================== SYSTEM OPERATION VARIABLES
#define PID_MODALITY 				'P'
#define MANUAL_CONTROL_MODALITY 	'M'
#define CHANGE_MODALITY_COMMAND 	'@'

char modality;
bool first_pass_modality_change;

// =============================== MANUAL CONTROL OPERATION VARIABLES
#define STEP_CW				'S'
#define STEP_CCW			's'
#define MICROSTEP_CW 		'U'
#define MICROSTEP_CCW 		'u'
#define READ_ANGLE	 		'r'
#define ZERO_REF	 		'z'

// =============================== PID OPERATION VARIABLES
#define SET_KP					'p'
#define SET_KI					'i'
#define SET_KD 					'd'
#define SET_TARGET_ANGLE		'a'
// ------------ testing variables (no serial comms)
// int counter = 0;
// int target_angle[10] = {50, -5, 10, -40, -11, 30, 7, 150, 80, 2}; // no stage
// int target_angle[10] = {-20, -5, 10, -25, 15, 28, 7, -10, 22, 2}; // mounted stage
// float desired_angle = target_angle[counter];

// Good values for mounted platform control
// float kp = 0.1;
// float ki = 0.5;
// float kd = 0;

int loop_delay = 20;
float kp = 0.1;
float ki = 0.5;
float kd = 0;
float desired_angle = 0.0;
float error = 0.0;
float prev_error = 0.0;
float derivative = 0.0;
CircularBuffer<float, 5> integrator;

// ----------------- MAGNETIC ENCODER FUNCTIONS
float buffer_summation(const CircularBuffer<float, 5>& buffer) {
  float sum = 0;  // Variable to hold the sum
  // Loop through all elements in the buffer and add them to the sum
  for (int i = 0; i < buffer.size(); i++) {
    sum += buffer[i];
  }
  return sum;
}

void readRawAngle(){ 
	//7:0 - bits
	Wire.beginTransmission(0x36); //connect to the sensor
	Wire.write(0x0D); //figure 21 - register map: Raw angle (7:0)
	Wire.endTransmission(); //end transmission
	Wire.requestFrom(0x36, 1); //request from the sensor

	while(Wire.available() == 0); //wait until it becomes available 
	lowbyte = Wire.read(); //Reading the data after the request

	//11:8 - 4 bits
	Wire.beginTransmission(0x36);
	Wire.write(0x0C); //figure 21 - register map: Raw angle (11:8)
	Wire.endTransmission();
	Wire.requestFrom(0x36, 1);

	while(Wire.available() == 0);  
	highbyte = Wire.read();

	//4 bits have to be shifted to its proper place as we want to build a 12-bit number
	highbyte = highbyte << 8; //shifting to left
	//What is happening here is the following: The variable is being shifted by 8 bits to the left:
	//Initial value: 00000000|00001111 (word = 16 bits or 2 bytes)
	//Left shifting by eight bits: 00001111|00000000 so, the high byte is filled in

	//Finally, we combine (bitwise OR) the two numbers:
	//High: 00001111|00000000
	//Low:  00000000|00001111
	//      -----------------
	//H|L:  00001111|00001111
	rawAngle = highbyte | lowbyte; //int is 16 bits (as well as the word)

	//We need to calculate the angle:
	//12 bit -> 4096 different levels: 360° is divided into 4096 equal parts:
	//360/4096 = 0.087890625
	//Multiply the output of the encoder with 0.087890625
	degAngle = rawAngle * 0.087890625; 

	// Serial.print("\n====================\n");
	// Serial.print("Deg angle: ");
	// Serial.println(degAngle, 2); //absolute position of the encoder within the 0-360 circle

}

void correctAngle(){
	//recalculate angle
	correctedAngle = degAngle - startAngle; //this tares the position

	if(correctedAngle < 0) //if the calculated angle is negative, we need to "normalize" it
	{
	correctedAngle = correctedAngle + 360; //correction for negative numbers (i.e. -15 becomes +345)
	}
	else
	{
	//do nothing
	}
	// Serial.print("Corrected angle: ");
	// Serial.println(correctedAngle, 2); //print the corrected/tared angle  
}

void checkQuadrant(){
	/*
	//Quadrants:
	4  |  1
	---|---
	3  |  2
	*/

	//Quadrant 1
	if (correctedAngle >= 0 && correctedAngle <=90){
		quadrantNumber = 1;
	}

	//Quadrant 2
	if (correctedAngle > 90 && correctedAngle <=180){
		quadrantNumber = 2;
	}

	//Quadrant 3
	if (correctedAngle > 180 && correctedAngle <=270){
		quadrantNumber = 3;
	}

	//Quadrant 4
	if (correctedAngle > 270 && correctedAngle <360){
		quadrantNumber = 4;
	}
	//Serial.print("Quadrant: ");
	//Serial.println(quadrantNumber); //print our position "quadrant-wise"

	if (quadrantNumber != previousquadrantNumber){ //if we changed quadrant
		if(quadrantNumber == 1 && previousquadrantNumber == 4){
			numberofTurns++; // 4 --> 1 transition: CW rotation
		}

		if(quadrantNumber == 4 && previousquadrantNumber == 1){
			numberofTurns--; // 1 --> 4 transition: CCW rotation
		}
		//this could be done between every quadrants so one can count every 1/4th of transition

		previousquadrantNumber = quadrantNumber;  //update to the current quadrant
	}  

	//Serial.print("Turns: ");
	//Serial.println(numberofTurns,0); //number of turns in absolute terms (can be negative which indicates CCW turns)  

	//after we have the corrected angle and the turns, we can calculate the total absolute position
	totalAngle = (numberofTurns*360) + correctedAngle; //number of turns (+/-) plus the actual angle within the 0-360 range
	totalAngle = totalAngle - real_zero; // final angle adjustment to account for user-defined 0 reference
    //Serial.print("Total angle: ");
	//Serial.println(totalAngle, 2); //absolute position of the motor expressed in degree angles, 2 digits
}

void checkMagnetPresence(){  
  //This function runs in the setup() and it locks the MCU until the magnet is not positioned properly

  while((magnetStatus & 32) != 32) //while the magnet is not adjusted to the proper distance - 32: MD = 1
  {
    magnetStatus = 0; //reset reading

    Wire.beginTransmission(0x36); //connect to the sensor
    Wire.write(0x0B); //figure 21 - register map: Status: MD ML MH
    Wire.endTransmission(); //end transmission
    Wire.requestFrom(0x36, 1); //request from the sensor

    while(Wire.available() == 0); //wait until it becomes available 
    magnetStatus = Wire.read(); //Reading the data after the request

    //Serial.print("Magnet status: ");
    //Serial.println(magnetStatus, BIN); //print it in binary so you can compare it to the table (fig 21)      
  }      
  
  //Status register output: 0 0 MD ML MH 0 0 0  
  //MH: Too strong magnet - 100111 - DEC: 39 
  //ML: Too weak magnet - 10111 - DEC: 23     
  //MD: OK magnet - 110111 - DEC: 55

  //Serial.println("Magnet found!");
  //delay(1000);  
}

void magneticEncoderMonitoring(){    
    readRawAngle(); //ask the value from the sensor
    correctAngle(); //tare the value
    checkQuadrant(); //check quadrant, check rotations, check absolute angular position
}

// ----------------- SERIAL COMMUNICATION FUNCTIONS
void send_current_angle(){
	// Send current magnetic encoder value (total angle)
	byte floatBytes[sizeof(float)]; // Create a byte array to hold the bytes of the float

	// Copy the bytes of the float into the byte array
	memcpy(floatBytes, &totalAngle, sizeof(float));
	
	// Send each byte over the serial connection
	for (int i = 0; i < sizeof(float); i++) {
		Serial.write(floatBytes[i]);
	}
}

void send_real_zero(){
	// Send current magnetic encoder value (total angle)
	byte floatBytes[sizeof(float)]; // Create a byte array to hold the bytes of the float

	// Copy the bytes of the float into the byte array
	memcpy(floatBytes, &real_zero, sizeof(float));
	
	// Send each byte over the serial connection
	for (int i = 0; i < sizeof(float); i++) {
		Serial.write(floatBytes[i]);
	}
}

void send_float(float float_value) {
	Serial.write((char *)&float_value, sizeof(float));
}

void setup() {
    // Initialize serial communication
	Serial.begin(115200); 

	// Initialize stepper
    stepper.begin(RPM, MICROSTEPS_PER_STEP);
    stepper.enable();

    // SIinitialize i2c bus communication 
	Wire.begin(); 
	Wire.setClock(800000L);

	// Initialize magnetic encoder
	checkMagnetPresence(); 	//check the magnet (blocks until magnet is found)
	readRawAngle(); 		//make a reading so the degAngle gets updated
	startAngle = degAngle; 	//update startAngle with degAngle - for taring

	// =========== SYSTEM OPERATION MODALITY SETUP ===========
	modality = MANUAL_CONTROL_MODALITY;
	first_pass_modality_change = false;
}

void loop(){

	// ============================ UPDATE SYSTEM STATES 
	magneticEncoderMonitoring();
	// Serial.print("totalAngle: ");
	// Serial.println(totalAngle);


	// ============================ CHECK FOR MODALITY CHANGE
	int available_bytes = Serial.available();
	char serial_command = '0'; // init value at every iteration - if it remains '0' after the first if statement, it means no serial command was received
	if (available_bytes){
		serial_command = Serial.read();
		if (serial_command == CHANGE_MODALITY_COMMAND){
			// Read next byte and set new modality
			modality = Serial.read();
			Serial.write(modality); // echo statement

			first_pass_modality_change = true;
		}
	}

	// ============================ MANUAL CONTROL OPERATION
	if (modality == MANUAL_CONTROL_MODALITY && !first_pass_modality_change){
		if (serial_command == STEP_CW){
			stepper.move(MICROSTEPS_PER_STEP);
		}
		else if (serial_command == STEP_CCW){
			stepper.move(-1*MICROSTEPS_PER_STEP);
		}
		else if (serial_command == MICROSTEP_CW){
			stepper.move(1);
		}
		else if (serial_command == MICROSTEP_CCW){
			stepper.move(-1);
		}
		else if (serial_command == READ_ANGLE){
			send_current_angle();
		}
		else if (serial_command == ZERO_REF){
			real_zero = totalAngle;
			send_real_zero();
		}
	}

	// ============================ PID CONTROL OPERATION
	if (modality == PID_MODALITY && !first_pass_modality_change){
		// Run PID algorithm
		if (serial_command == '0'){ // First case evaluated to skip all the other else if evaluations and get better timing on the PID execution
			error = desired_angle - totalAngle;
			integrator.push(error);
			// Serial.print("error: ");
			// Serial.println(error);

			derivative = error - prev_error;
			prev_error = error;
			int steps = kp*error + ki*buffer_summation(integrator) + kd*derivative;

			// Send new input
			stepper.move(steps);
			delay(loop_delay);
			
		}

		else if (serial_command == SET_KP){
			if (Serial.available() >= sizeof(float)) { // Check if enough bytes available
				byte buffer[sizeof(float)];
				Serial.readBytes(buffer, sizeof(float));
				memcpy(&kp, buffer, sizeof(float));
				// float receivedValue;
				// Serial.readBytes((char *)&receivedValue, sizeof(float));  // Read the float value
				// Serial.write((char *)&receivedValue, sizeof(float));  // Echo the same float back

				// -------------- Echo statement for feedback

				// Serial.write('#');
				send_float(kp);
			}
		}
		
		else if (serial_command == SET_KI){
			if (Serial.available() >= sizeof(float)) { // Check if enough bytes available
				byte buffer[sizeof(float)];
				Serial.readBytes(buffer, sizeof(float));
				memcpy(&ki, buffer, sizeof(float));

				// -------------- Echo statement for feedback
				send_float(ki);
			}
		}
		
		else if (serial_command == SET_KD){
			if (Serial.available() >= sizeof(float)) { // Check if enough bytes available
				byte buffer[sizeof(float)];
				Serial.readBytes(buffer, sizeof(float));
				memcpy(&kd, buffer, sizeof(float));

				// -------------- Echo statement for feedback
				send_float(kd);
			}
		}

		else if (serial_command == SET_TARGET_ANGLE){
			if (Serial.available() >= sizeof(float)) { // Check if enough bytes available
				byte buffer[sizeof(float)];
				Serial.readBytes(buffer, sizeof(float));
				memcpy(&desired_angle, buffer, sizeof(float));

				// -------------- Echo statement for feedback
				send_float(desired_angle);
			}
		}
		
		else {
			// Nothing
		}
	}

	first_pass_modality_change = false; //Update flag for first pass on modality change
}

// void loop() {

// 	magneticEncoderMonitoring();
// 	// Serial.print("===> angle: ");
// 	// Serial.println(totalAngle);

// 	error = desired_angle - totalAngle;
// 	integrator.push(error);
// 	Serial.print("error: ");
// 	Serial.println(error);

// 	if (fabs(error) < 0.3){
// 		if (counter == 9) { counter = 0;}
// 		Serial.println("\n*** Target reached! ***");
// 		magneticEncoderMonitoring();
// 		Serial.print("===> Desired angle: ");
// 		Serial.print(desired_angle);
// 		Serial.print(" || Final angle: ");
// 		Serial.println(totalAngle);
// 		counter++;
// 		desired_angle = target_angle[counter];
// 		Serial.print("new desired_angle: ");
// 		Serial.println(desired_angle);
// 		delay(2000);
// 		Serial.println("=========== PID PROCESS ==========");
// 	}

// 	else {
// 		// Execute PID algorithm
// 		derivative = error - prev_error;
// 		prev_error = error;
// 		int steps = kp*error + ki*buffer_summation(integrator) + kd*derivative;

// 		// Send new input
// 		stepper.move(steps);
// 		delay(loop_delay);
// 	}

// }