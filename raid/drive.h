#ifndef drive_h
#define drive_h

#include "Arduino.h"
#include "Encoder.h"

#include "configuration.h"

#include "init.h"

#include "algorithms.h"

//Velocity ISR
volatile long bottomLeftEncPos, bottomRightEncPos, topLeftEncPos, topRightEncPos = -999;
volatile long bottomLeftEncDelta, bottomRightEncDelta, topLeftEncDelta, topRightEncDelta = -999;

void getTickDeltas() {
  bottomLeftEncDelta = bottomLeftEnc.read() - bottomLeftEncPos;
  bottomLeftEncPos += bottomLeftEncDelta;
  topLeftEncDelta = topLeftEnc.read() - topLeftEncPos;
  topLeftEncPos += topLeftEncDelta;
  bottomRightEncDelta = bottomRightEnc.read() - bottomRightEncPos;
  bottomRightEncPos += bottomRightEncDelta;
  topRightEncDelta = topRightEnc.read() - topRightEncPos;
  topRightEncPos += topRightEncDelta;
}

struct velocities
{
	double bottomLeft;
	double topLeft;
	double bottomRight;
	double topRight;
};

velocities getVelocities() {
	velocities output;
	double bottomLeftEncDeltaCopy, bottomRightEncDeltaCopy, topLeftEncDeltaCopy, topRightEncDeltaCopy;

	noInterrupts();
  	bottomLeftEncDeltaCopy = bottomLeftEncDelta;
  	bottomRightEncDeltaCopy = bottomRightEncDelta; 
  	topLeftEncDeltaCopy = topLeftEncDelta;
  	topRightEncDeltaCopy = topRightEncDelta;
  	interrupts();

  	output.bottomLeft = bottomLeftEncDeltaCopy / (VELOCITY_UPDATE_TIME_uS/1000000.0) / double(CPR) * WHEEL_CIRCUMFERENCE;
  	output.bottomRight = bottomRightEncDeltaCopy / (VELOCITY_UPDATE_TIME_uS/1000000.0) / double(CPR) * WHEEL_CIRCUMFERENCE;
  	output.topLeft = topLeftEncDeltaCopy / (VELOCITY_UPDATE_TIME_uS/1000000.0) / double(CPR) * WHEEL_CIRCUMFERENCE;
  	output.topRight = topRightEncDeltaCopy / (VELOCITY_UPDATE_TIME_uS/1000000.0) / double(CPR) * WHEEL_CIRCUMFERENCE;

  	/*
  	Serial.print("bottomLeft velocity: "); Serial.println(output.bottomLeft);
  	Serial.print("bottomRight velocity: "); Serial.println(output.bottomRight);
  	Serial.print("topLeft velocity: "); Serial.println(output.topLeft);
  	Serial.print("topRight velocity: "); Serial.println(output.topRight);
  	*/

  	return output;
}


PID rightVelocityPID(VELOCITY_KP, VELOCITY_KI, VELOCITY_KI_MAX);
PID leftVelocityPID(VELOCITY_KP, VELOCITY_KI, VELOCITY_KI_MAX);

void setSpeed() {
	velocities temp = getVelocities();

	double leftVelocity = temp.topLeft;//(temp.bottomLeft + temp.topLeft) / 2;
	double rightVelocity = temp.topRight;//(temp.bottomRight + temp.topRight) / 2;

	leftDriver.writePWM(leftVelocityPID.calculatePID(leftVelocity));
	rightDriver.writePWM(rightVelocityPID.calculatePID(rightVelocity));
}

double boundSpeed(double speed){
	double closestObstacleDistance = 0.0;

	if (speed > 0){
		xyz frontTOF1XYZ = decomposeSensorReading(currentSensorReadings.frontTOF1, frontTOF1Transforms);
		xyz frontTOF2XYZ = decomposeSensorReading(currentSensorReadings.frontTOF2, frontTOF2Transforms);
		xyz frontTOF3XYZ = decomposeSensorReading(currentSensorReadings.frontTOF3, frontTOF3Transforms);
		xyz frontTOF4XYZ = decomposeSensorReading(currentSensorReadings.frontTOF4, frontTOF4Transforms);
		
		if (millis() - currentSensorReadings.frontTOF1.timestamp > 100) frontTOF1XYZ.y = 4000;
		if (millis() - currentSensorReadings.frontTOF2.timestamp > 100) frontTOF2XYZ.y = 4000;
		if (millis() - currentSensorReadings.frontTOF3.timestamp > 100) frontTOF3XYZ.y = 4000;
		if (millis() - currentSensorReadings.frontTOF4.timestamp > 100) frontTOF4XYZ.y = 4000;

		closestObstacleDistance = min(min(frontTOF1XYZ.y, frontTOF2XYZ.y), min(frontTOF3XYZ.y, frontTOF4XYZ.y));
		
		/*
		Serial.print("X"); Serial.println(frontTOF1XYZ.x);
		Serial.print("Y"); Serial.println(frontTOF1XYZ.y);
		Serial.print("Z"); Serial.println(frontTOF1XYZ.z);
		Serial.println();
		*/

		//Serial.println(closestObstacleDistance);

  		frontLED.setColor(RED); frontLED.setIntensity(255);
		frontLED.setBlinkPeriod(closestObstacleDistance / 4.0);

		double maxSpeed = constrain((closestObstacleDistance - DEAD_ZONE_MM) / ((TOF_MAX_M - (DEAD_ZONE_MM/1000.0)) * 1000.0), -0.2, 1.0) * MAX_VELOCITY;
		
		//Serial.println(speed);

		return min(speed, maxSpeed);
	}
	else if (speed < 0){
		xyz backTOFXYZ = decomposeSensorReading(currentSensorReadings.backTOF, backTOFTransforms);
		
		if (millis() - currentSensorReadings.backTOF.timestamp > 100) backTOFXYZ.y = 4000;

		closestObstacleDistance = backTOFXYZ.y;

  		backLED.setColor(RED); backLED.setIntensity(255);
		backLED.setBlinkPeriod(closestObstacleDistance / 4.0);

		double maxSpeed = -1 * constrain((closestObstacleDistance - DEAD_ZONE_MM) / ((TOF_MAX_M - (DEAD_ZONE_MM/1000.0)) * 1000.0), -1.0, 1.0) * MAX_VELOCITY;
		return max(speed, maxSpeed);
	}
	else{
		return 0;
	}
}

void move(double speed){

	speed = boundSpeed(speed);

	//Serial.print("bounded "); Serial.println(speed);

	if (globalStates[2] != (int)speed){
		//Serial.print(globalStates[2]);
		//Serial.println(speed);
		globalStatesUpdated[2] = true;
		globalStates[2] = speed;
	}

	rightVelocityPID.setSetpoint((globalStates[0] != ESTOP_MODE) ? speed : 0);
	leftVelocityPID.setSetpoint((globalStates[0] != ESTOP_MODE) ? speed : 0);
}

//speed is in mm/s
void move(double speed, double radius){

	speed = boundSpeed(speed);

	if (globalStates[2] != (int)speed){
		//Serial.print(globalStates[2]);
		//Serial.println(speed);
		globalStatesUpdated[2] = true;
		globalStates[2] = speed;
	}
	
	rightVelocityPID.setSetpoint((globalStates[0] != ESTOP_MODE) ? (radius - (WHEEL_TRACK / 2.0)) * (speed / radius) : 0);
	leftVelocityPID.setSetpoint((globalStates[0] != ESTOP_MODE) ? (radius + (WHEEL_TRACK / 2.0)) * (speed / radius) : 0);
}

void move2(double speed, double turnPercentage = 0){

	speed = boundSpeed(speed);

	if (globalStates[2] != (int)speed){
		//Serial.print(globalStates[2]);
		//Serial.println(speed);
		globalStatesUpdated[2] = true;
		globalStates[2] = speed;
	}
	
	if (turnPercentage > 0){
		leftVelocityPID.setSetpoint((globalStates[0] != ESTOP_MODE) ? speed : 0);
		rightVelocityPID.setSetpoint((globalStates[0] != ESTOP_MODE) ? speed - (abs(turnPercentage / 100.0) * 2 * speed) : 0);
	}
	else {
		rightVelocityPID.setSetpoint((globalStates[0] != ESTOP_MODE) ? speed : 0);
		leftVelocityPID.setSetpoint((globalStates[0] != ESTOP_MODE) ? speed - (abs(turnPercentage / 100.0) * 2 * speed) : 0);
	}
	
}

#endif