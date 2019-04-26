#ifndef sensing_h
#define sensing_h

#include "Arduino.h"
#include "configuration.h"

#include "VL53L1X.h"

VL53L1X TOFDummy;

const byte muxPorts[] = {FRONT_TOF_1_PORT, FRONT_TOF_2_PORT, FRONT_TOF_3_PORT, FRONT_TOF_4_PORT, BACK_TOF_PORT};

void initSensors(){
	//Initialise TOF Sensors
	for (int i = 0; i < 5; ++i) {
		TOFMUX.select(muxPorts[i]);
		delay(10);

	    TOFDummy.setTimeout(500);
	    delay(10);

	    while (!TOFDummy.init())
	    {
	      Serial.print("Failed to detect and initialize sensor: ");
	      delay(10);
	      Serial.println(muxPorts[i]);
	      //while (1);
	    }

	    delay(10);
	    TOFDummy.setDistanceMode(VL53L1X::Long);

	    delay(10);
	    TOFDummy.setMeasurementTimingBudget(TOF_POLL_PERIOD_mS * 1000);

	    delay(10);
	    TOFDummy.startContinuous(TOF_POLL_PERIOD_mS);
	}	

	//Initialise Compass
	pinMode(COMPASS_PIN, INPUT);
}

int getHeading() {
	int duration = pulseIn(COMPASS_PIN, HIGH, 3360);
	return (duration >= 600 && duration <= 1680) ? (((duration - 600)/3) + HEADING_OFFSET)%360 : 999;
}

struct sensorReading
{
	unsigned int reading;
	int quality = 0;
	unsigned long timestamp;
};

struct sensorReadings
{
	sensorReading frontTOF1;
	sensorReading frontTOF2;
	sensorReading frontTOF3;
	sensorReading frontTOF4;
	sensorReading backTOF;

	sensorReading compass;
};

sensorReadings currentSensorReadings;

unsigned long count = 0;

void pollSensors() {
	//Poll Compass
	int temp = getHeading();
	if (temp != 999) {
		currentSensorReadings.compass.reading = temp;
		currentSensorReadings.compass.timestamp = millis();
	}
	
	//Poll TOFs
	switch (count % 5){
		case 0:
			TOFMUX.select(FRONT_TOF_1_PORT);
			TOFDummy.read();
			if (TOFDummy.ranging_data.range_status >= 3 && TOFDummy.ranging_data.range_status != 7 && globalStates[0] >= 2) break; 
			currentSensorReadings.frontTOF1.reading = TOFDummy.ranging_data.range_mm;
			currentSensorReadings.frontTOF1.quality = TOFDummy.ranging_data.range_status;
			currentSensorReadings.frontTOF1.timestamp = millis();
			break;
		case 1:
			TOFMUX.select(FRONT_TOF_2_PORT);
			TOFDummy.read();
			if (TOFDummy.ranging_data.range_status >= 3 && TOFDummy.ranging_data.range_status != 7 && globalStates[0] >= 2) break; 
			currentSensorReadings.frontTOF2.reading = TOFDummy.ranging_data.range_mm;
			currentSensorReadings.frontTOF2.quality = TOFDummy.ranging_data.range_status;
			currentSensorReadings.frontTOF2.timestamp = millis();	
			break;																													
		case 2:
			TOFMUX.select(FRONT_TOF_3_PORT);
			TOFDummy.read();
			if (TOFDummy.ranging_data.range_status >= 3 && TOFDummy.ranging_data.range_status != 7 && globalStates[0] >= 2) break; 
			currentSensorReadings.frontTOF3.reading = TOFDummy.ranging_data.range_mm;
			currentSensorReadings.frontTOF3.quality = TOFDummy.ranging_data.range_status;
			currentSensorReadings.frontTOF3.timestamp = millis();
			break;
		case 3:
			TOFMUX.select(FRONT_TOF_4_PORT);
			TOFDummy.read();
			if (TOFDummy.ranging_data.range_status >= 3 && TOFDummy.ranging_data.range_status != 7 && globalStates[0] >= 2) break; 
			currentSensorReadings.frontTOF4.reading = TOFDummy.ranging_data.range_mm;
			currentSensorReadings.frontTOF4.quality = TOFDummy.ranging_data.range_status;
			currentSensorReadings.frontTOF4.timestamp = millis();
			break;
		case 4:
			TOFMUX.select(BACK_TOF_PORT);
			TOFDummy.read();
			if (TOFDummy.ranging_data.range_status >= 3 && TOFDummy.ranging_data.range_status != 7 && globalStates[0] >= 2) break; 
			currentSensorReadings.backTOF.reading = TOFDummy.ranging_data.range_mm;
			currentSensorReadings.backTOF.quality = TOFDummy.ranging_data.range_status;
			currentSensorReadings.backTOF.timestamp = millis();
			break;
	}
	count++;
}

void printCurrentSensorReadings() {
	//while(!Serial);

	Serial.print("frontTOF1: "); 
	Serial.print(currentSensorReadings.frontTOF1.reading); 
	Serial.print("\t"); 
	Serial.print(currentSensorReadings.frontTOF1.quality); 
	Serial.print(" "); 
	Serial.println(currentSensorReadings.frontTOF1.timestamp);

	Serial.print("frontTOF2: "); 
	Serial.print(currentSensorReadings.frontTOF2.reading); 
	Serial.print("\t"); 
	Serial.print(currentSensorReadings.frontTOF2.quality); 
	Serial.print(" "); 
	Serial.println(currentSensorReadings.frontTOF2.timestamp);

	Serial.print("frontTOF3: "); 
	Serial.print(currentSensorReadings.frontTOF3.reading); 
	Serial.print("\t"); 
	Serial.print(currentSensorReadings.frontTOF3.quality); 
	Serial.print(" "); 
	Serial.println(currentSensorReadings.frontTOF3.timestamp);

	Serial.print("frontTOF4: "); 
	Serial.print(currentSensorReadings.frontTOF4.reading); 
	Serial.print("\t"); 
	Serial.print(currentSensorReadings.frontTOF4.quality); 
	Serial.print(" "); 
	Serial.println(currentSensorReadings.frontTOF4.timestamp);

	Serial.print("backTOF: "); 
	Serial.print(currentSensorReadings.backTOF.reading); 
	Serial.print("\t"); 
	Serial.print(currentSensorReadings.backTOF.quality); 
	Serial.print(" "); 
	Serial.println(currentSensorReadings.backTOF.timestamp);
	Serial.println();
}

struct rt
{
	double r;
	double p;
	double h;
	double x;
	double y;
	double z;
};

struct xyz
{
	double x;
	double y;
	double z;
};

xyz decomposeSensorReading(sensorReading input, rt transforms) {
	xyz temp;
	temp.x = (input.reading * cos(transforms.p) * sin(transforms.h)) + transforms.x;
	temp.y = (input.reading * cos(transforms.p) * cos(transforms.h)) + transforms.y;
	double tempZ = pow(pow(input.reading, 2) - (pow(temp.x, 2) + pow(temp.y, 2)), 0.5);
	temp.z = (transforms.p > 0) ? tempZ + transforms.z : (-1 * tempZ) + transforms.z;
	return temp;
}

String getGPSTime() {
	String tempSeconds = gps.time.second();
    tempSeconds = (tempSeconds.length() < 2) ? "0" + tempSeconds : tempSeconds;
    String tempMins = gps.time.minute();
    tempMins = (tempMins.length() < 2) ? "0" + tempMins : tempMins;
    String tempHour = (gps.time.hour() + 8) % 24;
    tempHour = (tempHour.length() < 2) ? "0" + tempHour : tempHour;
    return tempHour + ":" + tempMins + ":" + tempSeconds;
}

String getGPSString() {    
    return getGPSTime() + "," + String(gps.location.lat(),6) + "," + String(gps.location.lng(),6);
}

byte getButtonStatus() {
	if (digitalRead(UP_BUTTON_PIN)) {
    	while (digitalRead(UP_BUTTON_PIN) && !modeReady);
    	return 1;
  	}
	else if (digitalRead(DOWN_BUTTON_PIN) ) {
		while (digitalRead(DOWN_BUTTON_PIN) && !modeReady);
		return 2;
	}
	else if (digitalRead(LEFT_BUTTON_PIN)) {
		while (digitalRead(LEFT_BUTTON_PIN) && !modeReady);
		return 3;
	}
	else if (digitalRead(RIGHT_BUTTON_PIN)) {
		while (digitalRead(RIGHT_BUTTON_PIN) && !modeReady);
		return 4;
	}
	else if (digitalRead(SELECT_BUTTON_PIN)) {
		while (digitalRead(SELECT_BUTTON_PIN) && !modeReady);
		return 5;
	}
	return 0;
}


struct polar
{
	double angle = 0;
	double distance = 0;
};

/*
double degreesToRadians(double degrees) {
	return (PI * degrees) / 180.0;
}

double radiansToDegrees(double radians) {
	return (180 * radians) / PI;
}

polar coordAngleDistance(double latFrom, double lonFrom, double latTo, double lonTo) {

  	unsigned int earthRadiusm = 6371000;

  	double dLat = degreesToRadians(latTo - latFrom);
  	double dLon = degreesToRadians(lonTo - lonFrom);

  	latFrom = degreesToRadians(latFrom);
  	latTo = degreesToRadians(latTo);

  	polar temp;

  	double a = sin(dLat / 2.0) * sin(dLat / 2.0) + sin(dLon / 2.0) * sin(dLon / 2.0) * cos(latFrom) * cos(latTo); 
  	double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a)); 
  
  	temp.distance = earthRadiusm * c;

  	double y = sin(dLon) * cos(latTo);
  	double x = (cos(latFrom) * sin(latTo)) - (sin(latFrom) * cos(latTo) * cos(dLon));

  	temp.angle = 360 - (((int)radiansToDegrees(atan2(y, x)) + 360) % 360);

  	return temp;
}
*/

#endif