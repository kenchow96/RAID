#ifndef comms_h
#define comms_h

#include <Metro.h>

#include "configuration.h"

#include "init.h"

void updateGPSState(){
	//Serial.println(naza.getSatellites());
	if (globalStates[1] != gps.satellites.value()){
		globalStates[1] = gps.satellites.value();
		globalStatesUpdated[1] = true;
	}
	//Serial.println(naza.getLongitude());
	if (globalStates[3] != gps.location.lng()){
		globalStates[3] = gps.location.lng();
		globalStatesUpdated[3] = true;
	}
	//Serial.println(naza.getLatitude());
	if (globalStates[4] != gps.location.lat()){
		globalStates[4] = gps.location.lat();
		globalStatesUpdated[4] = true;
	}	
}

Metro btMetro = Metro(BT_SEND_INTERVAL_MS);

const String messageTypes[6] = {"M", "S", "V", "L", "I", "W"};

unsigned long lastSentTime[6] = {0, 1, 2, 3, 4, 5};

void teensyToWorld(){
	if (btMetro.check()) {

		updateGPSState();

		byte messagesWaiting = globalStatesUpdated[0] + globalStatesUpdated[1] + globalStatesUpdated[2];
		byte sendIndex = 0;

		//send 
		if (messagesWaiting){
			if (messagesWaiting == 1) {
				sendIndex = (globalStatesUpdated[0]) ? 0 : (globalStatesUpdated[1]) ? 1 : 2;
			}
			else if (messagesWaiting == 2) {
				if (!globalStatesUpdated[0]){
					sendIndex = (lastSentTime[1] < lastSentTime[2]) ? 1 : 2;
				}
				else if (!globalStatesUpdated[1]){
					sendIndex = (lastSentTime[0] < lastSentTime[2]) ? 0 : 2;
				}
				else {
					sendIndex = (lastSentTime[0] < lastSentTime[1]) ? 0 : 1;
				}
			}
			else {
				sendIndex = (lastSentTime[0] < min(lastSentTime[1], lastSentTime[2])) ? 0 : (lastSentTime[1] < min(lastSentTime[0], lastSentTime[2])) ? 1 : 2;
			}
			BT_SERIAL_PORT.print(messageTypes[sendIndex]); 
			BT_SERIAL_PORT.print((int)globalStates[sendIndex]);
			BT_SERIAL_PORT.print(" ");

			RPI_SERIAL.print(messageTypes[sendIndex]); 
			RPI_SERIAL.print((int)globalStates[sendIndex]);
			RPI_SERIAL.print(" ");

			lastSentTime[sendIndex] = millis(); globalStatesUpdated[sendIndex] = false;
			}

		for (int i = 3; i < 6; ++i) {
			if (globalStatesUpdated[i]) {
				RPI_SERIAL.print(messageTypes[i]); 
				RPI_SERIAL.print(globalStates[i], 6);
				RPI_SERIAL.print(" ");

				lastSentTime[i] = millis(); globalStatesUpdated[i] = false;
			}
		}
	}
}

//Inbound Serial Listeners

//Allows any USB device (usually programming device or RPI)
//to switch to eStop mode with any message
String USBString = "";
bool USBStringComplete = false;

void serialEvent() {
	while (Serial.available()){
		char inChar = (char)Serial.read();
		USBString += inChar;
		if (inChar == '\n') {
      		USBStringComplete = true;
      		Serial.println(USBString);
    	}
	}
	if (USBStringComplete){
		if (USBString[0] == 'M' && USBString[1] == '5') gov.setMode(ESTOP_MODE);
		USBString = "";
		USBStringComplete = false;
	}
	
}

/* Parses GPS messages into object
Heading: Serial.print(naza.getHeading(), 2);
Lat: Serial.print(naza.getLatitude());
Long: Serial.print(naza.getLongitude());
Alt: Serial.print(naza.getAltitude(), 7);
Fix Type: Serial.print(naza.getFixType());
Sats: Serial.print(naza.getSatellites());
Locked: Serial.println(naza.isLocked());
*/

void serialEvent1() {
	while (Serial1.available()) {
		//naza.decode(Serial1.read());
		gps.encode(Serial1.read());
	}

}

//Serial listener for HC05 connected to companion app

String inputString = "";
bool stringComplete = false;

struct message
{
	int data;
	unsigned long timestamp;
};

struct btMessage
{
	message targetSpeed = {0, 0};
	message remoteDirection = {5, 0};
	message command = {3, 0};
	unsigned long lastAlive = 0;
};

btMessage latestBTMessage;

void serialEvent3() {
	while (Serial3.available()){
		char inChar = (char)Serial3.read();
		inputString += inChar;
		if (inChar == '\n') {
      		stringComplete = true;
    	}
	}
	if (stringComplete) {
    	//Serial.print(inputString);
    	switch (inputString[0]){
    		case 'T':
    			latestBTMessage.targetSpeed.data = inputString.substring(1).toInt();
    			latestBTMessage.targetSpeed.timestamp = millis();
    			latestBTMessage.lastAlive = millis();
    			break;
			case 'M':
				//Serial.println(inputString.substring(1).toInt());
				gov.setMode(inputString.substring(1).toInt());
				latestBTMessage.lastAlive = millis();
    			break;
    		case 'R':
    			latestBTMessage.remoteDirection.data = (globalStates[0] == 2) ? inputString.substring(1).toInt() : 5;
    			latestBTMessage.remoteDirection.timestamp = millis();
    			latestBTMessage.lastAlive = millis();
    			break;
    		case 'C':
    			latestBTMessage.command.data = inputString.substring(1).toInt();
    			latestBTMessage.command.timestamp = millis();
    			latestBTMessage.lastAlive = millis();
    			break;
    		case 'A':
    			latestBTMessage.lastAlive = millis();
    		default:
    			break;
    	}
    	stringComplete = false;
    	inputString = "";
	}
}

// Serial listener for Jevois Cameras

struct cameraReading
{
	int id = 0;
	int targx = 0;
	int targy = 0;
	int targw = 0;
	int targh = 0;
	int state = 0;
	unsigned long timestamp = 0;
};

String camFrontString = "";
bool camFrontStringComplete = false;
char instr[128 + 1];

cameraReading frontCamReading;

void serialEvent5() {
	while (Serial5.available()){
		char inChar = (char)Serial5.read();
		camFrontString += inChar;
		if (inChar == '\n') {
      		camFrontStringComplete = true;
    	}
	}
	if (camFrontStringComplete){
		//Serial.println(camFrontString);
		camFrontString.toCharArray(instr,camFrontString.length()); // +1?

		instr[camFrontString.length()] = 0;

		char * tok = strtok(instr, " \r\n");
	   	cameraReading temp;
	 
	   	while (tok)
	   	{
	    	// State machine:
	     	// 0: start parsing
	     	// 1: N2 command, parse id
	     	// 2: N2 command, parse targx
	     	// 3: N2 command, parse targy
	     	// 4: N2 command, parse targw
	     	// 5: N2 command, parse targh
	     	// 6: N2 command complete
	     	// 1000: unknown command
	    	switch (temp.state)
	     	{
	       		case 0: if (strcmp(tok, "N2") == 0) temp.state = 1; else temp.state = 1000; break;
	       		case 1: temp.id = atoi(&tok[1]); temp.state = 2; break; // ignore prefix
	       		case 2: temp.targx = atoi(tok); temp.state = 3; break;
	       		case 3: temp.targy = atoi(tok); temp.state = 4; break;
	       		case 4: temp.targw = atoi(tok); temp.state = 5; break;
	       		case 5: temp.targh = atoi(tok); temp.state = 6; break;
	       		default: break; // Skip any additional tokens
	     	}
	     	tok = strtok(0, " \r\n");
	   	}

	   	if (temp.state == 6 && temp.id == ARUCO_ID) {
	   		//Serial.print(temp.targx); Serial.print(" "); Serial.println(temp.targy);
	   		frontCamReading = temp;
	   		frontCamReading.timestamp = millis();
	   	}

	   	camFrontString = "";
	   	camFrontStringComplete = false;																												
	}
}

/*

cameraReading readCamera(HardwareSerial &port){
	char instr[128 + 1];
	byte len = port.readBytesUntil('\n', instr, 128);
   	instr[len] = 0;
 
   	char * tok = strtok(instr, " \r\n");
   	cameraReading temp;
 
   	while (tok)
   	{
    	// State machine:
     	// 0: start parsing
     	// 1: N2 command, parse id
     	// 2: N2 command, parse targx
     	// 3: N2 command, parse targy
     	// 4: N2 command, parse targw
     	// 5: N2 command, parse targh
     	// 6: N2 command complete
     	// 1000: unknown command
    	switch (temp.state)
     	{
       		case 0: if (strcmp(tok, "N2") == 0) temp.state = 1; else temp.state = 1000; break;
       		case 1: temp.id = atoi(&tok[1]); temp.state = 2; break; // ignore prefix
       		case 2: temp.targx = atoi(tok); temp.state = 3; break;
       		case 3: temp.targy = atoi(tok); temp.state = 4; break;
       		case 4: temp.targw = atoi(tok); temp.state = 5; break;
       		case 5: temp.targh = atoi(tok); temp.state = 6; break;
       		default: break; // Skip any additional tokens
     	}
     	tok = strtok(0, " \r\n");
   	}
 
   	// If a complete new N2 command was received, state will be 6:
  	return temp;
}
*/

#endif

