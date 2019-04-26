#ifndef init_h
#define init_h

#include "Arduino.h"
#include "Wire.h"
#include "Encoder.h"

#include "configuration.h"

// Global States -> 0: Mode, 1: Sat Links, 2: Velocity, 3: Longitude, 4: Latitude, 5: Warning
double globalStates[6] = {0, 0, 0, 0, 0, 0};
bool globalStatesUpdated[6] = {true, true, true, true, true, true};

void initSerials(){
	Serial.begin(115200);
	BT_SERIAL_PORT.begin(9600);
	GPS_SERIAL_PORT.begin(9600);

	//CAM_FRONT_SERIAL_PORT.setTimeout(50);
	CAM_FRONT_SERIAL_PORT.begin(115200);
	//CAM_FRONT_SERIAL_PORT.setTimeout(50);

	//CAM_RIGHT_SERIAL_PORT.setTimeout(50);
	CAM_RIGHT_SERIAL_PORT.begin(115200);
	//CAM_RIGHT_SERIAL_PORT.setTimeout(50);

	//CAM_BACK_SERIAL_PORT.setTimeout(50);
	CAM_BACK_SERIAL_PORT.begin(115200);
	//CAM_BACK_SERIAL_PORT.setTimeout(50);

	//CAM_LEFT_SERIAL_PORT.setTimeout(50);
	CAM_LEFT_SERIAL_PORT.begin(115200);
	//CAM_LEFT_SERIAL_PORT.setTimeout(50);
}

//Naza GPS no longer in use
//#include <NazaDecoder.h>
//NazaDecoder naza = NazaDecoder();

#include <ubloxGPS.h>
#include <TinyGPS++.h>
TinyGPSPlus gps;

#include <vector>
std::vector<int> permissibleModes{DEBUG_MODE, STANDBY_MODE, BT_MODE, LEARN_MODE, MISSION_MODE, ESTOP_MODE};

bool modeReady = false;

#include "governer.h"
modeGoverner gov(permissibleModes);

#include "comms.h"

#include "hardware.h"

Driver leftDriver("Left Motor Driver", LEFT_PWM_PIN, LEFT_DIR_PIN, LEFT_RESET_PIN, LEFT_FF1_PIN, LEFT_FF2_PIN);
Encoder bottomLeftEnc(BOTTOM_LEFT_ENC_A_PIN, BOTTOM_LEFT_ENC_B_PIN);
Encoder topLeftEnc(TOP_LEFT_ENC_A_PIN, TOP_LEFT_ENC_B_PIN);

Driver rightDriver("Right Motor Driver", RIGHT_PWM_PIN, RIGHT_DIR_PIN, RIGHT_RESET_PIN, RIGHT_FF1_PIN, RIGHT_FF2_PIN);
Encoder bottomRightEnc(BOTTOM_RIGHT_ENC_A_PIN, BOTTOM_RIGHT_ENC_B_PIN);
Encoder topRightEnc(TOP_RIGHT_ENC_A_PIN, TOP_RIGHT_ENC_B_PIN);

MUX TOFMUX(MUX_ADDR, TOF_ADDR);

LED frontLED("Front LED", FRONT_LED_RED_PIN, FRONT_LED_GREEN_PIN, FRONT_LED_BLUE_PIN);
LED backLED("Back LED", BACK_LED_RED_PIN, BACK_LED_GREEN_PIN, BACK_LED_BLUE_PIN);
LED powerStatusLED("Power Status LED", 0,0,0, POWER_STATUS_LED_PIN, false);
LED statusStatusLED("Status Status LED", 0,0,0, STATUS_STATUS_LED_PIN, false);
LED BTStatusLED("BT Status LED", 0,0,0, BT_STATUS_LED_PIN, false);

void runLED(){
	frontLED.updateLED();
	backLED.updateLED();
	powerStatusLED.updateLED();
	BTStatusLED.setIntensity(max(0,255 - (long)(millis() - latestBTMessage.lastAlive)));
	BTStatusLED.updateLED();
}

#include "sensing.h"

rt frontTOF1Transforms = {FRONT_TOF_1_R, FRONT_TOF_1_P, FRONT_TOF_1_H, FRONT_TOF_1_X, FRONT_TOF_1_Y, FRONT_TOF_1_Z};
rt frontTOF2Transforms = {FRONT_TOF_2_R, FRONT_TOF_2_P, FRONT_TOF_2_H, FRONT_TOF_2_X, FRONT_TOF_2_Y, FRONT_TOF_2_Z};
rt frontTOF3Transforms = {FRONT_TOF_3_R, FRONT_TOF_3_P, FRONT_TOF_3_H, FRONT_TOF_3_X, FRONT_TOF_3_Y, FRONT_TOF_3_Z};
rt frontTOF4Transforms = {FRONT_TOF_4_R, FRONT_TOF_4_P, FRONT_TOF_4_H, FRONT_TOF_4_X, FRONT_TOF_4_Y, FRONT_TOF_4_Z};
rt backTOFTransforms = {BACK_TOF_R, BACK_TOF_P, BACK_TOF_H, BACK_TOF_X, BACK_TOF_Y, BACK_TOF_Z};

#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,16,2);

void lcdPrintAt(String input, byte x, byte y) {
  if(x > 15 || y > 1) return;
  TOFMUX.select(LCD_PORT);
  lcd.setCursor(x,y);
  if(input.length() > 16 - x) input = input.substring(0,16 - x);
  for(int i = 0; i < input.length(); ++i){
    lcd.print(input.charAt(i));
    }
}

void initControlPanel(){
	TOFMUX.select(LCD_PORT);
	lcd.init(); 
	lcd.backlight();
	
	lcd.clear();
	lcdPrintAt("STARTING UP",0,0);

	pinMode(LEFT_BUTTON_PIN, INPUT);
	pinMode(SELECT_BUTTON_PIN, INPUT);
	pinMode(DOWN_BUTTON_PIN, INPUT);
	pinMode(RIGHT_BUTTON_PIN, INPUT);
	pinMode(UP_BUTTON_PIN, INPUT);

	pinMode(STATUS_STATUS_LED_PIN, OUTPUT);
	pinMode(BT_STATUS_LED_PIN, OUTPUT);

	powerStatusLED.updateLED();
	BTStatusLED.updateLED();
}

#include "drive.h"

IntervalTimer velocityTimer;
IntervalTimer pidLoopTimer;

void initISRs(){
  	velocityTimer.begin(getTickDeltas, VELOCITY_UPDATE_TIME_uS);
  	pidLoopTimer.begin(setSpeed, VELOCITY_UPDATE_TIME_uS * 2);
}

#include "fileIO.h"

void initSD(){
	//Serial.print("Initializing SD card...");
  
  	// see if the card is present and can be initialized:
  	if (!SD.begin(BUILTIN_SDCARD)) {
    	Serial.println("Card failed, or not present");
    	// don't do anything more:
    	return;
  	}

  	File dataFile = SD.open(GPSLOG_FILENAME, FILE_WRITE);
  	
  	populateFileList();
}


#endif
