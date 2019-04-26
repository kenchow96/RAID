#include "Arduino.h"

#include "governer.h"

modeGoverner::modeGoverner(const std::vector<int> &permissibleModes)
{
	//Serial.println("initialising governer");
	for (unsigned int i = 0; i < permissibleModes.size(); ++i) {
		_permissibleModes[permissibleModes[i]] = true;
		//Serial.println(_permissibleModes[i]);
	}

	_modeMapping[DEBUG_MODE] = "DEBUG  ";
	_modeMapping[STANDBY_MODE] = "STANDBY";
	_modeMapping[BT_MODE] = "BT     ";
	_modeMapping[LEARN_MODE] = "LEARN  ";
	_modeMapping[MISSION_MODE] = "MISSION";
	_modeMapping[ESTOP_MODE] = "ESTOP  ";
}

void modeGoverner::printInfo()
{
	for (int i = 0; i < 6; ++i) {
		Serial.print(_permissibleModes[i]); Serial.print(" ");
	}
	Serial.println();
}

bool modeGoverner::setMode(int mode, bool forced)
{
	if (globalStates[0] == mode) return true;

	if (((mode < 0 || mode > 4) && mode != ESTOP_MODE) || !_permissibleModes[mode]) {
		// invalid mode command
		//this->printInfo();
		return false;	
	}
	else {
		if (globalStates[0] != ESTOP_MODE || forced){

			modeReady = false;

			leftVelocityPID.setSetpoint(0);
			rightVelocityPID.setSetpoint(0);

			if (globalStates[2] != 0){
				globalStatesUpdated[2] = true;
				globalStates[2] = 0;
			}

			globalStates[0] = mode; //Serial.println(globalStates[0]);
			globalStatesUpdated[0] = true;

			//TOFMUX.select(LCD_PORT);
    		//lcd.setCursor(6,0);
    		//lcd.print(String(this->modeNumberToString((unsigned int)globalStates[0])));
    		//lcd.setCursor(0,1);
    		//lcd.print("                ");

    		lcdPrintAt(String(this->modeNumberToString((unsigned int)globalStates[0])),6,0);
    		lcdPrintAt("                ",0,1);

			switch ((int)globalStates[0]) {
				case DEBUG_MODE:
				{
					frontLED.setColor(WHITE); backLED.setColor(WHITE);
        			frontLED.setBlinkPeriod(0); backLED.setBlinkPeriod(0);
        			break;
				}

				case STANDBY_MODE:
				{
					frontLED.setColor(ORANGE); backLED.setColor(ORANGE);
        			frontLED.setBlinkPeriod(0); backLED.setBlinkPeriod(0);
        			break;
				}

				case BT_MODE:
				{
					frontLED.setColor(BLUE); backLED.setColor(BLUE);
        			frontLED.setBlinkPeriod(0); backLED.setBlinkPeriod(0);
        			frontLED.setIntensity(255); backLED.setIntensity(255);
        			break;
				}

				case LEARN_MODE:
				{
					frontLED.setColor(GREEN); backLED.setColor(GREEN);
        			frontLED.setBlinkPeriod(0); backLED.setBlinkPeriod(0);
        			frontLED.setIntensity(255); backLED.setIntensity(255);
        			break;
				}

				case MISSION_MODE:
				{
					frontLED.setColor(WHITE); backLED.setColor(WHITE);
        			frontLED.setBlinkPeriod(0); backLED.setBlinkPeriod(0);
        			frontLED.setIntensity(255); backLED.setIntensity(255);
        			break;
				}

				case ESTOP_MODE:
				{
					frontLED.setColor(RED); backLED.setColor(RED);
        			frontLED.setBlinkPeriod(100); backLED.setBlinkPeriod(100);
        			frontLED.setIntensity(255); backLED.setIntensity(255);
        			break;
				}

				default:
				{

				}
			}

			delay(250);

			return true;
		}
		else {
			// currently in estop state
			return false;

		}
	}
}

String modeGoverner::modeNumberToString(unsigned int modeNumber)
{
	return (modeNumber > 5) ? "" : _modeMapping[modeNumber];
}