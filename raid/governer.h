#ifndef governer_h
#define governer_h

#include "configuration.h"
#include "hardware.h"
#include "algorithms.h"

extern PID rightVelocityPID;
extern PID leftVelocityPID;

#include <LiquidCrystal_I2C.h>

extern LiquidCrystal_I2C lcd;
extern MUX TOFMUX;

#include <vector>

extern double globalStates[];
extern bool globalStatesUpdated[];

extern LED frontLED;
extern LED backLED;

extern bool modeReady;

extern void lcdPrintAt(String, byte, byte);

class modeGoverner
{
	public:
		modeGoverner(const std::vector<int> &permissibleModes); //debug, standby, BT, learn, mission
		void printInfo();
		bool setMode(int mode, bool forced = false);
		String modeNumberToString(unsigned int modeNumber);
	private:
		//bool _permissibleTransitions;
		bool _permissibleModes[6] = {false, false, false, false, false, false};
		String _modeMapping[6];
};

#endif

