/*
This file will attempt to define all hardware level objects
These objects will be accompanied by relevant functions to
- get inputs in the case of sensors
- perform actions in the case of actuators
*/

#ifndef hardware_h
#define hardware_h

#include <Metro.h>

#include "configuration.h"

#include <LiquidCrystal_I2C.h>

class Driver
{
	public:
    	Driver(String NAME, byte PWM_PIN, byte DIR_PIN, byte RESET_PIN, byte FF1_PIN, byte FF2_PIN);
    	void writePWM(int PWM);
    	void brake();
        void printInfo();
	private:
		byte _PWM_PIN; 
    	byte _DIR_PIN;
    	byte _RESET_PIN; 
    	byte _FF1_PIN;
    	byte _FF2_PIN; 

        int _PWM;

        String _NAME;
};

class MUX
{
    public:
        MUX(byte MUXAddr, byte TOFAddr);
        void select(byte i);
        void scan();
    private:
        byte _i;

        byte _MUXAddr;
        byte _TOFAddr;
};

class JeVois
{

};

struct RGB
{
    double red = 0.0;
    double green = 0.0;
    double blue = 0.0;
};

class LED
{
    public:
        LED(String NAME, byte RED_PIN = 0, byte GREEN_PIN = 0, byte BLUE_PIN = 0, byte LED_PIN = 0, bool isRGB = true);
        void setBlinkPeriod(unsigned int blinkPeriodms);
        void setColor(RGB color);
        void setIntensity(byte intensity);
        void updateLED();
    private:
        byte _RED_PIN, _GREEN_PIN, _BLUE_PIN;
        byte _LED_PIN;
        bool _isRGB;
        unsigned int _blinkPeriodms = 0;
        RGB _color = {1.0, 1.0, 1.0};
        byte _intensity = 255;
        bool _ledState = true;
        bool _noBlink = true;
        Metro _ledMetro = Metro(0);
};

#endif