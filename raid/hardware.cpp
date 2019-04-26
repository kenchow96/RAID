#include "Arduino.h"

#include "Wire.h"
extern "C" {
#include "utility/twi.h"  
}

#include "hardware.h"

Driver::Driver(String NAME, byte PWM_PIN, byte DIR_PIN, byte RESET_PIN = 0, byte FF1_PIN = 0, byte FF2_PIN = 0)
{
  pinMode(PWM_PIN, OUTPUT); pinMode(DIR_PIN, OUTPUT); 

  if (RESET_PIN) pinMode(RESET_PIN, OUTPUT);
  if (FF1_PIN) pinMode(FF1_PIN, INPUT);
  if (FF2_PIN) pinMode(FF2_PIN, INPUT);

  _NAME = NAME; _PWM_PIN = PWM_PIN; _DIR_PIN = DIR_PIN; _RESET_PIN = RESET_PIN; _FF1_PIN = FF1_PIN; _FF2_PIN = FF2_PIN;
}

void Driver::writePWM(int PWM)
{
  _PWM = PWM;
  analogWrite(_PWM_PIN, constrain(abs(_PWM), 0, 255));
  (_PWM < 0) ? digitalWrite(_DIR_PIN, LOW) : digitalWrite(_DIR_PIN, HIGH);
}

void Driver::brake()
{
  digitalWrite(_PWM_PIN, LOW);
}

void Driver::printInfo()
{
  Serial.print(_NAME); Serial.println(" info:");
  Serial.print("_PWM_PIN: "); Serial.println(_PWM_PIN);
  Serial.print("_DIR_PIN: "); Serial.println(_DIR_PIN);
  Serial.print("_RESET_PIN: "); Serial.println(_RESET_PIN);
  Serial.print("_FF1_PIN: "); Serial.println(_FF1_PIN);
  Serial.print("_FF2_PIN: "); Serial.println(_FF2_PIN);
}

MUX::MUX(byte MUXAddr, byte TOFAddr)
{
  _MUXAddr = MUXAddr;
  _TOFAddr = TOFAddr;

  //while (!Serial);
  //delay(1000);

  Wire.begin();
  Wire.setClock(400000);

  //if (DEBUG) scan();
}

void MUX::select(byte i)
{
  _i = i;

  if (i > 7) return;

  Wire.beginTransmission(_MUXAddr);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void MUX::scan()
{

  Serial.println("\nTCAScanner ready!");

  for (uint8_t t = 0; t < 8; t++) {
    select(t);
    Serial.print("TCA Port #"); Serial.println(t);

    for (uint8_t addr = 0; addr <= 127; addr++) {
      if (addr == _MUXAddr) continue;

      uint8_t data;
      if (! twi_writeTo(addr, &data, 0, 1, 1)) {
        if (addr == _TOFAddr) {
          Serial.println("Found TOF");
        }
        else {
          Serial.print("Found I2C 0x");  Serial.println(addr, HEX);
        }
      }
    }
  }
  Serial.println("\ndone");
}

LED::LED(String NAME, byte RED_PIN, byte GREEN_PIN, byte BLUE_PIN, byte LED_PIN, bool isRGB)
{
  _RED_PIN = RED_PIN;
  _GREEN_PIN = GREEN_PIN;
  _BLUE_PIN = BLUE_PIN;
  _LED_PIN = LED_PIN;
  _isRGB = isRGB;

  if (_isRGB){
    pinMode(_RED_PIN, OUTPUT);
    pinMode(_GREEN_PIN, OUTPUT);
    pinMode(_BLUE_PIN, OUTPUT);
  }
  else {
    pinMode(_LED_PIN, OUTPUT);
  }

}

void LED::setBlinkPeriod(unsigned int blinkPeriodms)
{
  if (blinkPeriodms == _blinkPeriodms) return;
  _blinkPeriodms = blinkPeriodms;
  if (_blinkPeriodms) {
    _blinkPeriodms = max ((unsigned int)100, _blinkPeriodms);
    _ledMetro.interval(_blinkPeriodms);
    _noBlink = false;
  }
  else {
    _noBlink = true;
  }
}

void LED::setColor(RGB color)
{
  _color = color;

  double _maxColor = max(color.red, max(color.blue, color.green));

  _color.red /= _maxColor;
  _color.green /= _maxColor;
  _color.blue /= _maxColor;
}

void LED::setIntensity(byte intensity)
{
  _intensity = intensity;
}

void LED::updateLED(){
  if (_ledMetro.check()) {
    _ledState = !_ledState;
  }

  if (_ledState || _noBlink) {
    if (_isRGB){
      analogWrite(_RED_PIN, _color.red * _intensity);
      analogWrite(_GREEN_PIN, _color.green * _intensity);
      analogWrite(_BLUE_PIN, _color.blue * _intensity);
    }
    else {
      analogWrite(_LED_PIN, _intensity);
    }
    
  }
  else {
    if (_isRGB){
      analogWrite(_RED_PIN, 0);
      analogWrite(_GREEN_PIN, 0);
      analogWrite(_BLUE_PIN, 0);
    }
    else {
      analogWrite(_LED_PIN, 0);
    }
  }
}