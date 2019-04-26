#include "algorithms.h"
#include "Arduino.h"

PID::PID(double KP, double KI, double KI_Max, double KD)
{
	_KP = KP;
	_KI = KI; _KI_Max = constrain(KI_Max, 0.0, 1.0);
	_KD = KD;
}

void PID::setSetpoint(double setPoint)
{
	_setPoint = setPoint;
}

void PID::clearAccumulator()
{
	_accumulator = 0;
}

double PID::calculatePID(double processValue)
{
	double error = _setPoint - processValue;

	_accumulator += error;
	_accumulator = constrain(_accumulator, -1.0 * _KI_Max * 255.0 / _KI,  _KI_Max * 255.0 / _KI);

	double temp =  (_KP * error) 
					+ (_KI * _accumulator)
					+ (_KD * (error - _lastError));

	_lastError = error;

	return temp;
}
