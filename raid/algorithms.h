#ifndef algorithms_h
#define algorithms_h

class PID
{
	public:
		PID(double KP = 0.0, double KI = 0.0, double KI_Max = 0.0, double KD = 0.0);
		void setSetpoint(double setPoint);
		void clearAccumulator();
		double calculatePID(double input);
	private:
		double _KP;
		double _KI, _KI_Max;
		double _KD;

		double _setPoint;

		double _accumulator = 0.0;
		double _lastError = 0.0;
};

#endif
