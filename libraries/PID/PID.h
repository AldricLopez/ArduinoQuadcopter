/*
  Por Aldric
*/
#ifndef PID_h
#define PID_h
#include "Arduino.h"

class PID
{
public:
	PID(); //constructor
	void setup_PID(float kp, float ki, float kd, float Max, float MaxI);//darle valores a las constantes, asi como los valores Maximos
	void calc_LPF_D(float freq, float cutoff);//frequencias en Hz
	float get_PID(float Error);//Calcular el valor del PID
	void reset_I();//Resetear la integracion asi como el tiempo previo
private:
	//variables del constantes pid, max debe ser positivo
	float _kp;
	float _ki;
	float _kd;
	float _Max;
	float _MaxI;
	float _LPF_D;//pasabajas para D
    //variables para diferenciales e integrales
    unsigned long _t_prev;
	float _Error_prev;
	float _I_prev;
	float _D_prev;
	
};

#endif
