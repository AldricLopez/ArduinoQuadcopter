/*
 Por Aldric
*/

#include "Arduino.h"
#include "PID.h"

PID::PID()
{
   _kp=0.0f;
   _ki=0.0f;
   _kd=0.0f;
   _Max=0.0f;
   _MaxI=0.0f;

   _t_prev=millis();
   _Error_prev=0.0f;
   _I_prev=0.0f;
   _D_prev=0.0f;
   _LPF_D=1.0f;//desactivado
 
}

void PID::setup_PID(float kp, float ki, float kd, float Max, float MaxI)
{
_kp=kp;
_ki=ki;
_kd=kd;
_Max=Max;
_MaxI=MaxI;
}

void PID::calc_LPF_D(float freq, float cutoff)
{
  float T=1/freq;

  float rc=1/(2*PI*cutoff);
  _LPF_D=T/(T+rc);

}

float PID::get_PID(float Error)
{

unsigned long t_actual = millis();//obtener tiempo actual

float dt=(float)(t_actual-_t_prev)/1000.0f; //diferencial de tiempo

//Calcular valores de PID
  float P = _kp*Error;
  float I = _ki*Error*dt + _I_prev;
  float D = _kd * (Error-_Error_prev)/dt;

//Pasabajas para parte derivativa
  D=_D_prev + _LPF_D*(D-_D_prev);  

//Antiwindup para integral
  I=constrain(I,-_MaxI,_MaxI);

//Salida de PID
 float output=P+I+D;
 output=constrain(output,-_Max,_Max);

 //preparar valores para siguiente vuelta
   _t_prev=t_actual;
  _Error_prev = Error;
  _I_prev = I;
  _D_prev = D;

return output;
}

void PID::reset_I()
{
  _I_prev=0.0f;
  _t_prev=millis();
}
