#include <Wire.h>
#include <EEPROMex.h>
#include <PID.h>
#include <avr/pgmspace.h>
#include "defines.h"


/*ToDo

implementar pid altitude hold

agregar sensores de los lados

conectar y usar pixy cam

agregar flow optico para hold

agregar gps hold

*/

//Pid´s
//           rate->motors             angle->rate                 rate->thr  alt->rate     
PID PID_roll,PID_pitch,PID_yaw, PID_angleX,PID_angleY,PID_angleZ, PID_alt,  PID_altRate;

//i2cdevlib ADXL345(Acc), L3G4200D(Gyro), HMC5883L(Mag) y BMP085(Baro)
#include "I2Cdev.h"
#include "ADXL345.h"
//#include "L3G4200D.h"
#include <L3G.h>
#include "HMC5883L.h"
#include "BMP085.h"

//booleano para arrancar motores e iniciar PID, o apagarlos al aterrizar
 boolean armed =false;

 //Setpoints de PID
 //primer pid de angulos
 int setX=0;
 int setY=0;
 int setZ=0;
 //segundo pid de rates
 float setRateX=0.0f;
 float setRateY=0.0f;
 float setRateZ=0.0f;
 //pid de alt hold
 float setAlt=0.0f;
 //constantes PID
float
 ROLL_PID_KP,
 ROLL_PID_KI,
 ROLL_PID_KD,
 ROLL_PID_MAX,
 ROLL_PID_MAX_I,

 PITCH_PID_KP,
 PITCH_PID_KI,
 PITCH_PID_KD,
 PITCH_PID_MAX,
 PITCH_PID_MAX_I,

 YAW_PID_KP,
 YAW_PID_KI,
 YAW_PID_KD,
 YAW_PID_MAX,
 YAW_PID_MAX_I,

 ANGLEX_KP,
 ANGLEX_KI,
 ANGLEX_KD,
 ANGLEX_MAX,
 ANGLEX_MAX_I,

 ANGLEY_KP,
 ANGLEY_KI,
 ANGLEY_KD,
 ANGLEY_MAX,
 ANGLEY_MAX_I,

 ANGLEZ_KP,
 ANGLEZ_KI,
 ANGLEZ_KD,
 ANGLEZ_MAX,
 ANGLEZ_MAX_I,

 ALT_KP,
 ALT_KI,
 ALT_KD,
 ALT_MAX,
 ALT_MAX_I,

 ALTRATE_KP,
 ALTRATE_KI,
 ALTRATE_KD,
 ALTRATE_MAX,
 ALTRATE_MAX_I

 ;

 //Valores del receptor
 //Aleron, Elevador, Throttle, Rudder, Aux, dummy
 volatile int Receptor[6]={1500,1500,1000,1500,1000,0};
 //RX estado previo
 volatile int ReceptorPrevVal[6]={0,0,0,0,0,0};
 volatile int ReceptorPrevTim[6]={0,0,0,0,0,0};

 //Sensores
 //Objetos de sensores I2C para i2cdevlib
 ADXL345 accelerometro;
 //L3G4200D giroscopio;
 L3G gyro;
 HMC5883L magnetometro;
 BMP085 barometro;
 //Valores ya procesados de sensores
 float angle_Acc[3]={0,0,0};//Angulos alrededor de x, y, hacia z
 float angvel_Gyro[3]={0,0,0};//velocidad angular alrededor de x, y, z
 float angle_heading=0;
 float angle_Mag;//Angulo del magnetometro
 float alt_Baro=0;//Altura de barometro
 float ground_Baro=0;//Presion en tierra
 float temp_Baro[2]={0,0};//Temperatura inicial y actual del barometro
 boolean  tempNbaro=true;//true leyendo temp false leyendo baro
 float angle[3]={0,0,0};//angulos alrededor de x,y,z
// Tiempos Previos para algunas funciones
volatile double prevMillisAcc=0;
volatile double prevMicrosGyro=0;
volatile double prevMillisMag=0;
volatile double prevMillisBaro=0;
volatile double prevMillisSerial=0;

//Para leer pulsos de sonar LV-EZ0 output PWM
 int sonar_PW[5]={0,0,0,0,0};
volatile int sonar_PrevVal=0;
volatile char sonar_i=0;
volatile boolean sonar_flag=false;
volatile double sonar_PrevTim=0;
volatile float alt_Sonar=0;
volatile float alturaPrev;

float altura=0;//Altura combinada
float altura_Rate=0;//velocidad de cambio de altura

//Arreglo de valores de PWM para motores 490Hz, pulsos de: 1/490*(X/255) 125<x<250
int MotoresPWM[4]={125,125,125,125};

//Para EEPROM
int address_EEPROM;//Donde se va a guardar
float temp_EEPROM[numDataEEPROM];//temporal para leer y escribir

//Serial
boolean debug=false; //si se conecta con la app de processing o no

// Declaraciones

void setupEEPROM();
void readEEPROM_PID();
void interfaz_EEPROM_RW();
void sensoresSetup();
void checarInicio();
void calcAngulos();
void calcDistancias();
void readAcc();
void readGyro();
void set_Home();
float readMag();
void readBaro();
void readTemp();
void pinModeSetup();
void RXInterruptSetup();
void ThrottleISR();
void PIDSetup();
void Estabilizar();
int AltHold();
void apagarMotores();
void DebugSerial();


 void setup(){
   Serial.begin(9600);
   Wire.begin();
   setupEEPROM();
   delay(100);
   pinModeSetup();
   sensoresSetup();
   RXInterruptSetup();
   PIDSetup();
   set_Home();
 }


void loop(){

//Leer los sensores y calcular angulo/////////////////////////////////////////

    if(millis()-prevMillisAcc > 20)
    {
      readAcc();//cada 20 ms
      prevMillisAcc=millis();
    }
    if(micros()-prevMicrosGyro >1300)
    { 
      readGyro();//cada 1.3ms
      prevMicrosGyro=micros();
    }
    if(millis()-prevMillisMag > 60)
    {
      angle_Mag=readMag();//cada 60 ms
      prevMillisMag=millis();
    }

    if((millis()-prevMillisBaro > 24) && (tempNbaro==false))
    {
      readBaro();//cada 24 ms
      tempNbaro=true;
      prevMillisBaro=millis();
    }
    if( (millis()-prevMillisBaro > 5) && (tempNbaro==true) )
    {
      readTemp();//cada 5 ms
      tempNbaro=false;
      prevMillisBaro=millis();
    }


    calcAngulos();//con angle_Acc[], angvel_Gyro[], angle_Mag y ponerlos en angle[]
    calcDistancias();//calcular altura decidiendo que usar, y distancias a los lados
 

//Con el control armar y desarmar los motores y pid///////////////////////////
checarInicio();//ver si se inicia o apaga el pid y los motores

   if(armed && Receptor[2]>1180)
   {
    Estabilizar();//Calcular y mezclar PIDs
   } 
   else
   {
    apagarMotores();//Apagar motores y resetear I
   }  

//Debugear enviando a terminal o aplicacion de processing si se esta conectado///////////////////

if(debug==true)
{
    if(millis()-prevMillisSerial > 100)
    { 
      DebugSerial(); //Envia datos de sensores por serial
      prevMillisSerial=millis();
    }

    interfaz_EEPROM_RW(); //app processing para leer y escribir eeprom
 }
 else
 {
    writeMotors();//escribir a motores solo si no esta conectado a pc

    if(Serial.available()>0)
    {
      char incoming=Serial.read();
      if(incoming=='c') debug=true; //coneccion con pc
    }
 }

 }

//Iniciar EEPROM seguro
void setupEEPROM(){
  address_EEPROM=EEPROM.getAddress(sizeof(float)*numDataEEPROM);
  EEPROM.setMemPool(memBase,EEPROMSizeATmega328);
  EEPROM.setMaxAllowedWrites(maxWrites*numDataEEPROM*4);

  readEEPROM_PID();

}

//Leer valores de PID desde la EEPROM
 void readEEPROM_PID(){
   EEPROM.readBlock(address_EEPROM,temp_EEPROM,numDataEEPROM);

 ROLL_PID_KP=temp_EEPROM[0];
 ROLL_PID_KI=temp_EEPROM[1];
 ROLL_PID_KD=temp_EEPROM[2];
 ROLL_PID_MAX=temp_EEPROM[3];
 ROLL_PID_MAX_I=temp_EEPROM[4];

 PITCH_PID_KP=temp_EEPROM[5];
 PITCH_PID_KI=temp_EEPROM[6];
 PITCH_PID_KD=temp_EEPROM[7];
 PITCH_PID_MAX=temp_EEPROM[8];
 PITCH_PID_MAX_I=temp_EEPROM[9];

 YAW_PID_KP=temp_EEPROM[10];
 YAW_PID_KI=temp_EEPROM[11];
 YAW_PID_KD=temp_EEPROM[12];
 YAW_PID_MAX=temp_EEPROM[13];
 YAW_PID_MAX_I=temp_EEPROM[14];

 ANGLEX_KP=temp_EEPROM[15];
 ANGLEX_KI=temp_EEPROM[16];
 ANGLEX_KD=temp_EEPROM[17];
 ANGLEX_MAX=temp_EEPROM[18];
 ANGLEX_MAX_I=temp_EEPROM[19];

 ANGLEY_KP=temp_EEPROM[20];
 ANGLEY_KI=temp_EEPROM[21];
 ANGLEY_KD=temp_EEPROM[22];
 ANGLEY_MAX=temp_EEPROM[23];
 ANGLEY_MAX_I =temp_EEPROM[24];

 ANGLEZ_KP=temp_EEPROM[25];
 ANGLEZ_KI=temp_EEPROM[26];
 ANGLEZ_KD=temp_EEPROM[27];
 ANGLEZ_MAX=temp_EEPROM[28];
 ANGLEZ_MAX_I=temp_EEPROM[29];

 ALT_KP=temp_EEPROM[30];
 ALT_KI=temp_EEPROM[31];
 ALT_KD=temp_EEPROM[32];
 ALT_MAX=temp_EEPROM[33];
 ALT_MAX_I =temp_EEPROM[34];

 ALTRATE_KP=temp_EEPROM[35];
 ALTRATE_KI=temp_EEPROM[36];
 ALTRATE_KD=temp_EEPROM[37];
 ALTRATE_MAX=temp_EEPROM[38];
 ALTRATE_MAX_I=temp_EEPROM[39];

 }

//Enviar y recibir valores de eeprom por serial
void interfaz_EEPROM_RW()
{
int i;
char incoming;
 if(Serial.available()>0)
 {
  incoming=Serial.read();
  if(incoming=='s')
  {
    Serial.println("Recibiendo");
    for(i=0; i<numDataEEPROM; i++)
    {
      temp_EEPROM[i]=Serial.parseFloat();
    }
    if(!EEPROM.updateBlock(address_EEPROM,temp_EEPROM,numDataEEPROM)){return;};
    delay(500);
    readEEPROM_PID();
    Serial.println("Listo");  
  }
  if(incoming=='r')
  {
    delay(1000);
    EEPROM.readBlock(address_EEPROM,temp_EEPROM,numDataEEPROM);
    for(i=0; i<numDataEEPROM; i++)
    {   
        Serial.print(temp_EEPROM[i],4);
        Serial.print(" ");
    }
    Serial.println("");
    delay(1000);
  }
  if(incoming=='d')
  {
    debug=false;
  }
  
 }

}

//Verificar combinacion de controles antes de iniciar para armar y desarmar todo abajo izq (armar) o abajo der (desarmar)
void checarInicio(){

  //Aleron, Elevador, Throttle, Rudder, Aux
    if(Receptor[0]<1200 && Receptor[1] < 1200 && Receptor[2]<1200 && Receptor[3] < 1200 && armed==false)
    {
      armed=true;
      digitalWrite(LED_PIN,HIGH);
      set_Home();
    }

    if(Receptor[0]>1800 && Receptor[1] < 1200 && Receptor[2]<1200 && Receptor[3] > 1800 && armed==true)
    {
      armed=false;
      digitalWrite(LED_PIN,LOW);
    }

}

void sensoresSetup(){
  accelerometro.initialize();
  //giroscopio.initialize();
  gyro.init();
  magnetometro.initialize();
  barometro.initialize();

 Serial.println();
 Serial.println(accelerometro.testConnection() ? "Accelerometro conectado" : "Accelerometro error");
 //Serial.println(giroscopio.testConnection() ? "Giroscopio conectado" : "Giroscopio error");
 Serial.println(magnetometro.testConnection() ? "Magnetometro conectado" : "Magnetometro error");
 Serial.println(barometro.testConnection() ? "Barometro conectado" : "Barometro error");


 //Data_Format = 0 0 0 0 1 0 1 1
  accelerometro.setSelfTestEnabled(false);//no self test
  accelerometro.setSPIMode(false); //3wire SPI
  accelerometro.setInterruptMode(false); //interrupts en alto
  accelerometro.setFullResolution(true); //4mg/LSB
  accelerometro.setDataJustification(false); //right
  accelerometro.setRange(3); //+-16g
 //Power_CTL = 0 0 0 0 1 0 0 0
  accelerometro.setLinkEnabled(false);//sin delay entre inactivity
  accelerometro.setAutoSleepEnabled(false);//no autosleep
  accelerometro.setMeasureEnabled(true);//modo tomar mediciones
  accelerometro.setSleepEnabled(false);//no sleep
  accelerometro.setWakeupFrequency(0);//8hz
 //BW_RATE = 0 0 0 0 1 0 0 1
  accelerometro.setLowPowerEnabled(false);
  accelerometro.setRate(9);//50hz output 25hz bandwidth

 /*
 //CTRL_REG1 1 0 0 0 1 1 1 1 =0x8F
 giroscopio.setOutputDataRate(200);//Data Rate 200
 // el 1111 es power on y XYZ set, default en initialize
 //CTRL_REG2 0 0 1 1 0 1 0 0 =0x34
 giroscopio.setHighPassMode(3);//11=3 autoreset high pass filter tabla datasheet
 giroscopio.setHighPassFilterCutOffFrequencyLevel(4);//0100=4 cuttoff level tabla datasheet
 //CTRL_REG4 0 0 1 1 0 0 0 0 =0x30
 giroscopio.setFullScale(2000);//full scale 2000dps
 //CTRL_REG5 0 0 0 1 0 0 1 1 =0x13
 giroscopio.setHighPassFilterEnabled(true);//High pass enable
 giroscopio.setDataFilter(L3G4200D_LOW_HIGH_PASS);//out_sel=11 de tabla para pasar por lpf y hpf

 */

 gyro.enableDefault();
 gyro.writeReg(L3G_CTRL_REG1,0x8F);
 gyro.writeReg(L3G_CTRL_REG2,0x34);
 gyro.writeReg(L3G_CTRL_REG4,0x30);
 gyro.writeReg(L3G_CTRL_REG5,0x13);

}

void calcAngulos()
{
  static unsigned long prevt;
  unsigned long t =millis();
  float dt =(float)(t-prevt)/1000;
  prevt=t;

  float alpha=Comp_Filter/(Comp_Filter+dt);

  angle[0]=alpha*(angle[0]-angvel_Gyro[1]*dt)+(1-alpha)*angle_Acc[0];
  angle[1]=alpha*(angle[1]+angvel_Gyro[0]*dt)+(1-alpha)*angle_Acc[1];
  angle[2]=alpha*(angle[2]-angvel_Gyro[2]*dt)+(1-alpha)*angle_Mag;
}

int cmpfunc (const void * a, const void * b)
{
   return ( *(int*)a - *(int*)b );
}

void calcDistancias()
{
    static unsigned long prevt;

  if(sonar_flag=true)
  {

    sonar_flag=false;
    qsort(sonar_PW,5,sizeof(int),cmpfunc);
    alt_Sonar = (float)sonar_PW[2]*0.0254/147; //147us = 2.54cm = 0.0254m
    alt_Sonar = -alt_Sonar*cos(angle_Acc[2]);

    altura=constrain(alt_Sonar,0.16,6);
    
  }

  if(millis()-prevt > 100)
    {
      altura_Rate=(altura-alturaPrev)*10;
      alturaPrev=altura;
      prevt=millis();
    }

/*
  if(alt_Sonar < Alt_MaxSonar)
  {
    altura=alt_Sonar;
  }
  else
  {
    altura=alt_Baro;
  }*/
    
}

//cada 20ms (50Hz)
void readAcc()
{
  int ax,ay,az;//acceleraciones xyz
  static float ax_f,ay_f,az_f;//acceleraciones filtradas

  accelerometro.getAcceleration(&ax, &ay, &az);//leer aceleraciones xyz

  //Pasarlas por Filtro
  ax_f=Acc_Filter*ax_f+(1-Acc_Filter)*(float)ax;
  ay_f=Acc_Filter*ay_f+(1-Acc_Filter)*(float)ay;
  az_f=Acc_Filter*az_f+(1-Acc_Filter)*(float)az;

  //calcular el angulo
  angle_Acc[0]= -atan2(ax_f,sqrt(ay_f*ay_f + az_f*az_f))*180/PI;
  angle_Acc[1]= -atan2(ay_f,sqrt(ax_f*ax_f + az_f*az_f))*180/PI;
  angle_Acc[2]= -atan2(sqrt(ax_f*ax_f + ay_f*ay_f),az_f);//ang a z en rad

}

//cada 1300us (800hz)
void readGyro()
{

  int gx_raw,gy_raw,gz_raw;//vel angular x,y,z raw
  float gx,gy,gz;//vel angular ya con factor escala 0.7
  static float gx_prev,gy_prev,gz_prev;//valores previos de vel angulares

  gyro.read();

 /*

  giroscopio.getAngularVelocity(&gx_raw, &gy_raw, &gz_raw);
  gx=(float)gx_raw*0.07;
  gy=(float)gy_raw*0.07;
  gz=(float)gz_raw*0.07;
  */

  gx= gyro.g.x * 0.07 - 0.2629;
  gy= gyro.g.y * 0.07 - 2.3964;
  gz= gyro.g.z * 0.07 + 0.4858;

  angvel_Gyro[0]=(gx+gx_prev)/2;
  angvel_Gyro[1]=(gy+gy_prev)/2;
  angvel_Gyro[2]=(gz+gz_prev)/2;

  gx_prev=gx;
  gy_prev=gy;
  gz_prev=gz;
}

//15Hz cada 60ms
float readMag()
{
  int mx,my,mz;
  static float direccion_MagPrev;
  magnetometro.getHeading(&mx,&my,&mz);//valores de vector magnetometro

  float direccion_Mag = atan2(my,mx); //obtener angulo de direccion plano xy

  float absAng_Mag;
  static int revoluciones;

  if(direccion_Mag<0)
      direccion_Mag+=2*M_PI;//cambiar de -180/180 a 0/360

  direccion_Mag=direccion_Mag*180/M_PI;//pasar a grados


  if((direccion_Mag-direccion_MagPrev)<-180)//checar si paso mas de una vuelta y eliminar discontinuidad de angulo
      revoluciones++;//sumar una vuelta
  if((direccion_Mag-direccion_MagPrev)>180)//checar si paso mas de una vuelta y eliminar discontinuidad de angulo
      revoluciones--;//restar una vuelta

  absAng_Mag=direccion_Mag + revoluciones*360;

  direccion_MagPrev=direccion_Mag;
  return absAng_Mag;
}

void readTemp()
{
  temp_Baro[1] = barometro.getTemperatureC();
  barometro.setControl(BMP085_MODE_PRESSURE_3);
}

//Cada 23.5ms maximo
void readBaro()
{

    float pressure = barometro.getPressure();
    alt_Baro = barometro.getAltitude(pressure)-barometro.getAltitude(ground_Baro);

    //alt_Baro=153.8462*(temp_Baro[0]+273.15)*(1-exp(0.190259*log(pressure/ground_Baro)));
    //alt_Baro=log(ground_Baro/pressure) * temp_Baro[0] * 29.271267f;
                                              

   barometro.setControl(BMP085_MODE_TEMPERATURE);

}

void set_Home()
{
  //Parametros de suelo del barometro
   barometro.setControl(BMP085_MODE_TEMPERATURE);
   delay(5);
   temp_Baro[0]=barometro.getTemperatureC();//Temperatura en el suelo en centigrados
   barometro.setControl(BMP085_MODE_PRESSURE_3);
   delay(24);
   ground_Baro=barometro.getPressure();
   barometro.setControl(BMP085_MODE_TEMPERATURE);

   angle_heading=readMag();//Heading inicial de magnetometro

}

void pinModeSetup()
{

  pinMode(MOTOR0_PIN,OUTPUT);
  pinMode(MOTOR1_PIN,OUTPUT);
  pinMode(MOTOR2_PIN,OUTPUT);
  pinMode(MOTOR3_PIN,OUTPUT);
  pinMode(LED_PIN,OUTPUT);
  pinMode(Aleron_PIN,INPUT_PULLUP);
  pinMode(Elevador_PIN,INPUT_PULLUP);
  pinMode(Throttle_PIN,INPUT_PULLUP);
  pinMode(Rudder_PIN,INPUT_PULLUP);
  pinMode(Aux_PIN,INPUT_PULLUP);
  pinMode(Sonar_PIN,INPUT_PULLUP);

  analogWrite(MOTOR0_PIN,0);
  analogWrite(MOTOR1_PIN,0);
  analogWrite(MOTOR2_PIN,0);
  analogWrite(MOTOR3_PIN,0);
}

 

 void RXInterruptSetup()
 {
   //Habilitar pc interrupts 0, 4 para Aleron y Rudder
   PCICR  |=  (1 << PCIE0);
   PCMSK0 |=  (1 << PCINT0) | (1 << PCINT4);
   //Habilitar pc interrupt 20, 23 para Elev y Aux , y 21 para sonar
   PCICR  |=  (1 << PCIE2);
   PCMSK2 |=  (1 << PCINT20)| (1 << PCINT23)| (1 << PCINT21);
   //Habilitar interrupt externo Int0
   attachInterrupt(0,ThrottleISR,CHANGE);
   sei();
 }

 //Handle de interrupt externo Int0
 void ThrottleISR()
 {
   if(ReceptorPrevVal[2]==0 && digitalRead(Throttle_PIN))
   {
     //Flanco de subida
     ReceptorPrevTim[2]=micros();
     ReceptorPrevVal[2]=digitalRead(Throttle_PIN);
   }
   else if(ReceptorPrevVal[2]==1 && !digitalRead(Throttle_PIN))
   {
     //Flanco de bajada
     Receptor[2]=micros()-ReceptorPrevTim[2];
     ReceptorPrevVal[2]=digitalRead(Throttle_PIN);
   }

 }

 //Handle de pc interrupts 0, 4 para Aleron, y Rudder
 ISR(PCINT0_vect)
 {
   int i=6;
   int temp=0;

   if(ReceptorPrevVal[0]!=digitalRead(Aleron_PIN))
   {
     i=0;
     temp=digitalRead(Aleron_PIN);
   }
   else if(ReceptorPrevVal[3]!=digitalRead(Rudder_PIN))
   {
      i=3;
     temp=digitalRead(Rudder_PIN);
   }
   
   if(ReceptorPrevVal[i]==0 && temp)
   {
     //Flanco de subida
     ReceptorPrevTim[i]=micros();
     ReceptorPrevVal[i]=temp;
   }
   else if(ReceptorPrevVal[i]==1 && !temp)
   {
     //Flanco de bajada
     Receptor[i]=micros()-ReceptorPrevTim[i];
     ReceptorPrevVal[i]=temp;
   }

 }

  //Handle pc interrupt 23 para Aux y 20 para elev, tambien 21 para sonar
 ISR(PCINT2_vect)
 {
   int i=6;
   int temp=0;

   if(ReceptorPrevVal[1]!=digitalRead(Elevador_PIN))
   {
     i=1;
     temp=digitalRead(Elevador_PIN);
   }
   if(ReceptorPrevVal[4]!=digitalRead(Aux_PIN))
   {
      i=4;
     temp=digitalRead(Aux_PIN);
   }
   
   if(sonar_PrevVal != digitalRead(Sonar_PIN))
   {

     if(sonar_PrevVal==0 && digitalRead(Sonar_PIN))
     {
     //Flanco de subida
     sonar_PrevTim=micros();
     }
     else if(sonar_PrevVal==1 && !digitalRead(Sonar_PIN))
     {
     //Flanco de bajada
     sonar_PW[sonar_i]=micros()-sonar_PrevTim;

     sonar_i++;
     
       if(sonar_i > 4) 
        {
          sonar_i=0;
          sonar_flag=true;
        }
     }
      sonar_PrevVal=digitalRead(Sonar_PIN);

   }
   
   if(ReceptorPrevVal[i]==0 && temp)
   {
     //Flanco de subida
     ReceptorPrevTim[i]=micros();
     ReceptorPrevVal[i]=temp;
   }
   else if(ReceptorPrevVal[i]==1 && !temp)
   {
     //Flanco de bajada
     Receptor[i]=micros()-ReceptorPrevTim[i];
     ReceptorPrevVal[i]=temp;
   }

 }

 //Pasar parametros de PID
void PIDSetup()
{
  PID_roll.setup_PID  (ROLL_PID_KP,  ROLL_PID_KI,  ROLL_PID_KD,  ROLL_PID_MAX,  ROLL_PID_MAX_I);
  PID_pitch.setup_PID (PITCH_PID_KP, PITCH_PID_KI, PITCH_PID_KD, PITCH_PID_MAX, PITCH_PID_MAX_I);
  PID_yaw.setup_PID   (YAW_PID_KP,   YAW_PID_KI,   YAW_PID_KD,   YAW_PID_MAX,   PITCH_PID_MAX_I);
  PID_angleX.setup_PID(ANGLEX_KP,    ANGLEX_KI,    ANGLEX_KD,    ANGLEX_MAX,    ANGLEX_MAX_I);
  PID_angleY.setup_PID(ANGLEY_KP,    ANGLEY_KI,    ANGLEY_KD,    ANGLEY_MAX,    ANGLEY_MAX_I);
  PID_angleZ.setup_PID(ANGLEZ_KP,    ANGLEZ_KI,    ANGLEZ_KD,    ANGLEZ_MAX,    ANGLEZ_MAX_I);
  PID_alt.setup_PID   (ALT_KP,       ALT_KI,       ALT_KD,       ALT_MAX,       ALT_MAX_I);
  PID_altRate.setup_PID(ALTRATE_KP,  ALTRATE_KI,   ALTRATE_KD,   ALTRATE_MAX,   ALTRATE_MAX_I);

  PID_roll.calc_LPF_D(100,20);
  PID_pitch.calc_LPF_D(100,20);
  PID_yaw.calc_LPF_D(100,20);
  PID_angleX.calc_LPF_D(100,20);
  PID_angleY.calc_LPF_D(100,20);
  PID_angleZ.calc_LPF_D(100,20);
  PID_alt.calc_LPF_D(100,20);
  PID_altRate.calc_LPF_D(100,20);
}


void Estabilizar()
{

  int throttle=map(Receptor[2],1000,2000,125,254);//Mapeo thr, cambio cuando haya alt hold

   if(Receptor[4]<1700)
    {
      throttle=AltHold();//alt hold cuando switch este en posicion
    }
    else
    {
      setAlt=altura;//poner altura en setpoint para cuando cambie el switch
      PID_alt.reset_I();
      PID_altRate.reset_I();
    }

  // Esquema de PIDs

     /*                     Rate---------|
       Angulo----------|                 |--------Salida
                       |---SetpointRate--|
       SetpointAngulo--|
      */

  //Obtener Setpoint Angulo
    setX=map(Receptor[0],1000,2000,-35,35); //Roll
    setY=map(Receptor[1],1000,2000,-35,35);//Pitch
    setZ=map(Receptor[3],1000,2000,-70,70);//Yaw

  //PID de angulo, entra angulo sale SetpointRate
   setRateX=PID_angleX.get_PID((float)setX+angle[0]);
   setRateY=PID_angleY.get_PID((float)setY-angle[1]);
  
  if(abs(setZ)>10)//Moviendo el yaw manualmente
  {
    setRateZ=(float)setZ; //poner el rate directo desde el control
    angle_heading=angle[2]; //poner el nuevo angulo deseado
    PID_angleZ.reset_I(); //resetear I para cuando se deje de mover
  }
  else
  {
    setRateZ=PID_angleZ.get_PID(angle[2]-(float)angle_heading); //ajustar rate para ir al ultimo angulo
  }

   
  //segundo PID, pid de rate, con rate deseados se calculan los pid´s para los 3 ejes
  int PIDroll_val=  (int)PID_roll.get_PID (setRateX-angvel_Gyro[1]);
  int PIDpitch_val= (int)PID_pitch.get_PID(setRateY-angvel_Gyro[0]);
  int PIDyaw_val=   (int)PID_yaw.get_PID  (setRateZ-angvel_Gyro[2]);

  //Pasarlos a motores
  MotoresPWM[0]=throttle+PIDroll_val-PIDpitch_val -PIDyaw_val;
  MotoresPWM[1]=throttle-PIDroll_val-PIDpitch_val +PIDyaw_val;
  MotoresPWM[2]=throttle-PIDroll_val+PIDpitch_val -PIDyaw_val;
  MotoresPWM[3]=throttle+PIDroll_val+PIDpitch_val +PIDyaw_val;


  //125 -> 250 =  1000 -> 2000
  MotoresPWM[0]=constrain(MotoresPWM[0],125,250);
  MotoresPWM[1]=constrain(MotoresPWM[1],125,250);
  MotoresPWM[2]=constrain(MotoresPWM[2],125,250);
  MotoresPWM[3]=constrain(MotoresPWM[3],125,250);

}

int AltHold()
{
  int throttle;
  float throttle_Rate;

    if(Receptor[2]>1400 && Receptor[2]<1600)//en el centro mantener altura
    {
      throttle_Rate = PID_altRate.get_PID(setAlt-altura);
    }
    else//subiendo o bajando conforme al stick
    {
      throttle_Rate = map(Receptor[2],1000,2000,-2.5,2.5);//maxima vel de 2.5m/s
      setAlt=altura;//nuevo setpoint de altura
      PID_altRate.reset_I();//no se usa altrate, se toma directo del rc
    }

  throttle =(int)PID_alt.get_PID(throttle_Rate-altura_Rate);
  throttle = map(throttle,-100,100,125,254); //125->254

  return throttle;
}

void writeMotors()
{
  analogWrite(MOTOR0_PIN,MotoresPWM[0]);
  analogWrite(MOTOR1_PIN,MotoresPWM[1]);
  analogWrite(MOTOR2_PIN,MotoresPWM[2]);
  analogWrite(MOTOR3_PIN,MotoresPWM[3]);
}

void apagarMotores()
{
    PID_roll.reset_I();
    PID_pitch.reset_I();
    PID_yaw.reset_I();
    PID_angleX.reset_I();
    PID_angleY.reset_I();
    PID_angleZ.reset_I();

    MotoresPWM[0]=120;
    MotoresPWM[1]=120;
    MotoresPWM[2]=120;
    MotoresPWM[3]=120;
}

void DebugSerial()
{


// Usados por Processing en ese orden Ax,Ay,Az, Al,El,Th,Ru,Au
  Serial.print(" AngX: ");
  Serial.print(angle[0]);
  Serial.print(" AngY: ");
  Serial.print(angle[1]);
  Serial.print(" AngZ: ");
  Serial.print(angle[2]);

  Serial.print(" Aileron: ");
  Serial.print(Receptor[0]);
  Serial.print(" Elevador: ");
  Serial.print(Receptor[1]);
  Serial.print(" Thruttle: ");
  Serial.print(Receptor[2]);
  Serial.print(" Rudder: ");
  Serial.print(Receptor[3]);
  Serial.print(" Aux: ");
  Serial.print(Receptor[4]);

  Serial.print(" M_Uno: ");
  Serial.print(MotoresPWM[0]);
  Serial.print(" M_Dos: ");
  Serial.print(MotoresPWM[1]);
  Serial.print(" M_Tres: ");
  Serial.print(MotoresPWM[2]);
  Serial.print(" M_Cuatro: ");
  Serial.print(MotoresPWM[3]);

  Serial.print(" Altura: ");
  Serial.print(altura);

// */


//


//

/*
  Serial.print(" AngAccX: ");
  Serial.print(angle_Acc[0]);
  Serial.print(" AngAccY: ");
  Serial.print(angle_Acc[1]);
  Serial.print(" AngRateX: ");
  Serial.print(angvel_Gyro[0]);
  Serial.print(" AngRateY: ");
  Serial.print(angvel_Gyro[1]);
  Serial.print(" AngRateZ: ");
  Serial.print(angvel_Gyro[2]);
*/


// */


  //Serial.print(" Mag: ");
  //Serial.print(readMag());
  //Serial.print(" Baro: ");
  //Serial.print(alt_Baro);

 // * 

Serial.println();
 }
