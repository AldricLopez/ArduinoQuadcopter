// QuadController_Mega.ino

//defines.h
//Filtros
   #define Acc_Filter 0   //Filtro para el acc (siempre no se uso pero por si se necesita esta aqui)
   #define Comp_Filter 1  //constante T del filtro alpha=T/(T+dt), T=tiempo en segundos


#include <Wire.h>

#define BMA180 0x40 //Address Acc
#define ITG3200 0x68 //Address Gyro (0x69)
#define HMC5883L 0x1E//Address Acc

#include "I2Cdev.h"
#include "BMP085.h"
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



// Declaraciones

void sensoresSetup();

void calcAngulos();

void readAcc();
void readGyro();
float readMag();
void readBaro();
void readTemp();
void DebugSerial();

void set_Home();

void writeTo(int DEVICE, byte address, byte val);
void readFrom(int DEVICE, byte address , int num ,byte buff[]);

void setup() {
   Serial.begin(9600);
   Wire.begin();
   
   delay(100);
   
   sensoresSetup();

   set_Home();
}

void loop() {
	
	//Leer los sensores y calcular angulo/////////////////////////////////////////

    if(millis()-prevMillisAcc > 50)
    {
      readAcc();//cada 50 ms
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
    if((millis()-prevMillisBaro > 25) && (tempNbaro==false))
    {
      readBaro();//cada 25 ms
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

	if(millis()-prevMillisSerial > 100)
    { 
      DebugSerial(); //Envia datos de sensores por serial
      prevMillisSerial=millis();
    }

}


void sensoresSetup(){


	//Setup ACC
	byte temp[1];
	byte temp1;
	writeTo(BMA180,0x10,0xB6);//Reset
	writeTo(BMA180,0x0D,0x10);//Power enable write
	readFrom(BMA180, 0x20,1,temp);//BandWidth=7,6,5,4 tcs=3,2,1,0   
	temp1=(temp[0]&0x0F)| 0x01; //Mask 0001 XXXX -> 0000=10Hz, 0001=20Hz, 0010=40Hz, , 0011=75Hz
	writeTo(BMA180, 0x20, temp1);   
	readFrom(BMA180, 0x35, 1 ,temp);// offxlsb=7,6,5,4  range=3,2,1 smp_skip=0
	temp1=(temp[0]&0xF1) | 0x0C;//Mask  0x0C=XXXX 110X  -> 110=+-16g
	writeTo(BMA180,0x35,temp1);

	//Setup Gyro
	//sample rate 800hz, +-2000dgr/sec, lpf= 256hz
	writeTo(ITG3200, 0x3E, 0x00); //HardReset=0,sleep=0,standby x=0,y=0,z=0, clksel=000 -> internal
	writeTo(ITG3200, 0x15, 0x09); // 8kHz/(N+1)=800Hz -> N=0x09
	writeTo(ITG3200, 0x16, 0x18); // +/- 2000 dgrs/sec, 8kHz
	writeTo(ITG3200, 0x17, 0x00); //no interrupts
	//Setup Mag
  writeTo(HMC5883L, 0x00, 0x70);//Config_A=0x00, 0x70=0111 0000 .: 6,5=11->8 samples; 4,3,2=100->15 output rate bits; 1,0=00->normal bias 
  writeTo(HMC5883L, 0x01, 0x20);//Config_B=0x01, 0x20=0010 0000 .: 7,6,5->1090[LSb/Gauss];   
  writeTo(HMC5883L, 0x02, 0x01);//MODE=0x02, 0x01 .: 1,0=01->single measurment

 	//SetupBaro
  barometro.initialize();

}


void calcAngulos()
{
  static unsigned long prevt;
  unsigned long t =millis();
  float dt =(float)(t-prevt)/1000;
  prevt=t;

  float alpha=Comp_Filter/(Comp_Filter+dt);

  angle[0]=alpha*(angle[0]+angvel_Gyro[0]*dt)+(1-alpha)*angle_Acc[0];
  angle[1]=alpha*(angle[1]+angvel_Gyro[1]*dt)+(1-alpha)*angle_Acc[1];
  angle[2]=alpha*(angle[2]+angvel_Gyro[2]*dt)+(1-alpha)*angle_Mag;
}


//cada 50ms (20Hz)
void readAcc()
{
  int ax,ay,az;//acceleraciones xyz
  static float ax_f,ay_f,az_f;//acceleraciones filtradas

  int offx = 257;  
  int offy = -26;   
  int offz = 64;

 // read in the 3 axis data, each one is 14 bits 
 byte result[5];
 readFrom(BMA180, 0x02, 6 , result);//read 3axis*2bytes=6
 
 ax= (( result[0] | result[1]<<8)>>2)+offx ; // (a/512)*9.81 => m/s^2
 ay= (( result[2] | result[3]<<8 )>>2)+offy;
 az= (( result[4] | result[5]<<8 )>>2)+offz;
 

  //Pasarlas por Filtro
  ax_f=Acc_Filter*ax_f+(1-Acc_Filter)*(float)ax;
  ay_f=Acc_Filter*ay_f+(1-Acc_Filter)*(float)ay;
  az_f=Acc_Filter*az_f+(1-Acc_Filter)*(float)az;

  //calcular el angulo
  angle_Acc[0]= atan2(ay_f,sqrt(ax_f*ax_f + az_f*az_f))*180/PI;
  angle_Acc[1]= atan2(ax_f,sqrt(ay_f*ay_f + az_f*az_f))*180/PI;
  angle_Acc[2]= ((float)az/512)*9.81;//-atan2(sqrt(ax_f*ax_f + ay_f*ay_f),az_f);//ang a z en rad

}

//cada 1300us (800hz)
void readGyro()
{

  int gx_raw,gy_raw,gz_raw,temp_raw;//vel angular x,y,z raw
  float gx,gy,gz;//vel angular ya con factor escala 
  static float gx_prev,gy_prev,gz_prev;//valores previos de vel angulares

	int g_offx = -6;
	int g_offy = 2;
	int g_offz = 6;

	byte buff[8];//2 bytes por axis
	readFrom(ITG3200, 0x1B, 8, buff); //read the gyro data 1B->22
	gx_raw = ((buff[2] << 8) | buff[3]) + g_offx; //x
	gy_raw = ((buff[4] << 8) | buff[5]) + g_offy; //y
	gz_raw = ((buff[6] << 8) | buff[7]) + g_offz; //z
	temp_raw = (buff[0] << 8) | buff[1]; // temperature


  gx= gx_raw/14.345;
  gy= -gy_raw/14.345;
  gz= gz_raw/14.345;

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
    
    byte buff[6];//2 bytes por axis
    readFrom(HMC5883L, 0x03, 6, buff); //XH=0x03, 2*3=6
    mx = ((buff[0]) << 8) | buff[1];
    my = ((buff[4]) << 8) | buff[5];
    mz = ((buff[2]) << 8) | buff[3];
    writeTo(HMC5883L, 0x02, 0x01); //MODE=0x02, 0x01 .: 1,0=01->single measurment



  float direccion_Mag = -atan2(my,mx); //obtener angulo de direccion plano xy

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

//Cada 25.5ms maximo
void readBaro()
{
    float pressure = barometro.getPressure();
    alt_Baro = barometro.getAltitude(pressure)-ground_Baro;

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
   delay(25);
   float pressure=barometro.getPressure();
   ground_Baro=barometro.getAltitude(pressure);
   barometro.setControl(BMP085_MODE_TEMPERATURE);

   angle_heading=readMag();//Heading inicial de magnetometro

}

//Writes val to address register on ACC
void writeTo(int DEVICE, byte address, byte val) 
{
  Wire.beginTransmission(DEVICE);   //start transmission 
  Wire.write(address);               //send register address
  Wire.write(val);                   //send value to write
  Wire.endTransmission();           //end trnsmisson
}

//reads num bytes starting from address register in to buff array
void readFrom(int DEVICE, byte address , int num ,byte buff[])
{
 Wire.beginTransmission(DEVICE); //start transmission 
 Wire.write(address);            //send register address
 Wire.endTransmission();        //end transmission
 
 Wire.beginTransmission(DEVICE); //start transmission to ACC
 Wire.requestFrom(DEVICE,num);  //request num bits
 
 int i=0;
 while(Wire.available())        // may abnormal
 {
 buff[i] =Wire.read();        //receive a byte
 i++;
 }
 Wire.endTransmission();         //end transmission
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

  // Serial.print(" Aileron: ");
  // Serial.print(Receptor[0]);
  // Serial.print(" Elevador: ");
  // Serial.print(Receptor[1]);
  // Serial.print(" Thruttle: ");
  // Serial.print(Receptor[2]);
  // Serial.print(" Rudder: ");
  // Serial.print(Receptor[3]);
  // Serial.print(" Aux: ");
  // Serial.print(Receptor[4]);

  // Serial.print(" M_Uno: ");
  // Serial.print(MotoresPWM[0]);
  // Serial.print(" M_Dos: ");
  // Serial.print(MotoresPWM[1]);
  // Serial.print(" M_Tres: ");
  // Serial.print(MotoresPWM[2]);
  // Serial.print(" M_Cuatro: ");
  // Serial.print(MotoresPWM[3]);

  // Serial.print(" Altura: ");
  // Serial.print(altura);

// */


//


//


  // Serial.print(" AngAccX: ");
  // Serial.print(angle_Acc[0]);
  // Serial.print(" AngAccY: ");
  // Serial.print(angle_Acc[1]);
  // Serial.print(" AngAccZ: ");
  // Serial.print(angle_Acc[2]);
  // Serial.print(" AngRateX: ");
  // Serial.print(angvel_Gyro[0]);
  // Serial.print(" AngRateY: ");
  // Serial.print(angvel_Gyro[1]);
  // Serial.print(" AngRateZ: ");
  // Serial.print(angvel_Gyro[2]);




  // Serial.print(" Mag: ");
  // Serial.print(readMag());
  // Serial.print(" Baro: ");
  // Serial.print(alt_Baro);

Serial.println();
 }