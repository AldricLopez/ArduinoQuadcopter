
 /* Configuracion de motores

  M1/CW     M2/CCW    Adelante
        \ /             /\
        / \             ||
  M4/CCW    M3/CW

 */

 // Pines
   #define MOTOR0_PIN     3   //PWM OC2B
   #define MOTOR1_PIN     9   //PWM OC1A
   #define MOTOR2_PIN     10   //PWM OC1B
   #define MOTOR3_PIN     11  //PWM OC2A

   #define LED_PIN        13  //convencion arduino

   #define Aleron_PIN     8   //PCINT0
   #define Elevador_PIN   4  //PCINT20
   #define Throttle_PIN   2   //INT0
   #define Rudder_PIN     12  //PCINT4
   #define Aux_PIN        7  //PCINT23

   #define Sonar_PIN      5  //PCINT21

//Filtros
   #define Acc_Filter 0   //Filtro para el acc (siempre no se uso pero por si se necesita esta aqui)
   #define Comp_Filter 1  //constante T del filtro alpha=T/(T+dt), T=tiempo en segundos

//cambio de alturas
  #define Alt_MaxSonar 6 //Maxima altura confiable de sonar

//EEPROM
  #define memBase 100 //Inicio de memoria para guardar los datos
  #define numDataEEPROM 40//5 por pid * 8 pid´s
  #define maxWrites 5//Maximo de veces que se puede reescribir sin marcar error





