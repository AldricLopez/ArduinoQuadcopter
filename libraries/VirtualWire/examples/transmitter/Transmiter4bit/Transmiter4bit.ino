#include <VirtualWire.h>//Incluye la libreria, primero copia el folder del zip que te mande en tu carpeta de arduino/libraries

void setupTransmisor(void); // solo pon esta en el setup

void Multicontactos(int,int,int,int);//para cada int 1=prendido 0=apagado

void setup()
{
  //Blah blah todo lo tuyo
  setupTransmisor();
}

void loop()
{
  int a,b,c,d;
  //Blah blah todo lo tuyo
  
  Multicontactos(a,b,c,d);
}

void setupTransmisor(void)
{
  vw_set_tx_pin(9); // usar el pin 9 para transmitir
  vw_setup(2000); // la transmision mas rapida y mas estable que se puede es 2000 bps
}

void Multicontactos(int a, int b, int c, int d)
{
  int num;//Codigo premensaje
  char BITS[6]; //Mensaje a transmitir
  
  
  num=10000 + 1000*a + 100*b + 10*c + d;//codificar premensaje
  itoa(num,BITS,10);//codificado a arreglo
  char *msg=BITS;//convertir el mensaje a apuntador
  
  vw_send((uint8_t *)msg, strlen(msg)); //enviar mensaje
  vw_wait_tx();//esperar a que termine de enviarse el mensaje

}
