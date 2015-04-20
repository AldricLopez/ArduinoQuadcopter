import controlP5.*;
import processing.serial.*;
import processing.opengl.*;

ControlP5 cp5;

final int Width =1500;
final int Height =900;
final int SquareWidth = 200;
final int SquareHeight = 200;
final int CompassSize = 150;
final int datosRecibidos =13;
final int sendEEPROM= 40;

Serial myPort;  // Create object from Serial class
String SerialDataString=null;
String[] SerialNumsStrings= new String[datosRecibidos+1];
float[] SerialVals= new float[datosRecibidos];//Ax,Ay,Az, Al,El,Th,Ru,Au,m1,m2,m3,m4,alt



float x1,y1,x2,y2,yaux;

void setup() 
{
  size(Width, Height, P3D);
  
  cp5 = new ControlP5(this);
  addControllers();
  
  String portName = Serial.list()[0];
  myPort = new Serial(this, portName, 9600);
  myPort.clear();
 
}

void draw()
{
  
   ReadSerialData();
 
   background(30);
   
   drawGUI();

}

void ReadSerialData()
{
  if( myPort.available() > 0) {  
    SerialDataString=myPort.readStringUntil(10);//leer hasta un newline=10
    if(SerialDataString!=null)
    {
      SerialDataString = SerialDataString.replaceAll("[^0-9|.|-]+", " ");
      SerialNumsStrings = split(SerialDataString," ");
      if((SerialNumsStrings.length)>datosRecibidos)
      {
        for(int i=0; i<datosRecibidos; i++)
        {
        SerialVals[i]=parseFloat(SerialNumsStrings[i+1]);
        //print(SerialVals[i]);
        //print(" ");
        }
        //println();
      }
      
    }
  }
}


void drawGUI()
{
  
  textSize(20);
  fill(255,255,255);
  text("Interfaz PC para Quad",Width/2-100,50);
  
  //Dibuja la caja simulando las rotaciones
   fill(200,200,200);
   pushMatrix();
   translate(Width/2, Height/2, 0);
   rotateX(SerialVals[1]*PI/180);
   rotateY(0);//SerialVals[2]*PI/180);
   rotateZ(-SerialVals[0]*PI/180);
   box(80,40,100);
   popMatrix();
   
   //Dibuja Brujula
   textSize(15);
   fill(255,255,255);
   text("N",Width-CompassSize-10,Height-CompassSize*1.5-10);
   fill(200,200,200);
   ellipse(Width-CompassSize,Height-CompassSize,CompassSize,CompassSize);
   fill(0,0,0);
   line(Width-CompassSize, Height-CompassSize, 
        Width-CompassSize+CompassSize*cos(radians(-SerialVals[2]))/2,
        Height-CompassSize+CompassSize*sin(radians(-SerialVals[2]))/2);
   
        
   //Dibuja Altimetro
   textSize(15);
   fill(255,255,255);
   text("Altura: "+SerialVals[12]+"m",Width-CompassSize*2.5,Height-CompassSize*1.5-10);
   
   //Dibuja los cuadros simulando el control remoto
   x1=Width/2-50-SquareWidth+map(SerialVals[6],1000,2000,0,SquareWidth);
   y1=Height-50-map(SerialVals[5],1000,2000,0,SquareHeight);
   x2=Width/2+50+map(SerialVals[3],1000,2000,0,SquareWidth);
   y2=Height-50-map(SerialVals[4],1000,2000,0,SquareHeight);
   yaux=Height-SquareHeight-50+map(SerialVals[7],1000,2000,40,0);
   
   fill(160,160,160);
   rect(Width/2-SquareWidth-50, Height-SquareHeight-50, SquareWidth,SquareHeight);
   rect(Width/2+50            , Height-SquareHeight-50, SquareWidth,SquareHeight);
   rect(Width/2+80+SquareWidth, Height-SquareHeight-50, 20,60);
   
   fill(0,0,0);
   ellipse(x1,y1,20,20);
   ellipse(x2,y2,20,20);
   rect(Width/2+80+SquareWidth, yaux, 20,20);
   
   
   //Dibuja Rectangulos simulando los motores  motor:(125-250)
   
   fill(200,10,10);
   rect(Width-50, Height/2,20,100);
   rect(Width-200, Height/2,20,100);
   rect(Width-50, Height/2-150,20,100);
   rect(Width-200, Height/2-150,20,100);
   
   fill(100,100,100);
   rect(Width-200, Height/2-150,20,constrain(map(SerialVals[8],120,250,100,0),0,100));
   rect(Width-50, Height/2-150,20,constrain(map(SerialVals[9],120,250,100,0),0,100));
   rect(Width-50, Height/2,20,constrain(map(SerialVals[10],120,250,100,0),0,100));
   rect(Width-200, Height/2,20,constrain(map(SerialVals[11],120,250,100,0),0,100));

 

}

void addControllers()
{
  PFont p = createFont("Verdana",12); 
  cp5.setControlFont(p);
  
  cp5.addNumberbox("Roll_KP")
     .setPosition(150,50)
     .setSize(60,20)
     .setScrollSensitivity(0.1)
     .setMultiplier(0.001)
     .setDecimalPrecision(3)
     .setValue(0.20)
     ;
  cp5.addNumberbox("Roll_KI")
     .setPosition(250,50)
     .setSize(60,20)
     .setScrollSensitivity(0.1)
     .setMultiplier(0.001)
     .setDecimalPrecision(3)
     .setValue(0.1)
     ;
   cp5.addNumberbox("Roll_KD")
     .setPosition(350,50)
     .setSize(60,20)
     .setScrollSensitivity(0.1)
     .setMultiplier(.001)
     .setDecimalPrecision(3)
     .setValue(0.004)
     ;
   cp5.addNumberbox("Roll_Max")
     .setPosition(450,50)
     .setSize(60,20)
     .setScrollSensitivity(0.1)
     .setMultiplier(1)
     .setDecimalPrecision(0)
     .setValue(50)
     ;
   cp5.addNumberbox("Roll_Max_I")
     .setPosition(550,50)
     .setSize(60,20)
     .setScrollSensitivity(0.1)
     .setMultiplier(1)
     .setDecimalPrecision(0)
     .setValue(35)
     ;
     cp5.addNumberbox("Pitch_KP")
     .setPosition(150,100)
     .setSize(60,20)
     .setScrollSensitivity(0.1)
     .setMultiplier(0.001)
     .setDecimalPrecision(3)
     .setValue(0.20)
     ;
  cp5.addNumberbox("Pitch_KI")
     .setPosition(250,100)
     .setSize(60,20)
     .setScrollSensitivity(0.1)
     .setMultiplier(0.001)
     .setDecimalPrecision(3)
     .setValue(0.1)
     ;
   cp5.addNumberbox("Pitch_KD")
     .setPosition(350,100)
     .setSize(60,20)
     .setScrollSensitivity(0.1)
     .setMultiplier(.001)
     .setDecimalPrecision(3)
     .setValue(0.004)
     ;
   cp5.addNumberbox("Pitch_Max")
     .setPosition(450,100)
     .setSize(60,20)
     .setScrollSensitivity(0.1)
     .setMultiplier(1)
     .setDecimalPrecision(0)
     .setValue(50)
     ;
   cp5.addNumberbox("Pitch_Max_I")
     .setPosition(550,100)
     .setSize(60,20)
     .setScrollSensitivity(0.1)
     .setMultiplier(1)
     .setDecimalPrecision(0)
     .setValue(35)
     ;
   cp5.addNumberbox("Yaw_KP")
     .setPosition(150,150)
     .setSize(60,20)
     .setScrollSensitivity(0.1)
     .setMultiplier(0.001)
     .setDecimalPrecision(3)
     .setValue(0.60)
     ;
  cp5.addNumberbox("Yaw_KI")
     .setPosition(250,150)
     .setSize(60,20)
     .setScrollSensitivity(0.1)
     .setMultiplier(0.001)
     .setDecimalPrecision(3)
     .setValue(0.02)
     ;
   cp5.addNumberbox("Yaw_KD")
     .setPosition(350,150)
     .setSize(60,20)
     .setScrollSensitivity(0.1)
     .setMultiplier(.001)
     .setDecimalPrecision(3)
     .setValue(0)
     ;
   cp5.addNumberbox("Yaw_Max")
     .setPosition(450,150)
     .setSize(60,20)
     .setScrollSensitivity(0.1)
     .setMultiplier(1)
     .setDecimalPrecision(0)
     .setValue(90)
     ;
   cp5.addNumberbox("Yaw_Max_I")
     .setPosition(550,150)
     .setSize(60,20)
     .setScrollSensitivity(0.1)
     .setMultiplier(1)
     .setDecimalPrecision(0)
     .setValue(40)
     ;
  cp5.addNumberbox("AngX_KP")
     .setPosition(150,200)
     .setSize(60,20)
     .setScrollSensitivity(0.1)
     .setMultiplier(0.01)
     .setDecimalPrecision(3)
     .setValue(4.5)
     ;
  cp5.addNumberbox("AngX_KI")
     .setPosition(250,200)
     .setSize(60,20)
     .setScrollSensitivity(0.1)
     .setMultiplier(0.001)
     .setDecimalPrecision(3)
     .setValue(0)
     ;
   cp5.addNumberbox("AngX_KD")
     .setPosition(350,200)
     .setSize(60,20)
     .setScrollSensitivity(0.1)
     .setMultiplier(.001)
     .setDecimalPrecision(3)
     .setValue(0)
     ;
   cp5.addNumberbox("AngX_Max")
     .setPosition(450,200)
     .setSize(60,20)
     .setScrollSensitivity(0.1)
     .setMultiplier(1)
     .setDecimalPrecision(0)
     .setValue(100)
     ;
   cp5.addNumberbox("AngX_Max_I")
     .setPosition(550,200)
     .setSize(60,20)
     .setScrollSensitivity(0.1)
     .setMultiplier(1)
     .setDecimalPrecision(0)
     .setValue(100)
     ;
  cp5.addNumberbox("AngY_KP")
     .setPosition(150,250)
     .setSize(60,20)
     .setScrollSensitivity(0.1)
     .setMultiplier(0.01)
     .setDecimalPrecision(3)
     .setValue(4.5)
     ;
  cp5.addNumberbox("AngY_KI")
     .setPosition(250,250)
     .setSize(60,20)
     .setScrollSensitivity(0.1)
     .setMultiplier(0.001)
     .setDecimalPrecision(3)
     .setValue(0)
     ;
   cp5.addNumberbox("AngY_KD")
     .setPosition(350,250)
     .setSize(60,20)
     .setScrollSensitivity(0.1)
     .setMultiplier(.001)
     .setDecimalPrecision(3)
     .setValue(0)
     ;
    cp5.addNumberbox("AngY_Max")
     .setPosition(450,250)
     .setSize(60,20)
     .setScrollSensitivity(0.1)
     .setMultiplier(1)
     .setDecimalPrecision(0)
     .setValue(100)
     ;
   cp5.addNumberbox("AngY_Max_I")
     .setPosition(550,250)
     .setSize(60,20)
     .setScrollSensitivity(0.1)
     .setMultiplier(1)
     .setDecimalPrecision(0)
     .setValue(100)
     ;
   cp5.addNumberbox("AngZ_KP")
     .setPosition(150,300)
     .setSize(60,20)
     .setScrollSensitivity(0.1)
     .setMultiplier(0.01)
     .setDecimalPrecision(3)
     .setValue(5)
     ;
  cp5.addNumberbox("AngZ_KI")
     .setPosition(250,300)
     .setSize(60,20)
     .setScrollSensitivity(0.1)
     .setMultiplier(0.001)
     .setDecimalPrecision(3)
     .setValue(0)
     ;
   cp5.addNumberbox("AngZ_KD")
     .setPosition(350,300)
     .setSize(60,20)
     .setScrollSensitivity(0.1)
     .setMultiplier(.001)
     .setDecimalPrecision(3)
     .setValue(0)
     ;
    cp5.addNumberbox("AngZ_Max")
     .setPosition(450,300)
     .setSize(60,20)
     .setScrollSensitivity(0.1)
     .setMultiplier(1)
     .setDecimalPrecision(0)
     .setValue(80)
     ;
   cp5.addNumberbox("AngZ_Max_I")
     .setPosition(550,300)
     .setSize(60,20)
     .setScrollSensitivity(0.1)
     .setMultiplier(1)
     .setDecimalPrecision(0)
     .setValue(50)
     ;
     
  cp5.addNumberbox("Alt_KP")
     .setPosition(150,350)
     .setSize(60,20)
     .setScrollSensitivity(0.1)
     .setMultiplier(0.01)
     .setDecimalPrecision(3)
     .setValue(10)
     ;
  cp5.addNumberbox("Alt_KI")
     .setPosition(250,350)
     .setSize(60,20)
     .setScrollSensitivity(0.1)
     .setMultiplier(0.001)
     .setDecimalPrecision(3)
     .setValue(0)
     ;
   cp5.addNumberbox("Alt_KD")
     .setPosition(350,350)
     .setSize(60,20)
     .setScrollSensitivity(0.1)
     .setMultiplier(.001)
     .setDecimalPrecision(3)
     .setValue(0)
     ;
    cp5.addNumberbox("Alt_Max")
     .setPosition(450,350)
     .setSize(60,20)
     .setScrollSensitivity(0.1)
     .setMultiplier(1)
     .setDecimalPrecision(0)
     .setValue(100)
     ;
   cp5.addNumberbox("Alt_Max_I")
     .setPosition(550,350)
     .setSize(60,20)
     .setScrollSensitivity(0.1)
     .setMultiplier(1)
     .setDecimalPrecision(0)
     .setValue(100)
     ;
     
   cp5.addNumberbox("AltRate_KP")
     .setPosition(150,400)
     .setSize(60,20)
     .setScrollSensitivity(0.1)
     .setMultiplier(0.01)
     .setDecimalPrecision(3)
     .setValue(10)
     ;
  cp5.addNumberbox("AltRate_KI")
     .setPosition(250,400)
     .setSize(60,20)
     .setScrollSensitivity(0.1)
     .setMultiplier(0.001)
     .setDecimalPrecision(3)
     .setValue(0)
     ;
   cp5.addNumberbox("AltRate_KD")
     .setPosition(350,400)
     .setSize(60,20)
     .setScrollSensitivity(0.1)
     .setMultiplier(.001)
     .setDecimalPrecision(3)
     .setValue(0)
     ;
    cp5.addNumberbox("AltRate_Max")
     .setPosition(450,400)
     .setSize(60,20)
     .setScrollSensitivity(0.1)
     .setMultiplier(1)
     .setDecimalPrecision(0)
     .setValue(3
     )
     ;
   cp5.addNumberbox("AltRate_Max_I")
     .setPosition(550,400)
     .setSize(60,20)
     .setScrollSensitivity(0.1)
     .setMultiplier(1)
     .setDecimalPrecision(0)
     .setValue(3)
     ;
     
  cp5.addButton("Enviar",1,50,150,80,20);
  cp5.addButton("Leer"  ,1,50,200,80,20);
  cp5.addButton("Conectar",1,50,50,80,20);
  cp5.addButton("Desconectar",1,50,100,80,20);
}

void controlEvent(ControlEvent theEvent) {
  
  if(theEvent.isController()) { 
    if(theEvent.controller().name()=="Conectar") {
      myPort.write('c');
    }
    if(theEvent.controller().name()=="Desconectar") {
      myPort.write('d');
    }
    if(theEvent.controller().name()=="Enviar") {
       
       myPort.write('s');
       myPort.write(" "+cp5.getController("Roll_KP").getValue()+
                    " "+cp5.getController("Roll_KI").getValue()+
                    " "+cp5.getController("Roll_KD").getValue()+
                    " "+cp5.getController("Roll_Max").getValue()+
                    " "+cp5.getController("Roll_Max_I").getValue()+
                    
                    " "+cp5.getController("Pitch_KP").getValue()+
                    " "+cp5.getController("Pitch_KI").getValue()+
                    " "+cp5.getController("Pitch_KD").getValue()+
                    " "+cp5.getController("Pitch_Max").getValue()+
                    " "+cp5.getController("Pitch_Max_I").getValue()+
                    
                    " "+cp5.getController("Yaw_KP").getValue()+
                    " "+cp5.getController("Yaw_KI").getValue()+
                    " "+cp5.getController("Yaw_KD").getValue()+
                    " "+cp5.getController("Yaw_Max").getValue()+
                    " "+cp5.getController("Yaw_Max_I").getValue()+
                    
                    " "+cp5.getController("AngX_KP").getValue()+
                    " "+cp5.getController("AngX_KI").getValue()+
                    " "+cp5.getController("AngX_KD").getValue()+
                    " "+cp5.getController("AngX_Max").getValue()+
                    " "+cp5.getController("AngX_Max_I").getValue()+
                    
                    " "+cp5.getController("AngY_KP").getValue()+
                    " "+cp5.getController("AngY_KI").getValue()+
                    " "+cp5.getController("AngY_KD").getValue()+
                    " "+cp5.getController("AngY_Max").getValue()+
                    " "+cp5.getController("AngY_Max_I").getValue()+
                    
                    " "+cp5.getController("AngZ_KP").getValue()+
                    " "+cp5.getController("AngZ_KI").getValue()+
                    " "+cp5.getController("AngZ_KD").getValue()+
                    " "+cp5.getController("AngZ_Max").getValue()+
                    " "+cp5.getController("AngZ_Max_I").getValue()+
                    
                    " "+cp5.getController("Alt_KP").getValue()+
                    " "+cp5.getController("Alt_KI").getValue()+
                    " "+cp5.getController("Alt_KD").getValue()+
                    " "+cp5.getController("Alt_Max").getValue()+
                    " "+cp5.getController("Alt_Max_I").getValue()+
                    
                    " "+cp5.getController("AltRate_KP").getValue()+
                    " "+cp5.getController("AltRate_KI").getValue()+
                    " "+cp5.getController("AltRate_KD").getValue()+
                    " "+cp5.getController("AltRate_Max").getValue()+
                    " "+cp5.getController("AltRate_Max_I").getValue()
       );
       println("Valores Enviados");
    }
    
    if(theEvent.controller().name()=="Leer") {
      println("Leyendo Valores...");
      
       myPort.write('r');
       
       delay(500);//Espera a que termine de recibir lo q enviabas antes
       myPort.clear();//Borralo
       delay(1000);//espera a que llegue nueva info
       
       if(myPort.available()>0){
       String tempS=myPort.readStringUntil(10);//leer hasta un newline=10

       String[] tempFS= split(tempS," ");;
       float[] tempF=new float[sendEEPROM];
      
       
       for(int i=0; i<sendEEPROM; i++)
       {
         tempF[i]=parseFloat(tempFS[i]);
       }
       
       cp5.getController("Roll_KP").setValue(tempF[0]);
       cp5.getController("Roll_KI").setValue(tempF[1]);
       cp5.getController("Roll_KD").setValue(tempF[2]);
       cp5.getController("Roll_Max").setValue(tempF[3]);
       cp5.getController("Roll_Max_I").setValue(tempF[4]);
       
       cp5.getController("Pitch_KP").setValue(tempF[5]);
       cp5.getController("Pitch_KI").setValue(tempF[6]);
       cp5.getController("Pitch_KD").setValue(tempF[7]);
       cp5.getController("Pitch_Max").setValue(tempF[8]);
       cp5.getController("Pitch_Max_I").setValue(tempF[9]);
       
       cp5.getController("Yaw_KP").setValue(tempF[10]);
       cp5.getController("Yaw_KI").setValue(tempF[11]);
       cp5.getController("Yaw_KD").setValue(tempF[12]);
       cp5.getController("Yaw_Max").setValue(tempF[13]);
       cp5.getController("Yaw_Max_I").setValue(tempF[14]);
       
       cp5.getController("AngX_KP").setValue(tempF[15]);
       cp5.getController("AngX_KI").setValue(tempF[16]);
       cp5.getController("AngX_KD").setValue(tempF[17]);
       cp5.getController("AngX_Max").setValue(tempF[18]);
       cp5.getController("AngX_Max_I").setValue(tempF[19]);
       
       cp5.getController("AngY_KP").setValue(tempF[20]);
       cp5.getController("AngY_KI").setValue(tempF[21]);
       cp5.getController("AngY_KD").setValue(tempF[22]);
       cp5.getController("AngY_Max").setValue(tempF[23]);
       cp5.getController("AngY_Max_I").setValue(tempF[24]);
       
       cp5.getController("AngZ_KP").setValue(tempF[25]);
       cp5.getController("AngZ_KI").setValue(tempF[26]);
       cp5.getController("AngZ_KD").setValue(tempF[27]);
       cp5.getController("AngZ_Max").setValue(tempF[28]);
       cp5.getController("AngZ_Max_I").setValue(tempF[29]);
       
        cp5.getController("Alt_KP").setValue(tempF[30]);
       cp5.getController("Alt_KI").setValue(tempF[31]);
       cp5.getController("Alt_KD").setValue(tempF[32]);
       cp5.getController("Alt_Max").setValue(tempF[33]);
       cp5.getController("Alt_Max_I").setValue(tempF[34]);
       
       cp5.getController("AltRate_KP").setValue(tempF[35]);
       cp5.getController("AltRate_KI").setValue(tempF[36]);
       cp5.getController("AltRate_KD").setValue(tempF[37]);
       cp5.getController("AltRate_Max").setValue(tempF[38]);
       cp5.getController("AltRate_Max_I").setValue(tempF[39]);
       
       
       println("Listo");
       }
    }

 }
}
