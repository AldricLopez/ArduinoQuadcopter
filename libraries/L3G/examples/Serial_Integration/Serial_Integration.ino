#include <Wire.h>
#include <L3G.h>

L3G gyro;

float Rate[3];
float GyroAng[3];

void setup() {
  Serial.begin(9600);
  Wire.begin();

  if (!gyro.init())
  {
    Serial.println("Failed to autodetect gyro type!");
    while (1);
  }

  gyro.enableDefault();
}

void loop() {
  
  gyro.read();

  Rate[0]= gyro.g.x * .00875;
  Rate[1]= gyro.g.y * .00875;
  Rate[2]= gyro.g.z * .00875;
  
  GyroAng[0]+=Rate[0]*0.1;
  GyroAng[1]+=Rate[1]*0.1;
  GyroAng[2]+=Rate[2]*0.1;
  
  Serial.print(GyroAng[0]);
  Serial.print("  ");
  Serial.print(GyroAng[1]);
  Serial.print("  ");
  Serial.println(GyroAng[2]);
  delay(100);
}
