# ArduinoQuadcopter
Quadcopter controller based on Arduino. Currently only supporting X configuration and specific hardware. For more information on the controller implementation see the PDF (Spanish) in the Explanation folder

##Hardware Required
The costum built controller uses the following sensors connected to an Atmega328 with Arduino bootloader

- Accelerometer	ADXL345 
- Gyroscope	L3G4200D 
- Magnetometer	HMC5883L
- Barometer	BMP085

##PC Software
There is also a pc software available to monitor the controller and change PID constants. This software is made with Processing and can be found on the Processing folder.