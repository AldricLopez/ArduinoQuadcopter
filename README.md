# ArduinoQuadcopter
Quadcopter controller based on Arduino. Currently only supporting X configuration and specific hardware. 
A video of this proyect working can be seen at: https://www.youtube.com/watch?v=z5Tj1yTc0Bw

To use this controller, download the whole proyect. Place everything  (except the ControlP5 library) that is in the libraries folder into your libraries folder from Arduino. (Documents/Arduino/libraries, see http://www.arduino.cc/en/Guide/Libraries#toc5). 
Then open the Arduino folder and place the QuadController folder in your sketchbook folder (Documents/Arduino/sketchbook). Finally open the Processing folder and copy everything to your Processing folder. Also copy the ControlP5 library to your processing libraries folder, similar to what you did with Arduino. 
Compile and program the Arduino Sketch, then build the Processing sketch and click Send button.

For more information on the controller implementation see the PDF (Spanish) in the Explanation folder.

##Hardware Required
The costum built controller uses the following sensors connected to an Atmega328 with Arduino bootloader

- Accelerometer	ADXL345 
- Gyroscope	L3G4200D 
- Magnetometer	HMC5883L
- Barometer	BMP085

Some possible IMUs:

- Kingduino http://www.hobbyking.com/hobbyking/store/__26913__Kingduino_10DOF_L3G4200D_ADXL345_HMC5883L_BMP085_Sensor_Stick_Breakout_for_MWC_KK_ACM.html

- Ebay http://www.ebay.com/itm/10DOF-9-axis-Attitude-Indicator-L3G4200D-ADXL345-HMC5883L-BMP085-Module-Arduino-/400344896529

##PC Software
There is also a pc software available to monitor the controller and change PID constants. This software is made with Processing and can be found on the Processing folder.


##List of Parts
This is a list of parts to build a quadcopter and controll board that works with this software:

- 4x Brushless Motor http://www.hobbyking.com/hobbyking/store/__19834__29_5x26mm_2800kv_Brushless_Outrunner_Motor.html

- 4x AfroESC 12A  http://www.hobbyking.com/hobbyking/store/__55241__Afro_ESC_12Amp_Ultra_Lite_Multi_rotor_Motor_Speed_Controller_SimonK_Firmware_Version_3.html

- Package of 6x3 propellers (extra spare recomended) http://www.hobbyking.com/hobbyking/store/__11329__6x3_Propellers_Standard_and_Counter_Rotating_6pc_.html

- X 2200mAh 3S Lipo http://www.hobbyking.com/hobbyking/store/uh_viewItem.asp?idProduct=8932

- 1x Orange Rx http://www.hobbyking.com/hobbyking/store/__20684__OrangeRx_R710_DSM2_Compatible_7Ch_w_Failsafe.html

- Kingduino http://www.hobbyking.com/hobbyking/store/__26913__Kingduino_10DOF_L3G4200D_ADXL345_HMC5883L_BMP085_Sensor_Stick_Breakout_for_MWC_KK_ACM.html

- Atmega328 (Arduino UNO, or any other arduino with 328) https://www.sparkfun.com/products/10524   

- Passive components for programming Atmega328 if not using Arduino.

- Frame, any scratch build or commercial frame, 30x30cm around 500g
