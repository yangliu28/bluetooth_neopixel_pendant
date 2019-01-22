
## Hardware design

ATMEGA32U4 has built-in USB interface, for easily reprogramming the device without a USB to serial converter. The system is powered at 3.3v and clocked at 8MHz. Bluetooth module is HC-06 configured in slave mode. RGB LEDs are Neopixel Nano, with a small footprint of 2.4x2.7mm.

Diameter of the circular PCB is 1.7in, radius of 850 mil.

Matlab script to generate pos and orientation of neopixels:
```
d=735;
pos=zeros(10:3);
pos(1:5,1) = 850+d*cos((90+22+34*n)*pi/180);
pos(6:10,1) = 850+d*cos((-90+22+34*n)*pi/180);
pos(1:5,2) = 850+d*sin((90+22+34*n)*pi/180);
pos(6:10,2) = 850+d*sin((-90+22+34*n)*pi/180);
pos(1:5,3) = -180+22+34*n;
pos(6:10,3) = 22+34*n;
```

### Board fix

![post-assembly modifications](board_fix.jpg)

**Reset pin of ATMEGA32U4 was be pulled up to 3V3 with a 10K resistor**

**Enable pins of two regulators for bluetooth and IMU were not working, because the voltage level of enable pin does not match output. Jumpers were used to keep regulators enabled always.**

