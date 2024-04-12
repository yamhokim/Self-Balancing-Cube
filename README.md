# MIE438 Self-Balancing Cube Project

A course project for MIE438: Microprocessors and Embedded Microcontrollers at the University of Toronto.

Necessary Hardware:
- ESP32 WROOM-32D Microcontroller
- 3 Brushed or Brushless DC Motors (Preferably the Nidec 24H Brushless Motors)
- 2 L298N Motor Driver Controllers
- MPU-6050 Inertial Measurement Unit
- Siglent Technologies SPD3303C DC Power Supply or 3S 500mAh lithium polymer battery

Design Overview/Layout:
![image](https://github.com/yamhokim/Self-Balancing-Cube/assets/116863345/0c43075b-69e7-461b-ad2e-08ab5a6d37bd)

Control Loop:
- Gyroscope and accelerometer readings captured by MPU-6050 to determine the angular position of the cube
- PID controller computes the necessary angular velocity of the respective reaction wheel

PID Control Loop Diagram
![image](https://github.com/yamhokim/Self-Balancing-Cube/assets/116863345/c71a250b-7d27-4050-89a7-3d3da811e106)

Overall Control Loop Diagram
![image](https://github.com/yamhokim/Self-Balancing-Cube/assets/116863345/bdcd7541-a730-4ffc-9780-c9e9a0a4150e)

To run the code:
```
ADD LATER
```
