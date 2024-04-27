# Light Sources and Objects Detector System

![light_detector](Images/light_detector.jpeg)

## Target of the project 
 The purpose of the project is to implement, dsesign and demonstrate an embedded system based on the MCU - PC and capable of detecting light sources and tracking objects in space 
 using a number of components such as the ultrasonic distance meter, LDR light sensors and a servo motor. The main goal is to create a scanning mechanism that covers an area 
 of 180 degrees to collect data from the surrounding environment just like radar.
 This innovative system can use for applications in civilian and military security systems, environmental monitoring and various other scenarios requiring essential light sources 
 detection and object monitoring.

## Features

The system consists of three main components:

- Distance sensor: An Ultrasonic Sensor distance sensor is used to measure the distance between the Servo motor handle and the light source.
- Light sensors: Two LDR light sensors are located on the sides of the distance sensor. These sensors are used to detect the location of the light source using ADC12.
- Servo motor: A Servo motor is used to rotate the light and distance sensors in a 180-degree range by PWM wave duty cycle.

The system operates as follows:

1. The Servo motor rotates the light and distance sensors in a 180-degree range.
2. The distance sensor measures the distance between the Servo motor handle and the light or object source at each position.
3. The distance and location data of the light sources are sent to the microcontroller.
4. The microcontroller processes the data and displays the location and distance results of the light sources on the PC screen through the user interface.

#### ADC12
We used an ADC12 to sample the digital voltage from the LDR, which we then sent to a computer and converted the digital voltage array to analog and then to distance using a linear function.

#### Timers

We used a timer by creating a PWM to activate the trigger of the ULTRASONIC sensor and in addition to activate the rotation of the mini robot by the servo motor, in addition to using the timer to create a delay on the LCD screen

#### UART

Universal Asyncroching Reciver Transmiter we used the COM1 communication port, 9600 baud rate, transfer of 8 bits with start and stop bits, using this protocol we communicated between the PC side and the MCU side

#### Servo Motor: 
The project will implement precise control of the servo motor's angular movement using a PWM signal from the MCU. The servo motor should be able to identify the designated area of 180 degrees in continuous and accurate movement.

#### Data Processing:
Algorithms will be developed to process the data collected from the ultrasonic ranger and LDR sensors. By processing sensor data to create a comprehensive understanding of the environment, the system be able to identify the distance of objects and light sources.

#### Computer GUI User Interface:
A user-friendly interface will be designed using the mouse to select the desired mode for the user, and will also display the scanning results in real time.

## Project modes:

This project is divided into several work modes, each mode will work independently from the other modes and at the end of the GUI mode

![Menu](Images/Menu.png)


## Objects Detector System Mode

The implementation of the Objects Detector System will dynamically monitor objects in space at a defined distance through a user interface 180 by performing only one scan within a scan scope of degrees and at an optimal level of accuracy, it uses a combination of a Serbo engine for scanning - and an ultra rangefinder. Sonic, the system receives a distance to mask, scans by the user's choice and a section of 180 degrees to identify and measure distances to moving objects. In the angular environment of a servo motor it is controlled by means of a Pulse Width Modulation (PWM) signal, the angle is controlled in particular by Duty Cycle The percentage determined with the help of the timer. Collected data undergoes an information processing process on the PC side to create a comprehensive understanding of the environment, providing information in real time of the distance objects.

![Object_Detector](Images/Object_Detector.png)

When we identify objects in vertical space, we place them in the diagram according to their distance and angle

## Telmeter System Mode

Position the servo motor at the angle given to be selected through the user interface and display the distance measured from the distance sensor dynamically and in real time with a resolution of cm

![Telemeter](Images/Telemeter.png)


## System Detector Light Sources

Implementation of the System Detector Sources Light system for dynamically monitoring light sources in space within a range of up to half a meter by performing only one scan with a scan scope of 180 and at an optimal level of accuracy.

With the help of two LDR type light sensors located on the sides of the distance sensor, it is possible to detect light sources in terms of location and distance within a scanning range of 180 degrees around the center point of the servo motor handle and display the results of the location and distance of the light sources on the PC screen through the user interface

## Script Mode 
Scripts activation of the entire system according to the file containing predefined Level High commands. The system can be operated automatically and all parts of the system can be tested. It is required to support the ability to send and receive up to three files and choose to run one of them separately and independently of the selection from the menu on the computer side. Only the supported commands:

![Commands](Images/Commands.png)

Selecting a command script file containing the commands indicated above

![Menu_Scripts](Images/Menu_Scripts.png)

Choosing to load the flash or activate it:

![LoadOrStartFile](Images/LoadOrStartFile.png)


## Finite State Machine

![FSM](Images/State_machine.png)


### Configuration
![Config](Images/Config.png)

## Authors

- Roee Shahmoon
- Noam Klainer 
