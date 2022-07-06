# Antenna-Tracker
This repository consist of all the work done on the Aries recruitment project Antenna Tracker.



DESCRIPTION- In this project we have to continuously track a drone using GPS coordinates and point the antenna to the drone by using motors so that that transmission and receiving can tak place between drone and software.



FILES- This repository consist of 4 files which are-

1- gps.txt- It is a text file consisitng of different GPGGA strings which act as an example og incoming GPS strings from drone.

2- processing.pde- It is a code written in processing language and will run in processing IDE. It open and reads the "gps.txt" file and extracts latitude, longitude and altitude from the GPGGA strings convert them to degrees and assigns positive and negative value according to its direction. After that it sens latitude, longitude and altitude through serial port to the connected Arduino for further calculations.

3- Antenna_Tracker_Arduino.ino- It is the code which runs on arduino and is written in arduino IDE. It does the major part of calculations and controlling of motors. After receiving the coordinates from processing code it calculates the distance between the GPS coordinates of drone and antenna and calculates the angles which will be given to motors for rotating antenna.

4- Speculation Document- It is the brief explaination of our project with circuit diagrams, code explainations, concepts and formulas used, components used, prerequisites, settings for IDE, challenges etc. For the complete working and running of the project, please read the speculation document.



SPECTULATION DOCUMENT
ANTENNA TRACKER

Description
What a life without GPS😊
Our project can make your life a lot easier😉
In this project basically we try to connect the flying device (or object) such as a drone to an antenna that rotates pointing towards that object using both software (programming and using Arduino) and hardware(motor controllers).

Prerequisites
	GPS coordinates 
	Their representations (two types)
	GPS protocols (NMEA sentences and their types)
	Haversine Formula (to calculate the distance between two GPS coordinates)
	Bearing angle (horizontal angle between the object and the true north)
	Arduino software and Arduino IDE
	How to code in Arduino (similar to C language) and it’s simulation.
	How to code in processing language.
	PID_v1 library for Arduino.


Setting Up Arduino & Processing IDE >>
Here are the details about how we got set up the environment:
Arduino IDE Setup:
1.	Go to this link https://www.arduino.cc/en/Main/Software and download Arduino IDE according to your system.
2.	Extract the zip file wherever you want.
3.	Open the extracted folder and run Arduino.exe.
4.	Your Arduino IDE is ready to use.
5.	Now to download PID_v1 library go to PID - Arduino Libraries and download the latest version of PID library
6.	Now open your Arduino IDE go to Sketch > Include Library > Add .zip library.
7.	Now select the downloaded zip file and you are all set.

Processing IDE Setup: 

1.	Go to this link Download \ Processing.org and download the setup according to your system.
2.	Extract the zip file wherever you want.
3.	Open the extracted folder and run processing.exe.
4.	Your processing IDE is ready to use.



Working >>
In the project we assume that there is gps transmitter over the flying object which is sending gps string in form of GPGGA format and gps module receiving that string and storing it in a file over computer. 
In project we already have a file containing lots of GPGGA NEMA string. Program developed in processing language read that string and extract the information about latitude(), longitude() and altitude. This program also transmits the this information as a input via serial communication for Arduino program which calculate the distance between flying object (drone) and the antenna (having fixed co-ordinate) using ‘Haversine Formula’, angle of rotation in horizontal plane using ‘Bearing Angle Formula’ and the third angle which is between the line joining drone and ground using ‘Trigonometry’. These all values comprise as the input for motor controllers which point the antenna over flying object. 
The two angles i.e., horizontal angle and vertical angle are sent to motors. Vertical angle is sent to a servo motor which rotates according to its reference state (0 deg).
Horizontal angle is sent to a motor controller (L293D) which is connected to a dc motor with encoder and uses PID algorithm for precise rotation of motor.
First of all, run the code on Arduino so that it waits for the serial input, then run the processing code which will give the manipulated latitude, longitude and altitude to the Arduino which uses them to calculate distance by haversine formula, calculates height, calculates the vertical angle using distance and height, calculates horizontal angle by using bearing angle formula. After that it gives the vertical angle to a servo motor connected to the Arduino and also sends the horizontal angle to a dc motor with encoder which uses a PID algorithm for precise position and speed.
For more information on dc motor and encoders visit-
How Rotary Encoder Works and How To Use It with Arduino - YouTube
https://youtu.be/dTGITLnYAY0
https://www.youtube.com/watch?v=K7FQSS_iAw0

Here is the Circuit diagram of our project which shows the connections of servo motor, motor controller, Arduino and dc motor:
 

GPGGA STRINGS
Structure ->
$GPGGA,[UTC],[Latitude],[direction],[Longitude],[Direction],[Quality],[No. of Satellites],[HDOP],[Altitude],[Unit],[geoidal separation],[age of correction],[station id],[check sum]  
GP represents that it is a GPS position (Global Positioning), GGA is a type of NMEA message 
UTC- Universal coordinated time in HHMMSS format
Latitude in the format of DDMM.MMMM 
Direction of Latitude (N/S)
Longitude in the format of DDMM.MMMM
Direction of Longitude (W/E)
Indicates the quality of signal 
        1 = uncorrected coordinate
        2 = differentially correct coordinate
        4 = RTK fix coordinate (centimetre precision) 
        5 = RTK float (decimetre precision) 
Number of satellites used to determine coordinates
Horizontal Dilution of Precision 
Altitude Unit of altitude (Metre/ Feet) 
Geoidal separation for more accurate height 
Unit of geoidal separation (Metre/Feet)
Age of correction (0/1) 
Correction station ID 
Checksum (terminate the string)


Calculations  >>
Haversine Formula:
Haversine(θ) = Sin2(θ/2)
(d/R) = Haversine (α2-α1) + Cos(α1) Cos(α2) Haversine (β1 – β2)
where R is the radius of earth (6371 km), d is the distance between two points ie flying object and antenna, α1, α2 are latitude of the two points and β1 , β2 are longitude of the two points respectively.

d = 2RSin-1(sin2 ((α2 – α1)/2) + Cos(α1) Cos(α2) Sin2 ((β2 – β1)/2) )1/2
 


Third Angle Formula:
θ = tan-1((alt2 – alt1 )/d)
where, alt1 is the height of the antenna (fixed)
             alt2 is the height of flying object 
             d is the distance between antenna and flying object (Haversine)

 



Bearing Angle Formula:
Bearing angle is a direction measured from north and it tracks angle in clockwise direction with north line.
θ = tan-12(X, Y)
where, X = Cos(lat1) * Sin(lat2) – [Sin(lat1) * Cos(lat2)] * Cos(lon2-lon1)
            Y = Sin(lon2-lon1) * Cos(lat2)
lat1, lat2, lon1 and lon2 are in radians.

For example, bearing from A to B is 245o.
 


PID:
First let us tell you a control loop system. A control loop feedback system is a system that runs in a close loop, with a set-point to reach. The command given to the actuator to reach the set-point depends on the feedback. This feedback consists in the actuator’s value measured by the sensor and compared to the set-point value. The resulting error is computed and re-injected into the initial order as a command that automatically corrects and adjusts the value of the actuator, in order to reach the set-point. 
The term PID stands for proportional, integral, derivative. They are the three main 
components of the code. These three components will be calculated in function of the error given by the sensor. To simplify things, let’s call these components functions: e.g. F_i(e) is the integral function in function of the error e. The original target is called set-point.

 
Here are few links -
An introduction to PID control with DC motor | by Simon BDY | luos | Medium
https://youtu.be/t7ImNDOQIzM


Helpful links
	https://learn.sparkfun.com/tutorials/what-is-an-arduino
	https://learn.sparkfun.com/tutorials/installing-arduino-ide
	https://learn.sparkfun.com/tutorials/data-types-in-arduino
	https://learn.sparkfun.com/tutorials/serial-communication
	https://learn.sparkfun.com/tutorials/pulse-width-modulation
	YouTube links>>
	https://youtube.com/playlist?list=PLA567CE235D39FA84
	https://youtube.com/playlist?list=PLGs0VKk2DiYw-L-RibttcvK-WBZm8WLEP



Challenges >>
The challenges we faced are:
	How to take input from the file having GPGGA string as a input for our Arduino ide program for calculating distance and bearing angle, as Arduino is not able to read the string from the file over computer and extract the latitude, longitude and altitude of flying object (drone).
	Since the whole project is done in online mode, there was lack of hardware and actual sensible data. Though we try to do the maximum work using simulators but there were many times when we just can’t do anything.( As a result, there are many points in the program where data needs to be modifies according to the situation)
	Using PID control algorithm with dc motor was also one of the challenges that we faced.


Note- Since the complete project has been done in online mode, there are several points where we have taken some assumptions, and also data that we have used is not quite accurate and doesn't resemble much with real time conditions so there are many points where the data has to be changed according to the situation.
