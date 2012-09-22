
#scibot
The beginnings of a (somewhat) self-aware robot. A set of python files enabling the reading of data from sensors, localization, and moving of the robot.

_scibot.py_ is the start-up file, where everything branches out from
_sensors.py_ handles communications between computer & external sensors
_filters.py_ is a collection of filters (only one at the moment though) for localizing the robot

**Licensed under Creative Commons Attribution 3.0 Unported (CC BY 3.0)** http://creativecommons.org/licenses/by/3.0/

###Designed to work with
* iRobot Create (using the _______ library)
* Hokuyo URG-04LX-UG01 LIDAR sensor*
* Analog Devices ADXR652 gyroscope*
______
*should work with other sensors as well though

####as of 22nd September 2012
* LIDAR serial interface (which runs in a separate process for multitasking) gets a continuous stream of data from the sensor (using the MS command) and converts it into a left, forward, and right sensor readings
* Gyroscope serial interface (which runs through the Create's analog port) handling the 90 degree rotations of the robot
* Basic Mote-Carlo localization (histogram filter) implemented that localizes the robot over a map (including the four orientations of N, E, S, W) 

_todo:_ upload map.png
