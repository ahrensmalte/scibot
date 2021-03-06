
#scibot
The beginnings of a (somewhat) self-aware robot. Currently, it's just a set of python files enabling the reading & interpreting of data from sensors, localization, and moving of the robot.

* _scibot.py_ is the start-up file, where everything branches out from
* _sensors.py_ handles communications between the computer and the external sensors
* _filters.py_ is a collection of filters (only one at the moment, though) for localizing the robot
* _create.py_ is a python wrapper for the iRobot Create's serial interface. It's written by Zach Dodds et al. from the [Rose-Hulman Institute of Technology](http://www.rose-hulman.edu/class/csse/resources/Robotics/). Unlike the rest of the code, it is **not** under the CC BY 3.0 license 
* _hallway.png_ is the image out of which the map is built. Each pixel denotes a 1m² block where white ⇒ movable terrain, black ⇒ a wall

######Designed to work with
* iRobot Create
* Hokuyo URG-04LX-UG01 LIDAR sensor¹
* Analog Devices ADXR652 gyroscope¹

¹should work with other sensors as well

**Licensed under Creative Commons Attribution 3.0 Unported (CC BY 3.0)** http://creativecommons.org/licenses/by/3.0/

______

######as of 22nd September 2012
* LIDAR serial interface (which runs in a separate process for multitasking) gets a continuous stream of data from the sensor (using the MS command) and converts it into a left, forward, and right sensor readings
* Gyroscope serial interface (which runs through the Create's analog port) handling the 90 degree rotations of the robot
* Basic Mote-Carlo localization (histogram filter) implemented that localizes the robot over a map (including the four orientations of NESW) 

Together, this allows the robot to successfully drive around and localize itself in our hallway at home (see [hallway.png](https://github.com/ahrensmalte/scibot/blob/master/hallway.png))

######as of 23rd September 2012
* There's now some debugging code that enables use with no robot, gyro, or lidar (just a computer is needed)
