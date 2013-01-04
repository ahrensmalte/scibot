"""scibot.py: the start-up file, where everything branches out from"""
__author__ = "Malte Ahrens"
__license__ = "Attribution 3.0 Unported (CC BY 3.0)"

from multiprocessing import Process, Array
from time import sleep
from create import *
from sensors import LIDAR, Gyroscope
from filters import Histogram

if __name__ == '__main__':
    #initialize COM connections with the robot (handled by the create library)
    robot = Create(5)

    #initialize the gyroscope (calibrate it)
    gyro = Gyroscope(robot)

    #initialize the LIDAR to com port 23 (& provide the SynchronizedArray from multiprocessing to facilitate the sharing of memory)
    lidar_results = Array('i', 3)
    lidar = LIDAR(23, lidar_results)
    lidar.start()

    #TODO: see if we need to wait here until the lidar has fully initialized

    #start the robot service (movement, localization, etc.)
    #for debugging, use histogram_filter = Histogram(0,0,0) and comment out all the initialization lines of robot, gyro, and lidar
    #and be sure to do some (un)commenting in filters.py, in the drive() function
    histogram_filter = Histogram(robot, gyro, lidar_results)

    #don't hide my cmd window!
    input()

    #end the lidar process
    lidar.terminate()
