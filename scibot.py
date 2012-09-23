"""scibot.py: the start-up file, where everything branches out from"""
__author__ = "Malte Ahrens"
__license__ = "Attribution 3.0 Unported (CC BY 3.0)"

from multiprocessing import Process, Array
from time import sleep
from create import *
from sensors import LIDAR, Gyroscope
from filters import Histogram
from web import WebControl, Tweetr

if __name__ == '__main__':
    #initialize COM connections with the robot (handled by the create library)
    robot = Create(5)

    #initialize the gyroscope (calibrate it)
    gyro = Gyroscope(robot)

    #initialize the LIDAR (provide the SynchronizedArray from multiprocessing to facilitate the sharing of memory)
    lidar_results = Array('i', 3)
    lidar = LIDAR(23, lidar_results)
    lidar.start()

    #start the robot service (movement, localization, etc.)
    histogram_filter = Histogram(robot, gyro, lidar_results)

    #don't hide my cmd window!
    input()

    #end the lidar process
    lidar.terminate()