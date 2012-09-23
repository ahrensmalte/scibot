"""sensors.py: handles communications between the computer and the external sensors"""
__author__ = "Malte Ahrens"
__license__ = "Attribution 3.0 Unported (CC BY 3.0)"

from multiprocessing import Process, Array
from time import clock, sleep
import serial

class Gyroscope():
    """Calibrates and provides the angular velocity from the gyroscope
    Designed for & tested with the Analog Devices ADXR652 - http://www.analog.com/en/mems-sensors/mems-inertial-sensors/adxrs652/products/product.html"""

    def __init__(self, robot):
        self.robot = robot
        self.calibrate()

    def get_angular_velocity(self):
        """Returns the current angular velocity"""

        #Convert [0-1023] iRobot 4byte number to a voltage between [0-5]
        #then corrects for calibrated temperature-voltage difference
        voltage = self.robot.getSensor('USER_ANALOG_INPUT') / 1023 * 5
        voltage += self.voltage_difference

        #Convert voltage to w (typical response for ADXRS652 is 7mV/degrees/sec)
        return (voltage - 2.5) / (0.007)

    yaw = 0
    def turn(self, angle = 90, sleep_time = 0.005):
        """turns the robot a set angle (in degrees) by integrating the angular velocity to get the yaw rate"""

        #counter for time required to stop the robot again (about 4 degrees, depending on speed)
        angle -= 10

        w_inital = 0
        while abs(self.yaw) < angle:
            #current time
            time_0 = clock()

            #sleep a little and allow for other comms to go through the robot
            sleep(sleep_time)
            
            w_final = self.get_angular_velocity()

            #get the elapsed time as the serial communication takes a non-negligible amount of time
            time_elapsed = clock() - time_0

            #update the yaw angle using the area of a trapezium
            self.yaw += (w_inital + w_final) / 2 * time_elapsed
            w_inital = w_final

        #turn off the robot
        self.robot.stop()

        #reset the yaw rate to 0
        self.yaw = 0

    def get_yaw_angle(self):
        """Returns the current yaw angle (where 0 is the angle the robot was, when this thread was started)"""
        return self.yaw

    voltage_difference = 0 #calibrated temperature-voltage difference
    def calibrate(self):
        """Calibrates the gyroscope for any temperature related differences"""

        #make sure gyroscope (the robo) is still
        self.robot.stop()

        #calibrate by comparing actual response to expected response (at angular velocity=0)
        voltage_0 = self.robot.getSensor('USER_ANALOG_INPUT') / 1023 * 5
        self.voltage_difference = 2.5 - voltage_0

        #play note to confirm calibration
        self.robot.playNote(50,25,0)


class LIDAR(Process):
    """Communicates with the LIDAR to give depth information. 
    It runs in a separate process (=> can run on a separate processor core) as to ensure a real-time data feed
    Designed for & tested with the Hokuyo URG-04LX-UG01 (though should work with other Hokuyo lasers as well)"""

    def __init__(self, port, results):
        #initialize the process
        Process.__init__(self)

        self.port = port
        self.results = results
        
    def decode(self, byte):
        """decodes the byte value response from the lidar to something more intelligible (base 10 number).
            Code from http://www.hokuyo-aut.jp/02sensor/07scanner/download/urg_programs_en/scip_capture_page.html"""
        value = 0

        for i in range(len(byte)):
            value <<= 6
            value &= ~0x3f
            value |= byte[i] - 0x30

        #if the value is zero (i.e. the 'thing' is out of range of the lidar, return 4085 - the furtherest the lidar can see
        return value if not value == 0 else 4085

    def average(self, list):
        """averages a list of numbers to two decimal places (e.g. [1,2,3] => 2)"""
        return round(sum(list) / len(list) / 1000, 2)
        
    def run(self):
        #connect the serial port of the lidar (note the -1 -> it's some pyserial nuance)
        comm = serial.Serial(self.port - 1, baudrate=19200, timeout=0.5)
        
        #lidar variables for distance data
        recieve_method = 'M' #G => single, M => continuous
        depth_limit = 'S' #S => 2 bytes w/ max of 4085mm, D => 3 bytes w/ max of 262144mm
        first_index = '0044' #initial measurement Step of Detection Range
        last_index = '0725' #end point of Detection Range
        data_grouping = '01' #resolution of the scan, the number of points to combine into one data point
        scan_skip_interval = '0' #not sure...
        scan_times = '00' #number of times to scan, 00 => infinite

        #send the MS command (continuous data acquisition) - with '\r' denoting end of command for the lidar
        command = recieve_method + depth_limit + first_index + last_index + data_grouping + scan_skip_interval + scan_times + '\r'
        comm.write((bytes(command, encoding='Latin-1')))

        #read the switching on response (while the laser completes a cycle till it is switched on)
        comm.read(21) 

        #with the lidar on, and continuously providing data, read and interpret the data stream
        while True == True:
            #read the returned data
            tempdat = comm.read(1435).decode()
            data = str(tempdat).split('\n')

            #decode the timestamp of the cycle
            time = self.decode(bytes(data[2], encoding='Latin-1'))

            data = data[3:] #just the depth data
            depth_data = []

            #convert to depth values - there are 65 bytes per line and each distance reading is 2 bytes (the last byte is a check sum of that line)
            #   further reading: http://www.hokuyo-aut.jp/02sensor/07scanner/download/urg_programs_en/scip_capture_page.html
            for i in range(len(data) - 2):
                for j in range(0, 63, 2):
                    depth_data.append(self.decode(bytes(data[i][j:j + 2], encoding = 'Latin-1')))

            #average parts of the distance data for slivers of averaged depth (to simplify coding the sense() function) and 
            #convert it to a Boolean (array - process safe) response of whether something is in the way [left, forward, right] of the lidar
            #given that it is greater than / less than the threshold (in meters) - e.g. 0.6m => 0 => DANGER! DANGER! DANGER! wall / obstacle there
            threshold = 0.9
            self.results[0] = int(self.average(depth_data[554:645]) < threshold) #looking left (from 190 to 200 degrees)
            self.results[1] = int(self.average(depth_data[296:387]) < threshold) #looking forward (from 110 to 130 degrees)
            self.results[2] = int(self.average(depth_data[38:129]) < threshold) #looking right (from 30 to 40 degrees)