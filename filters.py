"""filters.py: A collection of filters (only one so far) for localizing the robot"""
__author__ = "Malte Ahrens"
__license__ = "Attribution 3.0 Unported (CC BY 3.0)"

from PIL import Image
from time import sleep, time

class Histogram():
    """runs the histogram filter (Monte-Carlo localization) to localize the robot"""
    
    def __init__(self, robot, gyro, lidar_results):
        self.robot = robot
        self.gyro = gyro
        self.lidar_results = lidar_results
        	
        #construct the map from an image
        self.map = self.convert_image_to_map()

        #note sending map[0], they're all the same for the different directions, so only send one of them
        self.sense_map = self.create_sense_options(self.map[0]) 
        
        #send an initial probability where all cells are equal
        self.p = self.normalize(self.convert_image_to_map(False))
        
        #start the driving / localization
        self.drive()

    def convert_image_to_map(self, ones = True, fp = 'C:/Users/116927/SkyDrive/School/Robotics/Code/hallway.png'):
        """converts the specified image into a map (matrix) via pixel values"""

        #load the image with PIL (Python Imaging Library - http://www.pythonware.com/products/pil/)
        #   python3 version (unofficial) from http://www.lfd.uci.edu/~gohlke/pythonlibs/
        i = Image.open(fp)
        pixels = i.load()
        width, height = i.size
        map = []

        #go through all the pixels, and if they're black => it's non passable terrain (i.e. a wall),
        #                               if they're white => it's movable space (i.e. the corridor in this case)
        for y in range(height):
            row = []
            for x in range(width):
                val = pixels[x, y][0] #choose the first value (of RGB) -> doesn't matter, it's Black & White

                #if ones, the walls become 1
                if ones:
                    row.append(0 if val > 0 else 1)

                #if not ones, the movable space becomes 1 (needed for the initial probability)
                else:
                    row.append(1 if val > 0 else 0)

            map.append(row)

        #return an individual map for each of the four directions (N, E, S, W)
        #   (it's the same map, just four times)
        return [map for direction in range(4)]
    
    def create_sense_options(self, map):
        """Creates an array of the expected sensor values for different places in the map.
            The array is in the form of [left, forward, right] sensor values"""
        #fill the sense_map matrix with empty arrays
        sense_map = [[[[] for j in range(len(map[0]))] for i in range(len(map))] for direction in range(4)]
        
        #go through the map
        for y in range(len(map)):
            for x in range(len(map[y])):
                #if it is moveable terrain
                if map[y][x] == 0:
                        #looking north
                        sense_map[0][y][x] = [map[y][x - 1], map[y - 1][x], map[y][x + 1]]
                        
                        #looking east
                        sense_map[1][y][x] = [map[y - 1][x], map[y][x + 1], map[y + 1][x]]
                        
                        #looking south
                        sense_map[2][y][x] = [map[y][x + 1], map[y + 1][x], map[y][x - 1]]
                        
                        #looking west
                        sense_map[3][y][x] = [map[y + 1][x], map[y][x - 1], map[y - 1][x]]
                                
        return sense_map
            
            
    def convert_to_command(self, sensor_sees, distance = 1):
        """converts the lidar results (e.g. [0, 1, 0]) to a [x,y] turn vector (e.g. [1,1], or move right & drive)"""
        
        #if we can move forward, move forward
        if not sensor_sees[1]:
            command = [1 * distance, 0]
                
        #else try to move right
        elif not sensor_sees[2]:
            command = [1, 1 * distance]
                
        #if that doesn't work, try to move left
        elif not sensor_sees[0]:
            command = [1, -1 * distance]
        
        #no place to go, so we turn right (on the spot)
        else:
            command = [0, 1]

        return command

    def move_distance(self, distance = 1, speed = 20):
        """moves the robot a set distance (in meters) at a set speed (in cm/sec)"""

        distance *= 1000 #convert to mm
        
        #if we are to move forward
        if distance > 1:
            self.robot.go(speed, 0)
            distance *= 0.91 #tolerance

        #if we are to move backwards
        elif distance < 1:
            self.robot.go(-speed, 0)
            distance *= 0.91 #tolerance

        #continuously check the wheel encoders (on the robot) to see if we've travelled the set distances
        dtravelled = 0
        while abs(dtravelled) < abs(distance):
            dtravelled += int(self.robot.getSensor('DISTANCE') or 0) #'DISTANCE' is in mm
            sleep(0.01)

        #once we travelled that distance, stop
        self.robot.stop()

    def move_turn(self, turn = 'right', speed = 30):
        """turns the robot a set angle (in degrees) at a set speed (in cm/sec)"""

        #if we are to turn counterclockwise
        if turn == 'left':
            self.robot.go(0, speed)

        #if we are to turn clockwise
        elif turn == 'right':
            self.robot.go(0, -speed)

        #use the gyro to tell the robot when to stop (when it has turned 90 degrees)
        self.gyro.turn()

    def move_robot(self, move_command):
        """moves the robot as per the command given (e.g. [1,1] is translated to turn right then move forward 1m"""

        if move_command == [0, 1]:
            self.move_turn('right')

        elif move_command == [1,0]:
            self.move_distance(1)

        elif move_command == [-1,0]:
            self.move_distance(-1)

        elif move_command == [1, 1]:
            self.move_turn('right')
            self.move_distance(1)

        elif move_command == [1, -1]:
            self.move_turn('left')
            self.move_distance(1)

    p = [] #probability map of where we think we are
    map = [] #map of surroundings (1 => wall, 0 => movable terrain)
    def drive(self):
        """starts the process of sense (update probabilities) -> move (real robot) -> move (update probabilities), repeat"""

        #limit the loop to just 5 times...makes catching an escaping robot easier...
        for x in range(10): 

            #get the latest results from the lidar
            sensor_sees = self.lidar_results[:]

            #update the probabilities (sense)
            self.p = self.sense(sensor_sees)
            
            #debug stuff
            print('sensor reading: ' + str(sensor_sees))
            self.show(self.normalize(self.p))

            #convert turn & distance to [x,y] command vector
            move_command = self.convert_to_command(sensor_sees)            

            #actually move the robot
            self.move_robot(move_command)

            #update the probabilities (move)
            self.p = self.move(move_command)
                    
            #normalize the probability array
            self.p = self.normalize(self.p)
            
            #more debug stuff
            print('\nmove command: ' + str(move_command))
            self.show(self.p)
            print('\n\n')

    def normalize(self, p):
        """normalizes the probability array that they all add up to 1 again"""

        #first find the sum of all the cells
        #   (across all four directions so we can compare directional probability as well)
        p_sum = 0
        for direction in range(4):
            for row in range(len(p[direction])):
                p_sum += sum(p[direction][row])
                    
        #then divide each cell by the sum - i.e. normalize the probabilities
        for direction in range(4):
            for row in range(len(p[direction])):
                for col in range(len(p[direction][row])):
                        p[direction][row][col] /= p_sum

        return p

    p_sense = 0.95 #the probability of successful sensor reading...pretty high
    sense_map = [] #map of the expected sensor readings at different places (and directions)
    def sense(self, sensor_sees):
        """update all the probabilities given that we have new sensor information"""
        
        new_probability = [[] for direction in range(4)]

        #go through each cell on the map
        for direction in range(4):
            for row in range(len(self.p[direction])):
                    row_probabilities = []
                    for col in range(len(self.p[direction][row])):
                            #is the sensor result expected given the [row][col] cell?
                            hit = (sensor_sees == self.sense_map[direction][row][col])
                            
                            #multiplication factor of the existing probability of that cell
                            #   i.e. if the sensor reading matches up, it's likely we're there, so multiple by p_sense
                            #   if doesn't multiply by 1 - p_sense (which drastically decreases it's [the cell's] probability)
                            factor = self.p_sense if hit else 1 - self.p_sense
                            
                            row_probabilities.append(self.p[direction][row][col] * factor)
                    
                    new_probability[direction].append(row_probabilities)

        return new_probability

    p_move = 0.9 #the probability of successful movement...decently high
    def move(self, move, distance = 1):
        """update all the probabilities given that we move"""

        new_probability = [[] for direction in range(4)]
        for direction in range(4):
            #cycle the maps if there is a rotate in the move command. e.g.
            #   if we move right, N probabilities go to E, E goes to S, etc.
            #   using direction_index that we access the 'rotated map' data
            #   rather than that of the current direction
            direction_index = (direction - move[1]) % 4

            #convert the relative command (relative to the robot) to one that is relative
            #   to the current maps' direction
            directional_move = []
            if direction == 0: #N
                directional_move = [move[0], 0]
            elif direction == 1: #E
                directional_move = [0, move[0]]
            elif direction == 2: #S
                directional_move = [-move[0], 0]
            elif direction == 3: #W
                directional_move = [0, -move[0]]

            for row in range(len(self.p[direction_index])):
                row_probabilities = []
                for col in range(len(self.p[direction_index][row])):

                    #if the cell is not a wall
                    if self.map[direction_index][row][col] == 0:

                        #the probability of the cell we're coming from (given the motion)
                        pr_onthatcell = self.p[direction_index][(row + directional_move[0])][col - directional_move[1]]

                        #the probability that we therefore arrive at this cell
                        pr_movetocell = self.p_move * pr_onthatcell

                        #the probability that instead of moving, we stay on the current cell (e.g. broken robot)
                        #   the 1 - p_move makes it pretty small...
                        pr_stayoncell = (1 - self.p_move) * self.p[direction_index][row][col]

                        #the sum of the probabilities is the probability that we are on the current cell
                        #   either by arriving here, or by staying here
                        row_probabilities.append(pr_movetocell + pr_stayoncell)
                    else:
                        row_probabilities.append(0)
                        
                new_probability[direction].append(row_probabilities)

        return new_probability

    def show(self, p):
        """prints the map / p / sense_map matrices with rounding and (somewhat) nicer formatting"""

        for direction in range(4):
            print('\n\tdirection ' + str(direction))

            for i in range(len(p[direction])):
                to_print = '\t'

                for j in range(len(p[direction][0])):
                    to_print += str(round(p[direction][i][j], 3)) + ', '

                print(to_print)