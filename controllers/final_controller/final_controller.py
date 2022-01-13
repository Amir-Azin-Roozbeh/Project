# important points:
# use <robot_position> to get current position of robot in <x,y,theta> format.
# use <robot_omega> to get current values for the wheels in <w1,w2,w3> format.

import numpy as np
from initialization import * 
from controller import GPS, Compass, DistanceSensor
import math
from typing import List
############################################# Variables
TIME_STEP = 32
GPS_SAMPLING_PERIOD = 1000
COMPASS_SAMPLING_PERIOD = 1000
blind_spot_ir = 900

X_GOAL = -0.270026
Y_GOAL = 9.34

THETA_ACCURACY_THRESH = 10
ROBOT_SPEED = 10
EPSILON = 0.001

X_START = 1.14
Y_START = -8.96

############################################ Class
class StatesEnum: # Enum
    MOVE_TOWARD_T = 0
    FOLLOW_BOUNDARY = 1
    STOP = 2

class Direction: 
    LEFT = 0 
    RIGHT = 1
    UP = 2 
    DOWN = 3
    # UP_RIGHT_CORNER = 4
    # UP_LEFT_CORNER = 5 
    # DOWN_RIGHT_CORNER = 6 
    # DOWN_LEFT_CORNER = 7
    CUREVE_DOWN = 8
    CURVE_UP = 9 
    CURVE_RIGHT = 10
    RIGHT_DOWN_CORNER = 11 
    RIGHT_UP_CORNER = 12
    LEFT_UP_CORNER = 13
    LEFT_DOWN_CORNER = 14
############################################ Util functions
def calc_distance_to_destination(robot):
    x, z = get_robot_coordinate(robot)
    return math.sqrt(((X_GOAL-x) ** 2) + ((Y_GOAL-z) ** 2))

def get_bearing_in_degrees(compass_val):
    # calculate bearing angle in degrees 
    rad = math.atan2(compass_val[0], compass_val[1])
    bearing = (rad - 1.5708) / math.pi * 180.0
    if bearing < 0.0:
        bearing = bearing + 360.0
    return bearing

def get_robot_heading(compass_val):
    heading = get_bearing_in_degrees(compass_val)
    heading = 360-heading
    heading = heading + 90
    if heading > 360.0:
        heading = heading - 360.0
    return heading

def move_robot(x, y, theta, heading):
    rotation_matrix = np.array([[np.cos(heading), np.sin(heading), 0], [-np.sin(heading), np.cos(heading), 0], [0, 0, 1]])
    temp = np.dot(rotation_matrix, np.array([x, y, theta]))
    inverse_matrix = np.array([[-0.33, 0.58, 0.33], [-0.33, -0.58, 0.33], [0.67, 0, 0.33]])
    speed = np.matmul(inverse_matrix, temp)
    update_motor_speed(speed)

def calculate_theta_dot(inertial_theta): 
    if inertial_theta >= 180: 
        inertial_theta = -(360 - inertial_theta)
    return inertial_theta

def check_if_obstacle(ir_value) -> Direction:
    ir_value_1, ir_value_2, ir_value_3, ir_value_4, ir_value_5, ir_value_6 = ir_value
    number_of_on_irs = sum([ 1 if value<1000 else 0  for value in ir_value])
    if number_of_on_irs == 1 : 
        if ir_value_1 < blind_spot_ir: 
            return Direction.CUREVE_DOWN
        elif ir_value_3 < blind_spot_ir: 
            return Direction.CURVE_UP
        elif ir_value_5 < blind_spot_ir: 
            return Direction.CURVE_RIGHT
        elif ir_value_2 < blind_spot_ir:
            return Direction.RIGHT

    elif number_of_on_irs == 2 : 
        if ir_value_1 < blind_spot_ir and ir_value_4 < blind_spot_ir:
            return Direction.DOWN
        if ir_value_3 < blind_spot_ir and ir_value_5 < blind_spot_ir:
            return Direction.UP
        if ir_value_6 < blind_spot_ir and ir_value_3 < blind_spot_ir:
            return Direction.LEFT
        if ir_value_2 < blind_spot_ir and ir_value_5 < blind_spot_ir:
            return Direction.RIGHT 

    elif number_of_on_irs >= 3: 
        if ir_value_1 < blind_spot_ir and ir_value_4 < blind_spot_ir and ir_value_2 < blind_spot_ir: 
            return Direction.RIGHT_DOWN_CORNER
        elif ir_value_1 < blind_spot_ir and ir_value_4 < blind_spot_ir and ir_value_6 < blind_spot_ir:
            return Direction.LEFT_DOWN_CORNER
        elif ir_value_3 < blind_spot_ir and ir_value_5 < blind_spot_ir and ir_value_6 < blind_spot_ir:
            return Direction.LEFT_UP_CORNER
        elif ir_value_2 < blind_spot_ir and ir_value_5 < blind_spot_ir and ir_value_3 < blind_spot_ir: 
            return Direction.RIGHT_UP_CORNER

    return None


def follow_wall(direction, theta_dot, robot_position): 
    if direction == Direction.UP: 
        move_robot(-ROBOT_SPEED, 0, theta_dot, robot_position[2])
    elif direction == Direction.DOWN: 
        move_robot(ROBOT_SPEED, 0, theta_dot, robot_position[2])
    elif direction == Direction.RIGHT: 
        move_robot(0, ROBOT_SPEED, theta_dot, robot_position[2])
    elif direction == Direction.LEFT:
        move_robot(0, -ROBOT_SPEED, theta_dot, robot_position[2])
    elif direction == Direction.CURVE_RIGHT:
        counter = 0
        while robot.step(TIME_STEP) != -1 and counter <= 400: 
            if counter <= 120: 
                move_robot(-ROBOT_SPEED, 0, theta_dot, robot_position[2])
            elif counter <= 250: 
                move_robot(0, ROBOT_SPEED, theta_dot, robot_position[2])
            else: 
                move_robot(ROBOT_SPEED, 0, theta_dot, robot_position[2])
            counter += 1
    elif direction == Direction.CUREVE_DOWN: 
        counter = 0
        while robot.step(TIME_STEP) != -1 and counter <= 300: 
            if counter <= 120: 
                #RIGHT 
                move_robot(ROBOT_SPEED, 0, theta_dot, robot_position[2])
            elif counter <= 200: 
                #UP 
                move_robot(0, -ROBOT_SPEED, theta_dot, robot_position[2])
            else:
                #LEFT 
                move_robot(-ROBOT_SPEED, 0, theta_dot, robot_position[2])
            counter += 1
    elif direction == Direction.CURVE_UP: 
        counter = 0
        while robot.step(TIME_STEP) != -1 and counter <= 300: 
            if counter <= 120: 
                #DOWN 
                move_robot(0, -ROBOT_SPEED, theta_dot, robot_position[2])
            elif counter <= 200: 
                #RIGHT
                move_robot(ROBOT_SPEED, 0, theta_dot, robot_position[2])
            else:
                #UP
                move_robot(0, ROBOT_SPEED, theta_dot, robot_position[2])
            counter += 1
    elif direction == Direction.LEFT_UP_CORNER: 
        # down
        move_robot(0, -ROBOT_SPEED, theta_dot, robot_position[2])
    elif direction == Direction.RIGHT_UP_CORNER: 
        # left
        move_robot(-ROBOT_SPEED, 0, theta_dot, robot_position[2]) 
    elif direction == Direction.LEFT_DOWN_CORNER:  
        pass 
    elif direction == Direction.RIGHT_DOWN_CORNER: 
        print("mirim balaaa")
        # up 
        move_robot(0, ROBOT_SPEED, theta_dot, robot_position[2])  


def make_line(x1, y1, x2, y2):
    if abs(x2 - x1) < EPSILON:
        return None, None
    m = (y2 - y1) / (x2 - x1)
    c = m * x1 - y1
    return m, c



if __name__ == "__main__":

    robot = init_robot(time_step=TIME_STEP)
    goal_postition = np.array([0,0])
    state: StatesEnum = StatesEnum.MOVE_TOWARD_T
    m, c = make_line(X_START, Y_START, X_GOAL, Y_GOAL)

    while robot.step(TIME_STEP) != -1:

        gps_values,compass_val,sonar_value,position_value,ir_value = read_sensors_values()
        
        x_current, y_current, _ = gps_values
        theta = math.atan2(Y_GOAL - gps_values[1], X_GOAL - gps_values[0]) * 180 / math.pi
        inertial_theta = get_bearing_in_degrees(compass_val)
        theta_dot = calculate_theta_dot(inertial_theta)
        print('state ', state)
        print('ir value:', ir_value)
        if state == StatesEnum.MOVE_TOWARD_T:
            if check_if_obstacle(ir_value): # if it detects an obstacle  
                state = StatesEnum.FOLLOW_BOUNDARY
            else:
                sin_theta = math.sin(theta)
                cos_theta = math.cos(theta)
                move_robot(ROBOT_SPEED * cos_theta, ROBOT_SPEED * sin_theta, theta_dot , robot_position[2])
        elif state == StatesEnum.FOLLOW_BOUNDARY:
            if not check_if_obstacle(ir_value): 
                print('jaye eshtebah')
                state = StatesEnum.MOVE_TOWARD_T
            else:
                obstacle_directions = check_if_obstacle(ir_value)
                print('obs ', obstacle_directions)
                follow_wall(obstacle_directions, theta_dot, robot_position)
        elif state == StatesEnum.STOP:
            ...

        
    pass



## (-10, 0) --> left 