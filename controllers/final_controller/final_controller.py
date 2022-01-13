# important points:
# use <robot_position> to get current position of robot in <x,y,theta> format.
# use <robot_omega> to get current values for the wheels in <w1,w2,w3> format.

import numpy as np
from initialization import * 
from controller import GPS, Compass, DistanceSensor
import math
from typing import List, Tuple
############################################# Variables
TIME_STEP = 32
GPS_SAMPLING_PERIOD = 1000
COMPASS_SAMPLING_PERIOD = 1000
blind_spot_ir = 920

X_GOAL = -0.270026
Y_GOAL = 9.34

THETA_ACCURACY_THRESH = 10
ROBOT_SPEED = 10

EPSILON_LINE = 0.001
EPSILON = 10

X_START = 1.14
Y_START = -8.96

IS_LEFT = True 

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
    CURVE = 9 
    CURVE_RIGHT = 10
    RIGHT_DOWN_CORNER = 11 
    RIGHT_CORNER = 12
    LEFT_CORNER = 13
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


def get_target_zone(theta: float) -> Direction:
    if (theta >= 0 and theta <= 45) or (theta >= 315 and theta <= 360):
        return Direction.RIGHT
    if (theta > 45 and theta < 135):
        return Direction.UP
    if (theta >= 135 and theta < 225):
        return Direction.LEFT
    if (theta > 225 and theta < 315):
        return Direction.DOWN
    else:
        print("===============================================================================================")


def check_if_obstacle(ir_value, theta: float) -> Tuple[Direction, bool]:
    bool_return_value = False
    ir_value_1, ir_value_2, ir_value_3, ir_value_4, ir_value_5, ir_value_6 = ir_value
    target_zone = get_target_zone(theta)
    if target_zone == Direction.UP:
        if ir_value_3 < blind_spot_ir or ir_value_5 < blind_spot_ir:
            bool_return_value = True
    elif target_zone == Direction.LEFT:
        if ir_value_6 < blind_spot_ir:
            bool_return_value = True
    elif target_zone == Direction.DOWN:
        if ir_value_1 < blind_spot_ir or ir_value_4 < blind_spot_ir:
            bool_return_value = True
    elif target_zone == Direction.RIGHT:
        if ir_value_2 < blind_spot_ir:
            bool_return_value = True
    else:
        bool_return_value = False
    number_of_on_irs = sum([ 1 if value<1000 else 0  for value in ir_value])
    direction_return_value = None
    print(ir_value)
    if number_of_on_irs == 1: 
        if ir_value_1 < blind_spot_ir: 
            direction_return_value = Direction.CUREVE_DOWN
        elif ir_value_3 < blind_spot_ir: 
            direction_return_value = Direction.CURVE
        elif ir_value_5 < blind_spot_ir: 
            direction_return_value = Direction.CURVE_RIGHT
        elif ir_value_2 < blind_spot_ir:
            direction_return_value = Direction.RIGHT
        elif ir_value_6 < blind_spot_ir: 
            direction_return_value = Direction.LEFT

    elif number_of_on_irs == 2 : 
        if ir_value_1 < blind_spot_ir and ir_value_4 < blind_spot_ir:
            direction_return_value = Direction.DOWN
        if ir_value_3 < blind_spot_ir and ir_value_5 < blind_spot_ir:
            direction_return_value = Direction.UP
        if ir_value_6 < blind_spot_ir and ir_value_3 < blind_spot_ir:
            direction_return_value = Direction.LEFT
        if ir_value_2 < blind_spot_ir and ir_value_5 < blind_spot_ir:
            direction_return_value = Direction.RIGHT 

    elif number_of_on_irs >= 3: 
        if ir_value_1 < blind_spot_ir and ir_value_4 < blind_spot_ir and ir_value_2 < blind_spot_ir: 
            direction_return_value = Direction.RIGHT_DOWN_CORNER
        elif ir_value_1 < blind_spot_ir and ir_value_4 < blind_spot_ir and ir_value_6 < blind_spot_ir:
            direction_return_value = Direction.LEFT_DOWN_CORNER
        elif ir_value_3 < blind_spot_ir and ir_value_5 < blind_spot_ir and ir_value_6 < blind_spot_ir:
            direction_return_value = Direction.LEFT_CORNER
        elif ir_value_2 < blind_spot_ir and ir_value_5 < blind_spot_ir and ir_value_3 < blind_spot_ir: 
            direction_return_value = Direction.RIGHT_CORNER
    else:
        direction_return_value = None

    return direction_return_value, bool_return_value


def follow_wall(direction, theta_dot, robot_position): 
    global IS_LEFT
    if direction == Direction.UP: 
        if IS_LEFT == True: 
            move_robot(-ROBOT_SPEED, 0, theta_dot, robot_position[2])
        else: 
            move_robot(ROBOT_SPEED, 0, theta_dot, robot_position[2])
    elif direction == Direction.DOWN: 
        if IS_LEFT == True: 
            move_robot(ROBOT_SPEED, 0, theta_dot, robot_position[2])
        else: 
            move_robot(-ROBOT_SPEED, 0, theta_dot, robot_position[2])
    elif direction == Direction.RIGHT: 
        move_robot(0, ROBOT_SPEED, theta_dot, robot_position[2])
    elif direction == Direction.LEFT:
        move_robot(0, -ROBOT_SPEED, theta_dot, robot_position[2])
    elif direction == Direction.CURVE_RIGHT:
        counter = 0
        while robot.step(TIME_STEP) != -1 and counter <= 275: 
            if counter <= 120: 
                move_robot(-ROBOT_SPEED, 0, theta_dot, robot_position[2])
            elif counter <= 250: 
                move_robot(0, ROBOT_SPEED, theta_dot, robot_position[2])
            else: 
                move_robot(ROBOT_SPEED, 0, theta_dot, robot_position[2])
            counter += 1
    elif direction == Direction.CUREVE_DOWN: 
        counter = 0
        while robot.step(TIME_STEP) != -1 and counter <= 275: 
            if counter <= 120: 
                #RIGHT 
                move_robot(ROBOT_SPEED, 0, theta_dot, robot_position[2])
            elif counter <= 250: 
                #UP 
                move_robot(0, -ROBOT_SPEED, theta_dot, robot_position[2])
            else:
                #LEFT 
                move_robot(-ROBOT_SPEED, 0, theta_dot, robot_position[2])
            counter += 1
    elif direction == Direction.CURVE: 
        counter = 0
        while robot.step(TIME_STEP) != -1 and counter <= 275: 
            if counter <= 120: 
                #RIGHT 
                move_robot(ROBOT_SPEED, 0, theta_dot, robot_position[2])
            elif counter <= 250: 
                #UP
                move_robot(0, ROBOT_SPEED, theta_dot, robot_position[2])
            else:
                #LEFT
                move_robot(-ROBOT_SPEED, 0, theta_dot, robot_position[2])
            counter += 1
    elif direction == Direction.LEFT_CORNER: 
        # down
        print('are we here?')
        move_robot(0, -ROBOT_SPEED, theta_dot, robot_position[2])
    elif direction == Direction.RIGHT_CORNER: 
        # left
        move_robot(-ROBOT_SPEED, 0, theta_dot, robot_position[2]) 
    elif direction == Direction.LEFT_DOWN_CORNER:  
        pass 
    elif direction == Direction.RIGHT_DOWN_CORNER: 
        print("mirim balaaa")
        # up 
        move_robot(0, ROBOT_SPEED, theta_dot, robot_position[2])  

def euclid_distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)



if __name__ == "__main__":

    robot = init_robot(time_step=TIME_STEP)
    goal_postition = np.array([0,0])
    state: StatesEnum = StatesEnum.MOVE_TOWARD_T

    iteration_bound = 0
    while robot.step(TIME_STEP) != -1:
        gps_values,compass_val,sonar_value,position_value,ir_value = read_sensors_values()
        x_current, y_current, _ = gps_values
        theta = math.atan2(Y_GOAL - gps_values[1], X_GOAL - gps_values[0]) * 180 / math.pi
        inertial_theta = get_bearing_in_degrees(compass_val)
        theta_dot = calculate_theta_dot(inertial_theta)
        if state == StatesEnum.MOVE_TOWARD_T:
            direction_temp, bool_temp = check_if_obstacle(ir_value, theta)
            if direction_temp and bool_temp: # if it detects an obstacle 
                state = StatesEnum.FOLLOW_BOUNDARY
            else:
                sin_theta = math.sin(theta)
                cos_theta = math.cos(theta)
                move_robot(ROBOT_SPEED * cos_theta, ROBOT_SPEED * sin_theta, theta_dot , robot_position[2])
        elif state == StatesEnum.FOLLOW_BOUNDARY:
            if euclid_distance(x_current, y_current, X_GOAL, Y_GOAL) <= EPSILON_LINE: 
                state = StatesEnum.STOP
            else:
                direction_temp, bool_temp = check_if_obstacle(ir_value, theta)
                print(bool_temp)
                if not direction_temp or not bool_temp:
                    state = StatesEnum.MOVE_TOWARD_T
                    IS_LEFT = not IS_LEFT
                else:
                    print('obstacle ', direction_temp)
                    follow_wall(direction_temp, theta_dot, robot_position)
        elif state == StatesEnum.STOP:
            move_robot(0, 0, 0, robot_position[2])

        
    pass



## (-10, 0) --> left 