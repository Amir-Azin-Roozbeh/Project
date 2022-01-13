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
blind_spot_ir = 999
X_GOAL = -0.270026
Y_GOAL = -1.35
THETA_ACCURACY_THRESH = 10
ROBOT_SPEED = 10
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
    UP_RIGHT_CORNER = 4
    UP_LEFT_CORNER = 5 
    DOWN_RIGHT_CORNER = 6 
    DOWN_LEFT_CORNER = 7
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

# def calc_theta_dot(heading, destination_theta) :
#     theta_dot = destination_theta - heading
#     if theta_dot > 180:
#         theta_dot = -(360-theta_dot)
#     elif theta_dot < -180:
#         theta_dot = (360+theta_dot)
#     return theta_dot

# def calculate_theta(x, y, compass_val):
#     heading = get_robot_heading(compass_val)
#     theta = math.atan2(Y_GOAL - y, X_GOAL - x) * 180 / math.pi
#     return calc_theta_dot(heading, theta)

def calculate_theta_dot(inertial_theta): 
    if inertial_theta >= 180: 
        inertial_theta = -(360 - inertial_theta)
    return inertial_theta

def check_if_obstacle(ir_value) -> List[Direction]:
    directions = []
    ir_value_1, ir_value_2, ir_value_3, ir_value_4, ir_value_5, ir_value_6 = ir_value
    if ir_value_5 < blind_spot_ir and ir_value_3 >= blind_spot_ir and ir_value_2 >= blind_spot_ir:
        return [Direction.UP_RIGHT_CORNER]
    if ir_value_3 < blind_spot_ir and ir_value_6 >= blind_spot_ir and ir_value_5 >= blind_spot_ir: 
        return [Direction.UP_LEFT_CORNER]
    if ir_value_1 < blind_spot_ir or ir_value_4 < blind_spot_ir:
        directions.append(Direction.DOWN)
    if ir_value_3 < blind_spot_ir or ir_value_5 < blind_spot_ir:
        directions.append(Direction.UP)
    if ir_value_6 < blind_spot_ir:
        directions.append(Direction.LEFT)
    if ir_value_2 < blind_spot_ir:
        directions.append(Direction.RIGHT)

    return directions




def follow_wall(obstacle_directions):
    number_of_obstacles = len(obstacle_directions)
    if number_of_obstacles == 1 : 
        if Direction.UP in obstacle_directions: 
            return Direction.RIGHT
        if Direction.DOWN in obstacle_directions: 
            return Direction.RIGHT
        if Direction.RIGHT in obstacle_directions: 
            return Direction.UP
        if Direction.LEFT in obstacle_directions: 
            return Direction.DOWN
        return obstacle_directions[0]




def move(direction, theta_dot, robot_position): 
    if direction == Direction.UP: 
        move_robot(0, ROBOT_SPEED, theta_dot, robot_position[2])
    elif direction == Direction.DOWN: 
        move_robot(0, -ROBOT_SPEED, theta_dot, robot_position[2])
    elif direction == Direction.RIGHT: 
        move_robot(ROBOT_SPEED, 0, theta_dot, robot_position[2])
    elif direction == Direction.LEFT:
        move_robot(-ROBOT_SPEED, 0, theta_dot, robot_position[2])
    elif direction == Direction.UP_RIGHT_CORNER:
        counter = 0
        while robot.step(TIME_STEP) != -1 and counter <= 200: 
            if counter <= 120: 
                move_robot(-ROBOT_SPEED, 0, theta_dot, robot_position[2])
            else: 
                move_robot(0, ROBOT_SPEED, theta_dot, robot_position[2])
            counter += 1
    elif direction == Direction.UP_LEFT_CORNER: 
        counter = 0
        while robot.step(TIME_STEP) != -1 and counter <= 300: 
            if counter <= 100: 
                move_robot(ROBOT_SPEED, 0, theta_dot, robot_position[2])
            elif counter <= 250: 
                move_robot(0, ROBOT_SPEED, theta_dot, robot_position[2])
            else:
                move_robot(-ROBOT_SPEED, 0, theta_dot, robot_position[2])
            counter += 1


if __name__ == "__main__":

    robot = init_robot(time_step=TIME_STEP)
    goal_postition = np.array([0,0])
    state: StatesEnum = StatesEnum.MOVE_TOWARD_T
    # DEFINE STATES HERE!
    
    while robot.step(TIME_STEP) != -1:

        gps_values,compass_val,sonar_value,position_value,ir_value = read_sensors_values()
        
        x_current, y_current, _ = gps_values
        print(ir_value)
        theta = math.atan2(Y_GOAL - gps_values[1], X_GOAL - gps_values[0]) * 180 / math.pi
        inertial_theta = get_bearing_in_degrees(compass_val)
        theta_dot = calculate_theta_dot(inertial_theta)
        
        if state == StatesEnum.MOVE_TOWARD_T:
            if check_if_obstacle(ir_value): # if it detects an obstacle  
                state = StatesEnum.FOLLOW_BOUNDARY
            else:
                sin_theta = math.sin(theta)
                cos_theta = math.cos(theta)
                move_robot(ROBOT_SPEED * cos_theta, ROBOT_SPEED * sin_theta, theta_dot , robot_position[2])
        elif state == StatesEnum.FOLLOW_BOUNDARY:
            if not check_if_obstacle(ir_value): 
                state = StatesEnum.MOVE_TOWARD_T
            else:
                obstacle_directions = check_if_obstacle(ir_value)
                print(obstacle_directions)
                move_direction = follow_wall(obstacle_directions)
                move(move_direction, theta_dot, robot_position)
        elif state == StatesEnum.STOP:
            ...

        # if ir_value_1 < blind_spot_ir: 
        #     move_robot(0, -10, 0, robot_position[2])
        # else : 
        #     move_robot(-10, 0, 0, robot_position[2])
        # update_robot_state(ir_value_1) 
        # move_robot('right') 
        # DEFINE STATE MACHINE HERE!

        # update_motor_speed(input_omega=[0,0,0])
        
    pass