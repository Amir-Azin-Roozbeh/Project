# important points:
# use <robot_position> to get current position of robot in <x,y,theta> format.
# use <robot_omega> to get current values for the wheels in <w1,w2,w3> format.

import numpy as np
from initialization import * 
from controller import GPS, Compass, DistanceSensor
import math
############################################# Variables
TIME_STEP = 32
GPS_SAMPLING_PERIOD = 1000
COMPASS_SAMPLING_PERIOD = 1000
blind_spot_ir = 1000
x_goal = -1.25
z_goal = 0.25
############################################ Util functions
def calc_distance_to_destination(robot):
    x, z = get_robot_coordinate(robot)
    return math.sqrt(((x_goal-x) ** 2) + ((z_goal-z) ** 2))

def get_bearing_in_degrees(compass_val):
    # calculate bearing angle in degrees 
    rad = math.atan2(compass_val[0], compass_val[2])
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

def calc_theta_dot(heading, destination_theta) :
    theta_dot = destination_theta - heading
    if theta_dot > 180:
        theta_dot = -(360-theta_dot)
    elif theta_dot < -180:
        theta_dot = (360+theta_dot)
    return theta_dot

def calculate_theta(x, y, compass_val):
    heading = get_robot_heading(compass_val)
    theta = math.atan2(y_goal - y, x_goal - x) * 180 / math.pi
    return calc_theta_dot(heading, theta)

if __name__ == "__main__":

    robot = init_robot(time_step=TIME_STEP)
    goal_postition = np.array([0,0])
    # DEFINE STATES HERE!

    while robot.step(TIME_STEP) != -1:

        gps_values,compass_val,sonar_value,position_value,ir_value = read_sensors_values()
        ir_value_1, ir_value_6, ir_value_3, ir_value_5, ir_value_2, ir_value_4 = ir_value
       

        robot_position = update_robot_state()
        if ir_value_1 < blind_spot_ir: 
            move_robot(0, -10, 0, robot_position[2])
        else : 
            move_robot(-10, 0, 0, robot_position[2])
        # update_robot_state(ir_value_1) 
        # move_robot('right') 
        # DEFINE STATE MACHINE HERE!

        # update_motor_speed(input_omega=[0,0,0])
        
    pass