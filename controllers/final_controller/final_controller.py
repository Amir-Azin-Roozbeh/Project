# important points:
# use <robot_position> to get current position of robot in <x,y,theta> format.
# use <robot_omega> to get current values for the wheels in <w1,w2,w3> format.

import numpy as np
from initialization import * 
from controller import GPS, Compass, DistanceSensor
#############################################Variables
TIME_STEP = 32
GPS_SAMPLING_PERIOD = 1000
COMPASS_SAMPLING_PERIOD = 1000
state = 'move_forward'
blind_spot_ir = 1000
############################################Util functions
def calc_distance_to_destination(robot):
    x, z = get_robot_coordinate(robot)
    return math.sqrt(((x_goal-x) ** 2) + ((z_goal-z) ** 2))

# def update_robot_state():
    

def move_robot(direction): 
    moves = {
        'stop': [0, 0, 0], 
        'rotate_right': [1, 1, 1], 
        'rotate_left': [-1, -1, -1], 
        'forward': [-2, 2, 0], 
        'left': [-2, 2, 4], 
        'right': [2, -2, -4], 
        'back': [2, -2, 0]
    }
    
    print(direction)
    update_motor_speed(moves[direction])
    

if __name__ == "__main__":

    robot = init_robot(time_step=TIME_STEP)
    goal_postition = np.array([0,0])
    
    # DEFINE STATES HERE!

    while robot.step(TIME_STEP) != -1:

        gps_values,compass_val,sonar_value,position_value,ir_value = read_sensors_values()
        print(ir_value)
        update_robot_state()  
        move_robot('left')     
        # DEFINE STATE MACHINE HERE!

        # update_motor_speed(input_omega=[0,0,0])
        
    pass