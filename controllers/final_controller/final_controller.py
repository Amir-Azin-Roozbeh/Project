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

# def update_robot_state(ir_value_1):
    
#     print(ir_value_1)
#     if ir_value_1 < blind_spot_ir :
#         print('in if')
#         move_robot('stop')
#     else : 
#         move_robot('forward')

def get_bearing_in_degrees(north):
    # calculate bearing angle in degrees 
    rad = math.atan2(north[0], north[2])
    bearing = (rad - 1.5708) / math.pi * 180.0
    if bearing < 0.0:
        bearing = bearing + 360.0
    return bearing

def move_robot_inertial(x, y, theta, heading):
    rotation_matrix = np.array([[np.cos(heading), np.sin(heading), 0], [-np.sin(heading), np.cos(heading), 0], [0, 0, 1]])
    temp = np.dot(rotation_matrix, np.array([x, y, theta]))
    return move_robot(temp[0], temp[1], temp[2])


def move_robot(x, y, theta): 
    # moves = {
        # 'stop': [0, 0, 0], 
        # 'rotate_right': [1, 1, 1], 
        # 'rotate_left': [-1, -1, -1], 
        # 'forward': [-2, 2, 0], 
        # 'left': [0, 2, -2], 
        # 'right': [2, -2, -4], 
        # 'back': [2, -2, 0]
    # }
    
    inverse_matrix = np.array([[-0.33, 0.58, 0.33], [-0.33, -0.58, 0.33], [0.67, 0, 0.33]])
    speed = np.matmul(inverse_matrix, np.array([x, y, theta]))
    update_motor_speed(speed)
    

if __name__ == "__main__":

    robot = init_robot(time_step=TIME_STEP)
    goal_postition = np.array([0,0])
    
    # DEFINE STATES HERE!

    while robot.step(TIME_STEP) != -1:

        gps_values,compass_val,sonar_value,position_value,ir_value = read_sensors_values()
        ir_value_1, ir_value_6, ir_value_3, ir_value_5, ir_value_2, ir_value_4 = ir_value
        print(get_bearing_in_degrees(compass_val))
       

        robot_position = update_robot_state()
        move_robot_inertial(-10, 0, 0, robot_position[2])
        # update_robot_state(ir_value_1) 
        # move_robot('right') 
        # DEFINE STATE MACHINE HERE!

        # update_motor_speed(input_omega=[0,0,0])
        
    pass