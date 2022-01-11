# important points:
# use <robot_position> to get current position of robot in <x,y,theta> format.
# use <robot_omega> to get current values for the wheels in <w1,w2,w3> format.


import numpy as np
from initialization import * 


    
if __name__ == "__main__":
    
    TIME_STEP = 32
    robot = init_robot(time_step=TIME_STEP)
    init_robot_state(in_pos=[0,0,0],in_omega=[0,0,0]) 


    goal_postition = np.array([0,0])
    
    # DEFINE STATES HERE!

    while robot.step(TIME_STEP) != -1:

        gps_values,compass_val,sonar_value,encoder_value,ir_value = read_sensors_values()
        update_robot_state()       
        
        # DEFINE STATE MACHINE HERE!

        update_motor_speed(input_omega=[0,0,0])
        
    pass