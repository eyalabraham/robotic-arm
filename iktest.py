#!/usr/bin/python
###########################################################
#
# iktest.py
#
#   Test Inverse Kinematics model and arm movement classes.
#   Get x,y,z coordinate from use and move arm to position.
#
#   July 21, 2018
#
###########################################################

import oarmik as ik
import oarmop

###############################################################################
#
# main()
#
def main():
    """Get x,y,z coordinate from use and move arm to position."""

    print '     When prompted enter position as x,y,z in [mm]'
    print '     To exit enter: -1,-1,-1'

    # In [mm]
    position = [0,0,0]
    start_position = [0,0,0]
    end_position = [0,0,0]
    
    rotation = 0.0
    
    # in [step/sec]
    rates = [0,0,0]
    
    # Some constants
    LEGO_WIDTH = 108
    
    model = ik.OARMIK()

    with oarmop.OARMOP() as oArm:
    
        oArm.home()
        
        oArm.set_gripper(LEGO_WIDTH)
        
        while True:
        
            # Input coordinate from user
            position = input('     Enter position as x, y, z : ')
        
            # A way to exit the script
            if position == (-1,-1,-1):
                break
                
            # Special coordinates to 'home' the arm
            elif position == (0,0,0):
                oArm.home()
                position = [0,0,0]
                rotation = 0.0
                start_position = [0,0,0]
                end_position = [0,0,0]
                rates = [0,0,0]
                oArm.set_gripper(LEGO_WIDTH)
                continue
                
            rotation = input('     Enter grip rotation : ')

            # Calculate motor positions with Inverse Kinematics model
            end_position = position
            print '     Moving from: ', start_position, 'to: ', end_position
            
            arm, boom, turret, grip_rotator, overrun = model.get_positions(position[0],position[1],position[2], rotation)
        
            # Check for overrun
            if overrun == True:
                print 'err: Overrun.'
                continue

            # Calculate relative movement rates
            motor_pos = oArm.get_arm_positions(wait = True)
            arm_rate, boom_rate, turret_rate, grip_rotator_rate = model.get_move_rates(arm, boom, turret, grip_rotator, motor_pos[1], motor_pos[2], motor_pos[0], motor_pos[3])
     
            # Move arm to position, wait for completion of movement
            oArm.move_to(arm, arm_rate, boom, boom_rate, turret, turret_rate, grip_rotator, grip_rotator_rate)
            
            start_position = end_position

#
# Startup
#
if __name__ == '__main__':
    main()
