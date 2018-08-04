###########################################################
#
# oarmik.py
#
#   Inverse Kinematics for oArm robotic arm that is based
#   on the classic open source uArm.
#   The geometry of the arm is described here: https://sites.google.com/site/eyalabraham/robotic-arm
#   Other resource: https://robotacademy.net.au/lesson/inverse-kinematics-for-a-2-joint-robot-arm-using-geometry/
#
#   July 7, 2018
#
###########################################################

import math

class OARMIK(object):
    """Implements the Inverse Kinematics model for the robotic arm."""

    def __init__(self, base_rate = 250):
        """Initialize the Inverse Kinematics model."""

        # in [mm]
        self.boom_length = 160.0
        self.arm_length = 148.0
        self.grip_x_offset = 30.0
        self.grip_z_offset = -17.0

        # in [rad]
        self.boom_offset_angle = 1.343857

        # in [step/rad]
        self.stepper_motor = 652
        self.servo_motor = 64

        # in [Hz]
        self.base_move_rate = base_rate

        # in [steps]
        self.turret_offset = 750
        self.arm_limit = 750
        self.boom_limit = 950
        self.turret_limit = 1500
        self.grip_rotator_max = 250
        self.grip_rotator_center = 165
        self.grip_rotator_min = 80

    def get_positions(self, x, y, z, grip_rotation = 0.0):
        """Use Inverse Kinematics to calculate and return motor positions from (x,y,z) grip rotation in [deg] coordinates."""

        # Apply end-effector offsets to get true target position
        X = x - self.grip_x_offset
        Y = y
        Z = z + self.grip_z_offset

        # Calculate oArm geometry alpha/beta/gamma angles
        # see: https://sites.google.com/site/eyalabraham/robotic-arm
        q = math.sqrt(pow(X,2) + pow(Y,2))
        theta_turret = math.atan2(Y, X)
        
        try:
            alpha = math.acos((pow(X,2) + pow(Y,2) + pow(Z,2) - pow(self.arm_length,2) - pow(self.boom_length,2))/(-2*self.arm_length*self.boom_length))
        except ValueError:
            return 0,0,0,0,True
        except:
            print 'Exception with: x={}, y={}, z={}'.format(x,y,z)
            raise
        
        theta_boom = 3.141592 - alpha
        beta = math.atan2((self.boom_length * math.sin(theta_boom)), (self.arm_length + self.boom_length * math.cos(theta_boom)))
        gamma = math.atan2(Z, q)
        theta_arm = 1.570796 - (beta + gamma)

        #print 'alpha {}, beta {}, gamma {}'.format(self.alpha, self.beta, self.gamma)
        #print 'theta_arm {}, theta_boom {}, theta_turret {}'.format(self.theta_arm, self.theta_boom, self.theta_turret)

        # Calculate stepper motor absolute positions
        arm = int(round(theta_arm * self.stepper_motor))
        boom = int(round((theta_boom - self.boom_offset_angle) * self.stepper_motor) + arm)
        turret = int(round(self.turret_offset + theta_turret * self.stepper_motor))

        # Limit checking
        # 'overrun' indicates that the (x,y,z) coordinates cannot be physically reached by the end effector
        overrun = False

        if arm < 0:
            arm = 0
            overrun = True

        elif arm > self.arm_limit:
            arm = self.arm_limit
            overrun = True

        if boom < 0:
            boom = 0
            overrun = True

        elif boom > self.boom_limit:
            boom = self.boom_limit
            overrun = True

        if turret < 0:
            turret = 0
            overrun = True

        elif turret > self.turret_limit:
            turret = self.turret_limit
            overrun = True

        grip_rotator = self.get_rotator_position(grip_rotation)
        
        if grip_rotator < self.grip_rotator_min:
            grip_rotator = self.grip_rotator_min
            
        elif grip_rotator > self.grip_rotator_max:
            grip_rotator = self.grip_rotator_max
        
        # Return stepper motor positions and calculation status
        return arm, boom, turret, grip_rotator, overrun

    def get_move_rates(self, a, b, t, r, start_a = 0, start_b = 0, start_t = 0, start_r = 0):
        """Calculate and return movement rates for stepper motors."""

        # Calculate steps to move for each motor
        deltas = (abs(start_a - a),abs(start_b - b),abs(start_t - t),abs(start_r - r))
        max_delta = max(deltas)

        # Calculate relative step rate per motor and output as list
        if max_delta > 0:
            return map(lambda x: (self.base_move_rate * x / max_delta), deltas)
        else:
            return 0,0,0,0
            
    def get_rotator_position(self, a = 0.0):
        """Calculate and return grip rotator position for requested angle (in deg)."""
        
        a_rad = a * 3.141592 / 180.0
        
        pos = int(a_rad * self.servo_motor) + self.grip_rotator_center
        
        return pos
