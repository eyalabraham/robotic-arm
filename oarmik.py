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

    def get_positions(self, x, y, z):
        """Use Inverse Kinematics to calculate and return motor positions from (x,y,z) coordinates."""

        # Apply end-effector offsets to get true target position
        self.X = x - self.grip_x_offset
        self.Y = y
        self.Z = z + self.grip_z_offset

        # Calculate oArm geometry alpha/beta/gamma angles
        # see: https://sites.google.com/site/eyalabraham/robotic-arm
        self.q = math.sqrt(pow(self.X,2) + pow(self.Y,2))
        self.theta_turret = math.atan2(self.Y, self.X)
        
        try:
            self.alpha = math.acos((pow(self.X,2) + pow(self.Y,2) + pow(self.Z,2) - pow(self.arm_length,2) - pow(self.boom_length,2))/(-2*self.arm_length*self.boom_length))
        except ValueError:
            return 0,0,0,True
        except:
            print 'x={}, y={}, z={}'.format(x,y,z)
            raise
        
        self.theta_boom = 3.141592 - self.alpha
        self.beta = math.atan2((self.boom_length * math.sin(self.theta_boom)), (self.arm_length + self.boom_length * math.cos(self.theta_boom)))
        self.gamma = math.atan2(self.Z, self.q)
        self.theta_arm = 1.570796 - (self.beta + self.gamma)

        #print 'alpha {}, beta {}, gamma {}'.format(self.alpha, self.beta, self.gamma)
        #print 'theta_arm {}, theta_boom {}, theta_turret {}'.format(self.theta_arm, self.theta_boom, self.theta_turret)

        # Calculate stepper motor absolute positions
        self.arm = int(round(self.theta_arm * self.stepper_motor))
        self.boom = int(round((self.theta_boom - self.boom_offset_angle) * self.stepper_motor) + self.arm)
        self.turret = int(round(self.turret_offset + self.theta_turret * self.stepper_motor))

        # Limit checking
        # 'overrun' indicates that the (x,y,z) coordinates cannot be physically reached by the end effector
        self.overrun = False

        if self.arm < 0:
            self.arm = 0
            self.overrun = True

        elif self.arm > self.arm_limit:
            self.arm = self.arm_limit
            self.overrun = True

        if self.boom < 0:
            self.boom = 0
            self.overrun = True

        elif self.boom > self.boom_limit:
            self.boom = self.boom_limit
            self.overrun = True

        if self.turret < 0:
            self.turret = 0
            self.overrun = True

        elif self.turret > self.turret_limit:
            self.turret = self.turret_limit
            self.overrun = True

        # Return stepper motor positions and calculation status
        return self.arm, self.boom, self.turret, self.overrun

    def get_move_rates(self, a, b, t, start_a = 0, start_b = 0, start_t = 0):
        """Calculate and return movement rates for stepper motors."""

        # Calculate steps to move for each motor
        self.deltas = (abs(start_a - a),abs(start_b - b),abs(start_t - t))
        self.max_delta = max(self.deltas)

        # Calculate relative step rate per motor and output as list
        if self.max_delta > 0:
            return map(lambda x: (self.base_move_rate * x / self.max_delta), self.deltas)
        else:
            return 0,0,0
