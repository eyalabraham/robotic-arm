###########################################################
#
# oarmop.py
#
#   Robotic arm operations class. This class is used to control
#   the arm motors and movement. The class implements the
#   communication and command protocol needed to connect to
#   the robotic arm controller and send movement commands to it.
#   The class does not include the Inverse Kinematics model.
#
#   July 12, 2018
#
###########################################################

import serial
import sys
import time

class OARMOP(object):
    """Implements communication and command protocol to the robotic arm controller."""

    def __init__(self, ser_port = '/dev/ttyS4', baud = 19200):
        """Initialize class variables from controller's config.h header file."""

        # Motor IDs
        self.TURRET_STEPPER = 0
        self.ARM_STEPPER = 1
        self.BOOM_STEPPER = 2
        self.GRIP_ROTATE_SERVO = 3
        self.GRIP_SERVO = 4

        # in [steps]
        self.TURRET_OFFSET = 750
        self.ARM_LIMIT = 600
        self.BOOM_LIMIT = 950
        self.TURRET_LIMIT = 1500

        # in [step/rad]
        self.STEPPER_MOTOR = 652
        self.SERVO_MOTOR = 64

        # Gripper definitions
        self.SERVO_MID_POINT = 165
        self.SERVO_MIN_POINT = 80         # abs. min 78
        self.SERVO_MAX_POINT = 250        # abs. max 275
        self.GRIP_ROTATE_MAX_CW = self.SERVO_MIN_POINT
        self.GRIP_ROTATE_MAX_CCW = self.SERVO_MAX_POINT
        self.GRIP_OPEN_MAX = self.SERVO_MID_POINT
        self.GRIP_CLOSE_MAX = (self.SERVO_MIN_POINT+15)
        self.GRIP_SERVO_RATE = 30

        # Serial port definitions
        self.__ser = None
        self.SER_PORT = ser_port
        self.BAUD_RATE = baud
        self.STOP_BIT = 1
        self.PARITY_BITS = 'N'
        self.DATA_BITS = 8
        self.TIME_OUT = 0.5

        # Open serial communication and set controller mode
        self.__ser = serial.Serial(self.SER_PORT, baudrate=self.BAUD_RATE, timeout=self.TIME_OUT)
        print '     Serial port:{} open:{}'.format(self.__ser.name, self.__ser.is_open)

        result = self.send_cmd('mode remote')
        if result == -1:
            sys.exit('err: Controller time out!')
        else:
            print '     Controller mode set to: remote'

        result = self.__get_version()
        if result == None:
            sys.exit('err: Controller time out!')
        else:
            print '     Version: ', result

    def __enter__(self):
        return self
        
    def __exit__(self, exception_type, exception_value, traceback):
        """Reset controller mode to interactive and close serial communication."""

        # Restore interactive mode and exit
        self.__ser.write(b'mode interactive\r')
        while True:
            pr = self.__ser.read(3)
            if pr.strip('\r\n\t ') == '>':
                break

        # TODO For some reason the code below throws an exception:
        #      NameError: global name 'send_cmd' is not defined
        #
        #result = send_cmd('mode interactive')
        #if result == -1:
        #    sys.exit('err: Controller time out!')
        #else:
        #    print '     Controller mode set to: interactive'

        self.__ser.close()

        print '     Serial port closed.'

    def __get_version(self):
        """Read and return robotic arm controller version string."""

        self.__ser.write(b'version\r')
        self.__ser.readline()  # Get rid of command's CR-LF echo

        ver = self.__ser.readline().strip('\r\n\t ')

        while True:
            pr = self.__ser.read(3)
            if pr.strip('\r\n\t ') == '>':
                return ver

        return None

    def __get_position_matrix(self):
        """
        Read and return motor positions and run-status from robotic arm controller.
        Position input format uses controller command 'get pos' as one line per motor
        with comma separated values: motor,rate,dir,curr_pos,limit_high,limit_low
        Sample:
            0,300,1,143,1500,0
            1,0,0,0,500,0
            2,0,0,0,950,0
            3,0,0,165,250,80
            4,0,0,165,165,90
        """

        self.__ser.write(b'get pos\r')
        self.__ser.readline()  # Get rid of command's CR-LF echo

        a = []
        for i in range(5):
            pos = self.__ser.readline().strip('\r\n\t ').split(',')
            a.append(pos)

        while True:
            pr = self.__ser.read(3)
            if pr.strip('\r\n\t ') == '>':
                return a

        return None

    def get_arm_positions(self, wait = True):
        """Read and return motor positions. Wait, block, for movement to stop unless 'wait' is False."""

        if wait == True:
            # Wait for movement completion
            active = True
            while active:
                p = self.__get_position_matrix()
                active = False
                for motor in range(5):
                    if int(p[motor][2]) != 0:
                        active = True

        # Final read of position and return position tuple
        p = self.__get_position_matrix()
        return int(p[0][3]), int(p[1][3]), int(p[2][3]), int(p[3][3]), int(p[4][3])

    def send_cmd(self, cmd_string, time_out = 1):
        """
        Send a command string to the controller and block until it returns with prompt or times out.
        Return 0 if prompt ok, -1 if time-out is reached before prompt.
        """

        print 'cmd: {}'.format(cmd_string)
        send_bytes = bytearray()
        send_bytes.extend((cmd_string + '\r'))
        self.__ser.write(send_bytes)

        # Wait for prompt or time-out
        start = time.time()

        while True:
            pr = self.__ser.read(3)

            if pr.strip('\r\n\t ') == '>':
                return 0

            end = time.time()
            if ( end - start > time_out ):
                break

        return -1

    def wait(self):
        """Read arm movement status and wait (block) for all motors to stop."""
        
        active = True

        while active:
            p = self.__get_position_matrix()
            active = False
            for motor in range(5):
                if int(p[motor][2]) != 0:
                    active = True

    def move_to(self, a = 0, ar = 0, b = 0, br = 0, t = 0, tr = 0, r = 0, rr = 0, wait = True, grip_rotator_sync = False):
        """
        Move [a]rm, [b]oom, [t]urret, and grip [r]otator to absolute positions.
        A zero value indicated a 'no change' in position.
        Always wait, block, for movement to complete before returning unless 'wait' is False.
        'grip_rotator_sync' allows syncing grip rotation with turret rotation rate.
        """

        # A precaution against sending two move_to() commands without waiting
        self.send_cmd('stop')
        
        # Move ARM command string if inputs are valid
        if a > 0 and ar > 0:
            arm_move = 'go {} {} {}'.format(self.ARM_STEPPER, ar, a)
        else:
            arm_move = ''

        # Move BOOM command string if inputs are valid
        if b > 0 and br > 0:
            boom_move = 'go {} {} {}'.format(self.BOOM_STEPPER, br, b)
        else:
            boom_move = ''

        # Move TURRET command string if inputs are valid
        if t > 0 and tr > 0:
            turret_move = 'go {} {} {}'.format(self.TURRET_STEPPER, tr, t)
        else:
            turret_move = ''

        # Move GRIP ROTATOR command string if inputs are valid
        if r > 0 and rr > 0:
            grip_rotator_move = 'go {} {} {}'.format(self.GRIP_ROTATE_SERVO, rr, r)
        else:
            grip_rotator_move = ''

        # Send movement command strings
        result = self.send_cmd(arm_move)
        if result == -1:
            return result

        result = self.send_cmd(boom_move)
        if result == -1:
            return result

        result = self.send_cmd(turret_move)
        if result == -1:
            return result

        result = self.send_cmd(grip_rotator_move)
        if result == -1:
            return result

        if wait == True:
            self.wait()
            
        return 0

    def set_gripper(self, g = 95):
        """Set gripper to absolute position."""
        
        cmd = 'go {} {} {}'.format(self.GRIP_SERVO, self.GRIP_SERVO_RATE, g)

        result = self.send_cmd(cmd)
        
        return result
    
    def home(self, m = 'all'):
        """Set a motor to home position"""
        
        cmd = 'home ' + m
        result = self.send_cmd(cmd, time_out = 30)
        
        return result
        
