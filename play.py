#!/usr/bin/python
###############################################################################
# 
# play.py
#
#   Robotic arm movement script playback.
#   Reads robotic arm commands from a test file script and sends them to
#   robotic arm controller via serial connection.
#
#   Script command line parameters:
#       play.py -s <script>
#
#   The script can include the following commands:
#   
#   - Everything after ';' is treated as a comment
#   - Empty lines are ignored
#   - The script is a serial list of commands
#   - Robotic arm controller CLI:
#      home all|arm|boom|turret|grip|rotate  - home positions
#      move <motor> <rate> <dir> <step>      - move relative steps
#      go <motor> <rate> <position>          - go to absolute position
#      stop                                  - stop all motors
#   - 'wait' wait for all arm actuators to stop moving
#   - 'pause <time>' pause execution of script for <time> seconds or until
#                    user hits <CR> if <time> is 0
#   - 'alias <name> <value>' define an alias for a value
#
#   June 21, 2018
#
###############################################################################

import serial
import sys
import time

#
# Global definitions
#

# Serial port definitions
SER_PORT = '/dev/ttyS4'     # TODO Move to command line parameter(s)
BAUD_RATE = 19200
STOP_BIT = 1
PARITY_BITS = 'N'
DATA_BITS = 8
TIME_OUT = 0.5

###############################################################################
#
# main()
#
def main():
    """Read command list from text file and send to robot arm controller"""
    
    #
    # Handle command line parameters
    #
    if len(sys.argv) != 3 or sys.argv[1] != '-s':
        sys.exit('Usage: play.py -s <script>')
    
    SCRIPT_FILE = sys.argv[2]
    
    # Define a dictionary for script aliases
    aliases = {}
    
    #
    # Open serial connection
    #
    ser = serial.Serial(SER_PORT, baudrate=BAUD_RATE, timeout=TIME_OUT)
    print 'Serial port:{} open:{}'.format(ser.name, ser.is_open)
    
    #
    # Test response from robot arm controller
    # and change robot arm controller mode to 'remote'
    #
    result = send_cmd(ser, "mode remote")
    if result == -1:
        sys.exit('err: Controller time out!')
    
    #
    # Open script file and process commands
    #
    with open(SCRIPT_FILE) as script:
        for line in script:

            # Remove blanks from start of line
            # Remove blanks and new-line from end of line
            cmd = line.strip('\r\n\t ')
            
            # check for ';' and for empty line and skip            
            if len(cmd) == 0:
                continue
                
            if cmd[0] == ';':
                continue
                
            # Split line to tokens and process first word that is the command
            cmd_tokens = cmd.split()
            
            #
            # Get positions and wait for all actuators to stop
            #
            if cmd_tokens[0] == 'wait':
                print '     Waiting.'
                active = True
                
                while active:
                    p = get_arm_position(ser)
                    active = False
                    for motor in range(5):
                        if int(p[motor][2]) != 0:
                            active = True

                continue
             
            #   
            # Pause for requested seconds or prompt user if 0
            #
            elif cmd_tokens[0] == 'pause':
                wait_time = int(cmd_tokens[1])
                if wait_time == 0:
                    dummy = raw_input('     Paused. hit [ENTER] to resume...')
                else:
                    print '     Pausing for {}sec.'.format(wait_time)
                    time.sleep(wait_time)        
                continue
            
            #
            # Build a dictionary of aliases
            # (no error checking!)
            #
            elif cmd_tokens[0] == 'alias':
                aliases[cmd_tokens[1]] = cmd_tokens[2]
                continue
            
            #
            # For all other commands send the command to controller
            # and wait for '>' prompt to signal command accepted
            #
            
            # First try to replace aliases
            if (cmd_tokens[0] == 'go' or cmd_tokens[0] == 'move') and cmd_tokens[1] in aliases:
                cmd = cmd.replace(cmd_tokens[1], aliases[cmd_tokens[1]])
            
            # Send command to controller
            if cmd_tokens[0] == 'home':
                result = send_cmd(ser, cmd, 20)
            else:
                result = send_cmd(ser, cmd)
                
            if result == -1:
                sys.exit('err: Controller time out or command error!')
  
    # Restore interactive mode and exit        
    result = send_cmd(ser, "mode interactive")
    if result == -1:
        sys.exit('err: Controller time out!')
        
    ser.close()
    
###############################################################################
#
# send_cmd()
#
def send_cmd(serial_line, cmd_string, time_out=1):
    """
    Send a command string to controller and block until controller returns with prompt or times out.
    Return 0 if prompt ok, -1 if time-out is reached before prompt.
    """
    
    print 'cmd: {}'.format(cmd_string)
    send_bytes = bytearray()
    send_bytes.extend((cmd_string + '\r'))
    serial_line.write(send_bytes)
    
    # Wait for prompt or time-out
    start = time.time()
    
    while True:
        pr = serial_line.read(3)
        
        if pr.strip('\r\n\t ') == '>':
            return 0
            
        end = time.time()
        if ( end - start > time_out ):
            break
            
    return -1

###############################################################################
#
# get_arm_position()
#
#   Position input format from 'get pos' command is one line per motor
#   and comma separated values: motor,rate,dir,curr_pos,limit_high,limit_low
#
#   Sample:
#       0,300,1,143,1552,0
#       1,0,0,0,500,0
#       2,0,0,0,950,0
#       3,0,0,165,250,80
#       4,0,0,165,165,90
#
def get_arm_position(serial_line):
    """Read and return motor positions and run status from robotic arm controller"""
    
    serial_line.write(b'get pos\r')
    serial_line.readline()  # Get rid of command's CR-LF echo
    
    a = []
    for i in range(5):
        pos = serial_line.readline().strip('\r\n\t ').split(',')
        a.append(pos)
        
    while True:
       pr = serial_line.read(3)    
       if pr.strip('\r\n\t ') == '>':
           return a
           
    return None

#
# Startup
#
if __name__ == '__main__':
    main()
