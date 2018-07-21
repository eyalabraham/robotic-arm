#!/usr/bin/python
###############################################################################
#
# play2.py
#
#   Robotic arm movement script playback using the 'oarmop' class.
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

import oarmop as arm
import sys
import time

#
# Global definitions
#

# Serial port definitions
SER_PORT = '/dev/ttyS4'     # TODO Move to command line parameter(s)
BAUD_RATE = 19200

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
    # Open script file and process commands
    #
    with open(SCRIPT_FILE) as script, arm.OARMOP() as oArm:
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
                oArm.wait()

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
                result = oArm.home(cmd_tokens[1])
            else:
                result = oArm.send_cmd(cmd)

            if result == -1:
                sys.exit('err: Controller time out or command error!')

#
# Startup
#
if __name__ == '__main__':
    main()
