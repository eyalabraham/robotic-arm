/* 
 * o-arm.c
 *
 *  This program is the control program for a 5-DoF robot arm modeled after uARM. 
 *  The robot is managed through by an ATmega328P, controller through RS232.
 *  The ATmega328P AVR performs these tasks:
 *   (1) stepper motor commands to 3 stepper motors (arm, boom, turret)
 *   (2) servo command to gripper (rotate, open/close)
 *   (3) limit switch inputs
 *   (5) {optional} G-Code interpreter
 * 
 *  +-----+             +-----+
 *  |     |             |     |
 *  | PC  |             | AVR |
 *  |     +--< UART >---+     |
 *  |     |             |     |
 *  |     |             |     |
 *  |     |             |     |
 *  +-----+             +--+--+
 *                         |
 *               2x PWM channels to gripper servos
 *               3x limit micro switches
 *               12bit IO to 3 stepper motor drivers
 *               {optional} miscellaneous indicators/switches
 *
 * ATmega AVR IO
 *
 * | Function            | AVR       | Pin            | I/O |
 * |---------------------|-----------|----------------|-----|
 * | Gripper rotate      | OCR1B     | pin 16         | out |
 * | Gripper open/close  | OCR1A     | pin 15         | out |
 * | Turret stepper      | PD4..PD7  | pin 6,11..13   | out |
 * | Arm stepper         | PB4..PB7  | pin 18,19,9,10 | out |
 * | Boom stepper        | PC0..PB3  | pin 23..26     | out |
 * | Turret limit switch | PD2       | pin 4          | in  |
 * | Arm limit switch    | PC4       | pin 27         | in  |
 * | Boom limit switch   | PC5       | pin 28         | in  |
 * | UART                | RXD       | pin 2          | in  |
 * | UART                | TXD       | pin 3          | out |
 * | LED                 | PD3       | pin 5          | out |
 * | RESET               | PC6       | pin 1          | in  |
 *
 * Port B bit assignment
 *
 *  b7 b6 b5 b4 b3 b2 b1 b0
 *  |  |  |  |  |  |  |  |
 *  |  |  |  |  |  |  |  +--- 'i'
 *  |  |  |  |  |  |  +------ 'o' OCR1A gripper open/close servo PWM
 *  |  |  |  |  |  +--------- 'o' OCR1B gripper rotator servo PWM
 *  |  |  |  |  +------------ 'i'
 *  |  |  |  +--------------- 'o' \
 *  |  |  +------------------ 'o'  |
 *  |  +--------------------- 'o'  | arm stepper driver
 *  +------------------------ 'o' /
 *
 * Port C bit assignment
 *
 *     b6 b5 b4 b3 b2 b1 b0
 *     |  |  |  |  |  |  |
 *     |  |  |  |  |  |  +--- 'o' \
 *     |  |  |  |  |  +------ 'o'  |
 *     |  |  |  |  +--------- 'o'  | boom stepper driver
 *     |  |  |  +------------ 'o' /
 *     |  |  +--------------- 'i' arm limit switch
 *     |  +------------------ 'i' boom limit switch
 *     +--------------------- 'i' RESET (power-on and push button)
 *
 * Port D bit assignment
 *
 *  b7 b6 b5 b4 b3 b2 b1 b0
 *  |  |  |  |  |  |  |  |
 *  |  |  |  |  |  |  |  +--- 'i' UART Rx
 *  |  |  |  |  |  |  +------ 'o' UART Tx
 *  |  |  |  |  |  +--------- 'i' turret limit switch
 *  |  |  |  |  +------------ 'o' heart beat LED
 *  |  |  |  +--------------- 'o' \
 *  |  |  +------------------ 'o'  |
 *  |  +--------------------- 'o'  | turret stepper driver
 *  +------------------------ 'o' /
 *
 * Note: all references to data sheet are for ATmega 328P Rev. 8161D–AVR–10/09
 * 
 * Command line list see source or type 'help'<CR>
 *
 */

#include    <stdint.h>
#include    <stdlib.h>
#include    <string.h>
#include    <stdio.h>
#include    <stdarg.h>
#include    <math.h>

#include    <avr/pgmspace.h>
#include    <avr/io.h>
#include    <avr/interrupt.h>
#include    <avr/wdt.h>

#include    "config.h"
#include    "uart_drv.h"
#include    "utils.h"

// debug print to UART port definition
// UART is assumed to be defined and initialized
//#define     __DEBUG_PRINT__

#define     __FW_VERSION__  "1.0"

// IO ports B C and D initialization
#define     PB_DDR_INIT     0xf6    // port data direction
#define     PB_PUP_INIT     0x00    // port input pin pull-up
#define     PB_INIT         0x00    // port initial values

#define     PC_DDR_INIT     0x0f    // port data direction
#define     PC_PUP_INIT     0x30    // port input pin pull-up
#define     PC_INIT         0x00    // port initial values

#define     PD_DDR_INIT     0xfa    // port data direction
#define     PD_PUP_INIT     0x04    // port input pin pull-up
#define     PD_INIT         0x00    // port initial values

// misc masks
#define     HEART_BEAT      0x08    // toggle heart beat

// Timer0 configuration
#define     TIM0_CTRLA      0x02
#define     TIM0_CTRLB      0x02
#define     TIM0_INT_MASK   0x02
#define     SEQ_FREQ        20000L  // <------ change rate frequency in Hz
#define     PRE_SCALER      1000000L// 8MHz clock divided by 8 pre-scaler in TCCR0B
#define     TIM0_FREQ_CONST ((PRE_SCALER / SEQ_FREQ) -1)

// Timer1 configuration
#define     TIM1_CTRLA      0xa2
#define     TIM1_CTRLB      0x1b

// Timer1 frequency constants
#define     SERVO_PERIOD    2499    // 20mSec PWM period with Clock Select to Fclk/64

// UART command line processing
#define     CLI_BUFFER      80
#define     CR              0x0d    // carriage return
#define     LF              0x0a    // line feed
#define     BELL            0x07    // bell
#define     BS              0x08    // back space
#define     SPACE           0x20    // space
#define     MAX_TOKENS      5       // max number of command line tokens
#define     CMD_DELIM       " \t"   // command line white-space delimiters
#define     PROMPT          ">"

// CLI definitions
#define     SYNTAX_ERR      "syntax error.\n"
#define     SYN_ERR_PROMPT  "!"
#define     HELP_TEXT       "\n\
  help                                  - help text\n\
  home all|arm|boom|turret|grip|rotate  - home positions\n\
  get pos                               - print motor positions\n\
  move <motor> <rate> <dir> <step>      - move relative steps\n\
  go <motor> <rate> <position>          - go to absolute position\n\
  mode interactive|remote               - control mode\n\
  version                               - show version\n\
  stop                                  - stop all motors\n"

// Device definitions
#define     MOTOR_COUNT     5

/****************************************************************************
  Types
****************************************************************************/
struct motor_t
    {
        int rate;           // rate at which to change motor position command [1..MAX_STEP_RATE]
        int counter;        // motor is moved when counter == 0, decrement every time ISR runs
        int dir;            // direction, '-1' to low, '+1' to high, '0' stop
        int steps;          // steps to move, if '-1' then move until stopped or limit reached
        int curr_pos;       // current position
        int limit_high;     // high limit step count
        int limit_low;      // low limit step count
        int stepper_coil;   // stepper motor only: coil sequence index
    };

/****************************************************************************
  Function prototypes
****************************************************************************/
// This function is called upon a HARDWARE RESET:
void reset(void) __attribute__((naked)) __attribute__((section(".init3")));

void ioinit(void);
void delay(uint16_t);
int  process_cli(char*);
void home_turret(void);
void home_arm(void);
void home_boom(void);
void home_gripper_rorator(void);
void home_gripper(void);

/****************************************************************************
  Globals
****************************************************************************/
volatile     uint32_t timerTicks = 0;           // clock timer ticks, increment at PID_FREQ [Hz]
volatile int total_steps = 0;
int          nInteractive = 1;   // print prompt

struct motor_t motors[MOTOR_COUNT] =
    {
        {0, 0, 0, 0, 0, TURRET_MAX_STEP, 0, 0},                                    // 0 - Turret stepper
        {0, 0, 0, 0, 0, ARM_MAX_STEP, 0, 0},                                       // 1 - Arm stepper
        {0, 0, 0, 0, 0, BOOM_MAX_STEP, 0, 0},                                      // 2 - Boom stepper
        {0, 0, 0, 0, SERVO_MID_POINT, GRIP_ROTATE_MAX_CCW, GRIP_ROTATE_MAX_CW, 0}, // 3 - Grip rotator servo
        {0, 0, 0, 0, GRIP_OPEN_MAX, GRIP_OPEN_MAX, GRIP_CLOSE_MAX, 0}              // 4 - Grip servo
    };

/*  motor coil arrangement TEAC 14769070-60
 *
 *  One-Phase On sequence
 *      bit-3       bit-2       bit-1       bit-0
 *      white       blue        red         yellow
 *      [c2p2]      [c1p2]      [c2p1]      [c1p1]
 *        0           0           0           1         0x01
 *        0           0           1           0         0x02
 *        0           1           0           0         0x04
 *        1           0           0           0         0x08
 *
 *  Two-Phase On sequence
 *      source: http://www.haydonkerk.com/Resources/StepperMotorTheory/tabid/192/Default.aspx
 *      bit-3       bit-2       bit-1       bit-0
 *      white       blue        red         yellow
 *      [c2p2]      [c1p2]      [c2p1]      [c1p1]
 *        0           0           1           1         0x03
 *        0           1           1           0         0x06
 *        1           1           0           0         0x0c
 *        1           0           0           1         0x09
 */

/* UNIPOLAR */
//uint8_t motorChannel[4] = {0x01, 0x02, 0x04, 0x08}; // one-phase on, full step, CW
//uint8_t motorChannel[4] = {0x08, 0x04, 0x02, 0x01}; // one-phase on, full step, CCW

//uint8_t motorChannel[4] = {0x03, 0x06, 0x0c, 0x09}; // two-phase on, full step, CW
//uint8_t motorChannel[4] = {0x09, 0x0c, 0x06, 0x03}; // two-phase on, full step, CCW

//uint8_t motorChannel[4] = {0x03, 0x06, 0x0c, 0x09}; // two-phase on, half step, CW
//uint8_t motorChannel[4] = {0x09, 0x0c, 0x06, 0x03}; // two-phase on, half step, CCW

/* BIPOLAR */
//uint8_t motorChannelCW[4] = {0x08, 0x02, 0x04, 0x01}; // one-phase on, full step, CW
//uint8_t motorChannelCCW[4] = {0x01, 0x04, 0x02, 0x08}; // one-phase on, full step, CCW

uint8_t motorChannelCW[4] = {0x09, 0x0a, 0x06, 0x05}; // two-phase on, full step, CW
uint8_t motorChannelCCW[4] = {0x05, 0x06, 0x0a, 0x09}; // two-phase on, full step, CCW


/* ----------------------------------------------------------------------------
 * main() control functions
 *
 */
int main(void)
{
    static char     commandLine[CLI_BUFFER] = {0};
    static int      nCliIndex;
    static uint8_t  inChar;

    // initialize command line buffer
    memset(commandLine, 0, CLI_BUFFER);
    nCliIndex = 0;

    // initialize IO devices
    ioinit();

    // clear console and output message
    vprintfunc("%c[2J", 27);
    printstr_p(PSTR("o-arm.c AVR arm controller\n"));
    vprintfunc("build %s %s\n", __DATE__, __TIME__);

    // enable interrupts
    sei();

    // print command line prompt
    printstr_p(PSTR(PROMPT));

    // loop forever and scan for commands from RPi host
    while ( 1 )
    {
        // sample UART input and act on command line
        if ( uart_ischar() )
        {
            inChar = uart_getchr();
            switch ( inChar )
            {
                case CR:
                    uart_putchr(inChar);                        // output a CR-LF even in 'remote' mode
                    uart_putchr(LF);

                    if ( process_cli(commandLine) == -1 )       // -- process command --
                    {
                        if ( nInteractive )                     // output appropriate prompt
                        {
                            printstr_p(PSTR(SYNTAX_ERR));
                            printstr_p(PSTR(PROMPT));
                        }
                        else
                        {
                            printstr_p(PSTR(SYN_ERR_PROMPT));
                        }
                    }
                    else
                    {
                        printstr_p(PSTR(PROMPT));               // output a prompt if required
                    }
                    memset(commandLine, 0, CLI_BUFFER);         // reinitialize command line buffer for next command
                    nCliIndex = 0;
                    break;

                default:
                    {
                        if ( nCliIndex < CLI_BUFFER )
                        {
                            if ( inChar != BS )                 // is character a back-space?
                                commandLine[nCliIndex++] = inChar;  // no, then store in command line buffer
                            else if ( nCliIndex > 0 )           // yes, it is a back-space, but do we have characters to remove?
                            {
                                nCliIndex--;                    // yes, remove the character
                                commandLine[nCliIndex] = 0;
                                if ( nInteractive )
                                {
                                    uart_putchr(BS);
                                    uart_putchr(SPACE);
                                }
                            }
                            else
                                inChar = 0;                     // no, so do nothing
                        }
                        else
                            inChar = BELL;

                        if ( nInteractive )
                            uart_putchr(inChar);                // echo character to console
                    }
            }
        } /* UART character input */
    } /* endless while loop */

    return 0;
}

/* ----------------------------------------------------------------------------
 * reset()
 *
 *  Clear SREG_I on hardware reset.
 *  source: http://electronics.stackexchange.com/questions/117288/watchdog-timer-issue-avr-atmega324pa
 */
void reset(void)
{
     cli();
    // Note that for newer devices (any AVR that has the option to also
    // generate WDT interrupts), the watchdog timer remains active even
    // after a system reset (except a power-on condition), using the fastest
    // prescaler value (approximately 15 ms). It is therefore required
    // to turn off the watchdog early during program startup.
    MCUSR = 0; // clear reset flags
    wdt_disable();
}

/* ----------------------------------------------------------------------------
 * ioinit()
 *
 *  initialize IO interfaces
 *  timer and data rates calculated based on 4MHz internal clock
 *
 * initialize IO: UART, timer and IO pins
 * - timer0 provides 2 PWM signals for gripper servo control
 * - timer1 provides time base for stepper motor movement/synchronization
 * - IO pins to control 3 L293D H-bridge for 3 bipolar stepper motors, limit switches etc.
 * - UART to send/receive commands from host
 *
 */
void ioinit(void)
{
    // Reconfigure system clock scaler to 8MHz
    CLKPR = 0x80;   // change clock scaler (sec 8.12.2 p.37)
    CLKPR = 0x00;

    // initialize Timer0 to provide a periodic interrupt
    // with Clear Timer on Compare Match (CTC) Mode (sec 14.7.2 p.100)
    // the interrupt routine will drive stepper and servo PWM updates
    TCNT0  = 0;
    OCR0A  = TIM0_FREQ_CONST;
    TCCR0A = TIM0_CTRLA;
    TCCR0B = TIM0_CTRLB;
    TIMSK0 = TIM0_INT_MASK;     // interrupt on OCR0A match
    TIFR0  = 0;

    // Initialize Timer1 for servo PWM (see sec 15.9.3 Fast PWM Mode)
    // 20mSec pulse interval, with 1mSec to 2mSec variable pulse width
    // Fast PWN mode with Non-inverting Compare Output mode to clear output
    TCNT1  = 0;
    OCR1A  = SERVO_MID_POINT;
    OCR1B  = SERVO_MID_POINT;
    ICR1   = SERVO_PERIOD;
    TCCR1A = TIM1_CTRLA;
    TCCR1B = TIM1_CTRLB;
    TCCR1C = 0;
    TIMSK1 = 0;

    // initialize general IO PB and PD pins for output
    // pin directions
    // initial value of pins is '0', and input with pull-up
    DDRB  = PB_DDR_INIT;
    PORTB = PB_INIT | PC_PUP_INIT | (motorChannelCCW[0] << 4);

    DDRC  = PC_DDR_INIT;
    PORTC = PC_INIT | PC_PUP_INIT | motorChannelCW[0];

    DDRD  = PD_DDR_INIT;
    PORTD = PD_INIT | PD_PUP_INIT | (motorChannelCW[0] << 4);

    // initialize UART interface to 19200 BAUD, 8 bit, 1 stop, no parity
    uart_initialize();
}


/* ----------------------------------------------------------------------------
 * delay()
 *
 * Blocking delay function based on clock tick counter
 *
 * param:   msec
 * return:  nothing
 *
 */
void delay(uint16_t msec)
{
    uint32_t    delta_time;
    uint32_t    current_time;

    current_time = timerTicks;
    delta_time = (SEQ_FREQ / 1000) * msec;

    while ( (timerTicks - current_time) < delta_time );
}

/* ----------------------------------------------------------------------------
 * process_cli()
 *
 * process the command line text and execute appropriate action
 * return '-1' on syntax error, otherwise '0'
 *
 */
int process_cli(char *commandLine)
{
    char       *tokens[MAX_TOKENS] = {0, 0, 0, 0, 0};
    char       *token;
    char       *tempCli;
    int         numTokens;

    int         i, intTemp;

    // separate command line into tokens
    tempCli = commandLine;
    for ( numTokens = 0; numTokens < MAX_TOKENS; numTokens++, tempCli = NULL)
    {
        token = strtok(tempCli, CMD_DELIM);
        if ( token == NULL )
            break;
        tokens[numTokens] = token;
    }

    // if nothing found then this is an empty line, just exit
    if ( numTokens == 0 )
        return 0;
    
    /* Parse and execute commands starting with a
     * simple CLI 'help' printout
     */
    if ( strcmp(tokens[0], "help") == 0 )
    {
        printstr_p(PSTR(HELP_TEXT));
        return 0;
    }

    /* Move all to home position
     */
    else if ( strcmp(tokens[0], "home") == 0 )
    {
        if ( strcmp(tokens[1], "all") == 0 )
        {
            home_gripper();
            home_gripper_rorator();
            home_boom();
            home_arm();
            home_turret();
            return 0;
        }
        else if ( strcmp(tokens[1], "boom") == 0 )
        {
            home_boom();
            return 0;
        }
        else if ( strcmp(tokens[1], "arm") == 0 )
        {
            home_arm();
            return 0;
        }
        else if ( strcmp(tokens[1], "turret") == 0 )
        {
            home_turret();
            return 0;
        }
        else if ( strcmp(tokens[1], "grip") == 0 )
        {
            home_gripper();
            return 0;
        }
        else if ( strcmp(tokens[1], "rotate") == 0 )
        {
            home_gripper_rorator();
            return 0;
        }
    }

    /* Get motor positions from motor schedule table
     * and print out
     */
    else if ( strcmp(tokens[0], "get") == 0 && strcmp(tokens[1], "pos") == 0 )
    {
        if ( nInteractive )
            printstr_p(PSTR("motor rate dir curr_pos limit_high limit_low\n"));

        for ( i = 0; i < MOTOR_COUNT; i ++ )
        {
            if ( nInteractive )
            {
                vprintfunc("%5d %4d %3d %8d %10d %9d\n",
                           i,
                           motors[i].rate,
                           motors[i].dir,
                           motors[i].curr_pos,
                           motors[i].limit_high,
                           motors[i].limit_low);
            }
            else
            {
                vprintfunc("%d,%d,%d,%d,%d,%d\n",
                           i,
                           motors[i].rate,
                           motors[i].dir,
                           motors[i].curr_pos,
                           motors[i].limit_high,
                           motors[i].limit_low);
            }
        }

        return 0;
    }

    /* Get movement command and translate into
     * schedule table format to move a motor.
     * Motor start moving as soon as command is interpreted.
     * Range check errors will result in 'syntax error.'
     * Command assumes four parameters:
     *   move <motor> <rate> <dir> <step>
     */
    else if ( strcmp(tokens[0], "move") == 0 )
    {
        // Motor number should be in range
        intTemp = atoi(tokens[1]);
        if ( intTemp < 0 || intTemp >= MOTOR_COUNT )
            return -1;

        i = intTemp;

        intTemp = atoi(tokens[2]);
        if ( intTemp < 0 )
            return -1;
        else if ( intTemp > MAX_STEP_RATE )
            intTemp = MAX_STEP_RATE;

        motors[i].rate = (uint16_t)intTemp;

        intTemp = atoi(tokens[4]);
        if ( intTemp < -1 )
            return -1;
        else
            // The interrupt routine will range check motor position against limits
            motors[i].steps = intTemp;

        /* Process direction and update this last
         * because this change will start the motors
         */
        intTemp = atoi(tokens[3]);
        motors[i].dir = (intTemp > 0) ? 1 : ((intTemp < 0) ? -1 : 0);

        return 0;
    }

    /* Move to an absolute position
     * Calculate relative movement between current position
     * and requested absolute position, and set motor to move.
     * Out of range requests end up as movements to max positions.
     *  format: go <motor> <rate> <position>
     */
    else if ( strcmp(tokens[0], "go") == 0 )
    {
        // Motor number should be in range
        intTemp = atoi(tokens[1]);
        if ( intTemp < 0 || intTemp >= MOTOR_COUNT )
            return -1;

        i = intTemp;

        // Rate of movement
        intTemp = atoi(tokens[2]);
        if ( intTemp < 0 )
            return -1;
        else if ( intTemp > MAX_STEP_RATE )
            intTemp = MAX_STEP_RATE;

        motors[i].rate = (uint16_t)intTemp;

        /* Calculate relative steps and direction
         * from current position to requested absolute position
         */
        intTemp = atoi(tokens[3]);

        if ( intTemp > motors[i].limit_high )
            intTemp = motors[i].limit_high;
        if ( intTemp < motors[i].limit_low )
            intTemp = motors[i].limit_low;

        intTemp -= motors[i].curr_pos;
        motors[i].steps = abs(intTemp);

        /* Process direction and update this last
         * because this change will start the motors
         */
        motors[i].dir = (intTemp > 0) ? 1 : ((intTemp < 0) ? -1 : 0);

        return 0;
    }

    /* Switch between interactive mode and remote control mode.
     * Use 'remote' control for sending commands and receiving
     * responses through serial connection.
     */
    else if ( strcmp(tokens[0], "mode") == 0 )
    {
        if ( strcmp(tokens[1], "interactive") == 0 )
        {
            nInteractive = 1;
            return 0;
        }
        else if ( strcmp(tokens[1], "remote") == 0 )
        {
            nInteractive = 0;
            return 0;
        }
    }

    /* Display firmware version
     */
    else if ( strcmp(tokens[0], "version") == 0 )
    {
        vprintfunc("%s %s %s\n", __FW_VERSION__, __DATE__, __TIME__);
        return 0;
    }

    /* Stop all motors by setting the motor's 'dir' to 0
     * Motors will stop at the next invocation of the timer0 interrupt
     */
    else if ( strcmp(tokens[0], "stop") == 0 )
    {
        for ( i = 0; i < MOTOR_COUNT; i ++ )
        {
            motors[i].dir = 0;
        }
        return 0;
    }

    /* Hidden test command to exercise motor control
     */
/*
    else if ( strcmp(tokens[0], "test") == 0 )
    {
        // setup the the scheduling table for the gripper rotate servo
        motors[4].rate = 23;
        motors[4].counter = 0;
        motors[4].steps = -1;
        motors[4].curr_pos = GRIP_OPEN_MAX;

        motors[3].rate = 50;
        motors[3].counter = 0;
        motors[3].steps = -1;
        motors[3].curr_pos = GRIP_ROTATE_MAX_CW;

        // set starting position
        OCR1A = GRIP_OPEN_MAX;
        OCR1B = GRIP_ROTATE_MAX_CW;
        delay(2000);

        // enable the movement
        motors[4].dir = -1;
        motors[3].dir = +1;

        return 0;
    }
*/

    /* Fall through to signal
     * error in command line syntax
     */
    return -1;
}

/* ----------------------------------------------------------------------------
 * home_turret()
 *
 *  Home the turret rotation.
 *  The function rotates the turret stepper until the turret limit switch is engaged.
 *  The function then resets the turret motor parameters in the scheduling table.
 *
 * param:   nothing
 * return:  nothing
 *
 */
void home_turret(void)
{
    uint8_t limit_switch;

    /* Setup motor for movement towards limit switch
     * with a bogus low limit to allow full turret rotation
     */
    motors[TURRET_STEPPER].rate = STEPPER_HOMING_RATE;
    motors[TURRET_STEPPER].steps = -1;
    motors[TURRET_STEPPER].curr_pos = 0;
    motors[TURRET_STEPPER].limit_low = -4096;
    motors[TURRET_STEPPER].dir = -1;

    /* Run the motor and test limit switch
     * Stop the motor when limit switch is engaged
     */
    limit_switch = PIND & TURRET_LIMIT_SWITCH;

    while ( limit_switch )
    {
        limit_switch = PIND & TURRET_LIMIT_SWITCH;
    }

    motors[TURRET_STEPPER].dir = 0;

    /* Reset motor schedule table to 'home' position
     */
    delay(100); // so that ISR does not clobber changes below
    motors[TURRET_STEPPER].rate = 0;
    motors[TURRET_STEPPER].counter = 0;
    motors[TURRET_STEPPER].dir = 0;
    motors[TURRET_STEPPER].steps = 0;
    motors[TURRET_STEPPER].curr_pos = 0;
    motors[TURRET_STEPPER].limit_high = TURRET_MAX_STEP;
    motors[TURRET_STEPPER].limit_low = STEPPER_LIM_LOW;
}

/* ----------------------------------------------------------------------------
 * home_arm()
 *
 *  Home the arm.
 *  The function homes the arm stepper until the arm limit switch is engaged.
 *  The function then resets the arm motor parameters in the scheduling table.
 *
 * param:   nothing
 * return:  nothing
 *
 */
void home_arm(void)
{
    uint8_t limit_switch;

    /* Setup motor for movement towards limit switch
     * with a bogus low limit to allow full turret rotation
     */
    motors[ARM_STEPPER].rate = STEPPER_HOMING_RATE;
    motors[ARM_STEPPER].steps = -1;
    motors[ARM_STEPPER].curr_pos = 0;
    motors[ARM_STEPPER].limit_low = -4096;
    motors[ARM_STEPPER].dir = -1;

    /* Run the motor and test limit switch
     * Stop the motor when limit switch is engaged
     */
    limit_switch = PINC & ARM_LIMIT_SWITCH;

    while ( limit_switch )
    {
        limit_switch = PINC & ARM_LIMIT_SWITCH;
    }

    motors[ARM_STEPPER].dir = 0;

    /* Reset motor schedule table to 'home' position
     */
    delay(100); // so that ISR does not clobber changes below
    motors[ARM_STEPPER].rate = 0;
    motors[ARM_STEPPER].counter = 0;
    motors[ARM_STEPPER].dir = 0;
    motors[ARM_STEPPER].steps = 0;
    motors[ARM_STEPPER].curr_pos = 0;
    motors[ARM_STEPPER].limit_high = ARM_MAX_STEP;
    motors[ARM_STEPPER].limit_low = STEPPER_LIM_LOW;
}

/* ----------------------------------------------------------------------------
 * home_boom()
 *
 *  Home the boom arm.
 *  The function moves the boom arm stepper until its limit switch is engaged.
 *  The function then resets the boom arm motor parameters in the scheduling table.
 *
 * param:   nothing
 * return:  nothing
 *
 */
void home_boom(void)
{
    uint8_t limit_switch;

    /* Setup motor for movement towards limit switch
     * with a bogus low limit
     */
    motors[BOOM_STEPPER].rate = STEPPER_HOMING_RATE;
    motors[BOOM_STEPPER].steps = -1;
    motors[BOOM_STEPPER].curr_pos = 0;
    motors[BOOM_STEPPER].limit_low = -4096;
    motors[BOOM_STEPPER].dir = -1;

    /* Run the motor and test limit switch
     * Stop the motor when limit switch is engaged
     */
    limit_switch = PINC & BOOM_LIMIT_SWITCH;

    while ( limit_switch )
    {
        limit_switch = PINC & BOOM_LIMIT_SWITCH;
    }

    motors[BOOM_STEPPER].dir = 0;

    /* Reset motor schedule table to 'home' position
     */
    delay(100); // so that ISR does not clobber changes below
    motors[BOOM_STEPPER].rate = 0;
    motors[BOOM_STEPPER].counter = 0;
    motors[BOOM_STEPPER].dir = 0;
    motors[BOOM_STEPPER].steps = 0;
    motors[BOOM_STEPPER].curr_pos = 0;
    motors[BOOM_STEPPER].limit_high = BOOM_MAX_STEP;
    motors[BOOM_STEPPER].limit_low = STEPPER_LIM_LOW;
}

/* ----------------------------------------------------------------------------
 * home_gripper_rorator()
 *
 *  Home the gripper rotator servo.
 *  The function homes the gripper rotator servo.
 *  The function then resets the gripper motor parameters in the scheduling table.
 *
 * param:   nothing
 * return:  nothing
 *
 */
void home_gripper_rorator(void)
{
    OCR1B = SERVO_MID_POINT;

    motors[GRIP_ROTATE_SERVO].rate = 0;
    motors[GRIP_ROTATE_SERVO].counter = 0;
    motors[GRIP_ROTATE_SERVO].dir = 0;
    motors[GRIP_ROTATE_SERVO].steps = 0;
    motors[GRIP_ROTATE_SERVO].curr_pos = SERVO_MID_POINT;
    motors[GRIP_ROTATE_SERVO].limit_high = GRIP_ROTATE_MAX_CCW;
    motors[GRIP_ROTATE_SERVO].limit_low = GRIP_ROTATE_MAX_CW;
}

/* ----------------------------------------------------------------------------
 * home_gripper()
 *
 *  Home the gripper servo.
 *  The function homes the gripper servo.
 *  The function then resets the gripper motor parameters in the scheduling table.
 *
 * param:   nothing
 * return:  nothing
 *
 */
void home_gripper(void)
{
    OCR1A = GRIP_OPEN_MAX;

    motors[GRIP_SERVO].rate = 0;
    motors[GRIP_SERVO].counter = 0;
    motors[GRIP_SERVO].dir = 0;
    motors[GRIP_SERVO].steps = 0;
    motors[GRIP_SERVO].curr_pos = GRIP_OPEN_MAX;
    motors[GRIP_SERVO].limit_high = GRIP_OPEN_MAX;
    motors[GRIP_SERVO].limit_low = GRIP_CLOSE_MAX;
}

/* ----------------------------------------------------------------------------
 * This ISR will trigger when Timer-0 compare reaches the time interval
 * The ISR will be responsible for timing motor movement (3x steppers and 2x servos)
 * ISR will trigger at 20KHz rate, and will activate motors according to
 * a scheduling table.
 *
 * Scheduling table will be read by the ISR, current position updated by the USR,
 * and all other parameters updated by the main program loop
 *
 */
ISR(TIMER0_COMPA_vect)
{
    int     motor_index;
    uint8_t tempPort;

    PORTD ^= HEART_BEAT;

    timerTicks++;

    /* Scan motor list and move each motor according to its set parameters
     * 1. decrement motor counter, if counter == 0 then it is time to move the motor
     * 2. move the motor one step in direction 'dir' until 'steps' is 0 or until limits are reached
     * 3. if 'steps' it -1 move until limits are reached.
     */
    for ( motor_index = 0; motor_index < MOTOR_COUNT; motor_index ++ )
    {
        // When 'dir' is 0 then motor should not move
        if ( motors[motor_index].dir == 0 )
            continue;

        /* When a motor's 'counter' is 0 it is time to move one step
         * in the direction indicated by 'dir' and check limits
         */
        if ( motors[motor_index].counter == 0 )
        {
            // Check motor limits
            if ( motors[motor_index].curr_pos > motors[motor_index].limit_high )
            {
                motors[motor_index].curr_pos = motors[motor_index].limit_high;
                motors[motor_index].dir = 0;
            }
            else if ( motors[motor_index].curr_pos < motors[motor_index].limit_low )
            {
                motors[motor_index].curr_pos = motors[motor_index].limit_low;
                motors[motor_index].dir = 0;
            }

            /* If a 'step' count was specified then only move
             * that number of steps until the value is 0
             */
            if ( motors[motor_index].steps == 0 )
            {
                motors[motor_index].dir = 0;
            }

            // TODO check limit switches and stop motor at limit switch

            /* If 'dir' is 0 then the motor was stopped in one of the conditions above
             * Before continuing to the next motor, clear the hardware interface bits in the stepper drivers.
             * I do this because the steppers are over-driver to get more torque. Turning them off will
             * prevent over heating
             */
            if ( motors[motor_index].dir == 0 )
            {
/*
                switch ( motor_index )
                {
                    case TURRET_STEPPER:
                        tempPort = PORTD;
                        tempPort &= 0x0f;
                        PORTD = tempPort;
                        break;

                    case ARM_STEPPER:
                        tempPort = PORTB;
                        tempPort &= 0x0f;
                        PORTB = tempPort;
                        break;

                    case BOOM_STEPPER:
                        tempPort = PORTC;
                        tempPort &= 0xf0;
                        PORTC = tempPort;
                        break;

                    default:;
                }
*/
                continue;
            }

            /* If the motor is a stepper type then select the appropriate
             * coil sequence
             */
            if ( motor_index == TURRET_STEPPER ||
                 motor_index == ARM_STEPPER    ||
                 motor_index == BOOM_STEPPER      )
            {
                motors[motor_index].stepper_coil += motors[motor_index].dir;
                if ( motors[motor_index].stepper_coil > 3 )
                    motors[motor_index].stepper_coil = 0;
                else if ( motors[motor_index].stepper_coil < 0 )
                    motors[motor_index].stepper_coil = 3;
            }

            /* At this point move the motor one step in the indicated 'dir'
             * Use the appropriate hardware interface for the motor
             */
            switch ( motor_index )
            {
                case TURRET_STEPPER:
                    tempPort = PORTD;
                    tempPort &= 0x0f;
                    tempPort |= motorChannelCW[motors[motor_index].stepper_coil] << 4;
                    PORTD = tempPort;
                    break;

                case ARM_STEPPER:
                    tempPort = PORTB;
                    tempPort &= 0x0f;
                    tempPort |= motorChannelCCW[motors[motor_index].stepper_coil] << 4;
                    PORTB = tempPort;
                    break;

                case BOOM_STEPPER:
                    tempPort = PORTC;
                    tempPort &= 0xf0;
                    tempPort |= motorChannelCW[motors[motor_index].stepper_coil];
                    PORTC = tempPort;
                    break;

                case GRIP_ROTATE_SERVO:
                    OCR1B = motors[motor_index].curr_pos;
                    break;

                case GRIP_SERVO:
                    OCR1A = motors[motor_index].curr_pos;
                    break;

                default:;
            }

            /* Update motor position and decrement step count
             * if 'steps' were specified (i.e. 'steps' != -1)
             */
            motors[motor_index].curr_pos += motors[motor_index].dir;
            if ( motors[motor_index].steps > 0 )
            {
                motors[motor_index].steps--;
            }

            /* Recalculate the 'counter' based on movement 'rate' for the
             * next cycle
             */
            motors[motor_index].counter = SEQ_FREQ / motors[motor_index].rate;
        }
        else
        {
            motors[motor_index].counter--;
        }
    }

    PORTD ^= HEART_BEAT;
}
