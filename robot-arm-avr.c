/* robot-arm-avr.c
 *
 * this program is the control program for a robotics arm.
 * the robot is managed by a host work station connected to an ATmega328P via UART
 *
 * send commands to AVR for changing PID constants and getting status
 *
 * +-----+             +-----+
 * |     |             |     |
 * | PC  |             | AVR |
 * |     +--< UART >---+     |
 * |     |             |     |
 * |     |             |     |
 * |     |             |     |
 * +-----+             +--+--+
 *                        |
 *              4-bit channel for Stepper motor
 *
 * ATmega AVR IO
 * ---------------
 * base stepper motor       PD4..PD7    pin 6,11,12,13  out
 * timing                   PB0         pin 14
 *
 * Port B bit assignment
 *
 * b7 b6 b5 b4 b3 b2 b1 b0
 * |  |  |  |  |  |  |  |
 * |  |  |  |  |  |  |  +--- 'o' timing test point
 * |  |  |  |  |  |  +------
 * |  |  |  |  |  +---------
 * |  |  |  |  +------------ \
 * |  |  |  +---------------  | in circuit serial programmer
 * |  |  +------------------ /
 * |  +---------------------
 * +------------------------
 *
 * Port D bit assignment
 *
 * b7 b6 b5 b4 b3 b2 b1 b0
 * |  |  |  |  |  |  |  |
 * |  |  |  |  |  |  |  +--- 'i' UART Rx
 * |  |  |  |  |  |  +------ 'o' UART Tx
 * |  |  |  |  |  +---------
 * |  |  |  |  +------------
 * |  |  |  +--------------- 'o' Stepper motor b1
 * |  |  +------------------ 'o' Stepper motor b2
 * |  +--------------------- 'o' Stepper motor b3
 * +------------------------ 'o' Stepper motor b4
 *
 * note: all references to data sheet are for ATmega 328P Rev. 8161D–AVR–10/09
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

//#include    "i2c_drv.h"
#include    "uart_drv.h"

// debug print to UART port definition
// UART is assumed to be defined and initialized
//#define     __DEBUG_PRINT__

// IO ports B and D initialization
#define     PB_DDR_INIT     0x29    // port data direction
#define     PB_PUP_INIT     0x00    // port input pin pull-up
#define     PB_INIT         0x00    // port initial values

#define     PD_DDR_INIT     0xf2    // port data direction
#define     PD_PUP_INIT     0x00    // port input pin pull-up
#define     PD_INIT         0x00    // port initial values

// misc masks
#define     LOOP_TEST_POINT 0x01    // toggle PID loop timing test point

// Timer1 frequency constants (sec 15.9.2 page 126..126)
#define     SEQ_FREQ        50     // <------ change rate frequency in Hz
#define     TIM1_FREQ_CONST ((PRE_SCALER / SEQ_FREQ) -1)
#define     PRE_SCALER      7812    // 8MHz clock divided by 1024 pre scaler

// UART command line processing
#define     CLI_BUFFER      80
#define     CR              0x0d    // carriage return
#define     LF              0x0a    // line feed
#define     BELL            0x07    // bell
#define     BS              0x08    // back space
#define     SPACE           0x20    // space
#define     MAX_TOKENS      3       // max number of command line tokens
#define     CMD_DELIM       " \t"   // command line white-space delimiters
#define     PROMPT          ">"
#define     PRINT_BUFFER    64      // output print buffer

#define     SYNTAX_ERR      "syntax error.\n"
#define     HELP_TEXT       "\n\
  help                         - help text\n"

/****************************************************************************
  special function prototypes
****************************************************************************/
// This function is called upon a HARDWARE RESET:
void reset(void) __attribute__((naked)) __attribute__((section(".init3")));

/****************************************************************************
  Globals
****************************************************************************/
volatile    uint16_t timerTicks = 0;    // clock timer ticks, increment at PID_FREQ [Hz]
int         nDoPrompt = 1;              // print prompt

/* ----------------------------------------------------------------------------
 * ioinit()
 *
 *  initialize IO interfaces
 *  timer and data rates calculated based on 4MHz internal clock
 *
 */
void ioinit(void)
{
    // reconfigure system clock scaler to 8MHz
    CLKPR = 0x80;   // change clock scaler (sec 8.12.2 p.37)
    CLKPR = 0x00;

    // initialize UART interface to 19200 BAUD, 8 bit, 1 stop, no parity
    uart_initialize();

    // initialize Timer1 to provide a periodic interrupt for PID
    // with Clear Timer on Compare Match (CTC) Mode (sec 15.9.2 p.125)
    // the interrupt routine will drive the PID control loop:
    // - read tilt/gyro
    // - calculate PID values
    // - drive PWM
    TCNT1  = 0;                 // zero initial counter value
    OCR1A  = TIM1_FREQ_CONST;   // OCR1A value for PID ISR frequency
    TCCR1A = 0x00;              // CTC mode with OC1x pins kept in normal IO port mode (not used by timer)
    TCCR1B = 0x0D;              // use OCR1A for compare and internal clock with scaler=1024 (256uSec resolution) and start timer
    TCCR1C = 0;
    TIMSK1 = 0x02;              // interrupt on OCR1A match

    // initialize ADC converter input ADC0
    ADMUX  = 0x60;  // external AVcc reference, left adjusted result, ADC0 source
    ADCSRA = 0xEF;  // enable auto-triggered conversion and interrupts, ADC clock 62.5KHz @ 8MKz system clock
    ADCSRB = 0x00;  // auto trigger source is free-running

    // initialize general IO PB and PD pins for output
    // - PB0, PB1: output, no pull-up, right and left motor fwd/rev control
    // -
    DDRB  = PB_DDR_INIT;            // PB pin directions
    PORTB = PB_INIT | PB_PUP_INIT;  // initial value of pins is '0', and input with pull-up

    DDRD   = PD_DDR_INIT;           // PD data direction
    PORTD  = PD_INIT | PD_PUP_INIT; // initial value of pins is '0', and input with pull-up
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
 * printstr()
 *
 * Send a NULL-terminated string down the UART Tx
 *
 */
void printstr(const char *s)
{

  while (*s)
    {
      if (*s == '\n')
          uart_putchr('\r');

      uart_putchr(*s++);
    }
}

/* ----------------------------------------------------------------------------
 * printstr_p()
 *
 * Same as printstr(), but the string is located in program memory,
 * so "lpm" instructions are needed to fetch it.
 *
 */
void printstr_p(const char *s)
{
  char c;

  for (c = pgm_read_byte(s); c; ++s, c = pgm_read_byte(s))
    {
      if (c == '\n')
          uart_putchr('\r');

      uart_putchr(c);
    }
}

/* ----------------------------------------------------------------------------
 * vprintfunc()
 *
 * print a formatter string
 *
 */
int vprintfunc(char *format, ...)
{
    char    text[PRINT_BUFFER] = {0};             // text buffer for printing messages
    va_list aptr;
    int     ret;

    va_start(aptr, format);
    ret = vsnprintf(text, PRINT_BUFFER, format, aptr);
    va_end(aptr);

    printstr(text);
    
    return(ret);
}

/* ----------------------------------------------------------------------------
 * printfloat()
 *
 * print a floating point number
 *
 */
void printfloat(float val)
{
    int     d1, d2;
    float   f2;
    char    sig;

    d1 = (int) val;
    f2 = val - d1;
    d2 = (int) (f2 * 1000.0);
    sig = (val < 0) ? '-' : '+';
    d1 = abs(d1);
    d2 = abs(d2);
    vprintfunc("%c%d.%03d", sig, d1, d2);
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
    char    *tokens[MAX_TOKENS] = {0, 0, 0};
    char    *token;
    char    *tempCli;
    int     numTokens;

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
    
    // parse and execute command
    if ( strcmp(tokens[0], "help") == 0 )
    {
        printstr_p(PSTR(HELP_TEXT));
        return 0;
    }
    else
        return -1;

    return 0;
}

/* ----------------------------------------------------------------------------
 * this ISR will trigger when Timer-1 compare reaches the time interval
 * the ISR will rotate the 4-bit sequence that drives the stepper motor under test
 *
 */
ISR(TIMER1_COMPA_vect)
{
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
    //static  uint8_t motorChannel[4] = {0x01, 0x02, 0x04, 0x08}; // one-phase on, full step, CW
    //static  uint8_t motorChannel[4] = {0x08, 0x04, 0x02, 0x01}; // one-phase on, full step, CCW

    static  uint8_t motorChannel[4] = {0x03, 0x06, 0x0c, 0x09}; // two-phase on, full step, CW
    //static  uint8_t motorChannel[4] = {0x09, 0x0c, 0x06, 0x03}; // two-phase on, full step, CCW

    /* BIPOLAR */
    //static  uint8_t motorChannel[4] = {0x08, 0x02, 0x04, 0x01}; // one-phase on, full step, CW
    //static  uint8_t motorChannel[4] = {0x01, 0x04, 0x02, 0x08}; // one-phase on, full step, CCW

    //static  uint8_t motorChannel[4] = {0x09, 0x0a, 0x06, 0x05}; // two-phase on, full step, CW
    //static  uint8_t motorChannel[4] = {0x05, 0x06, 0x0a, 0x09}; // two-phase on, full step, CCW

    static  int i = 0;

    timerTicks++;                       // increment timer ticks

    PORTB ^= LOOP_TEST_POINT;           // toggle timing TP to output a cycle-test signal

    PORTD  = (motorChannel[i] << 4);    // write to IO port
    i++;
    if (i > 4) i = 0;

    PORTB ^= LOOP_TEST_POINT;           // toggle cycle-test signal
}

/* ----------------------------------------------------------------------------
 * This ISR will trigger when the ADC completes a conversion
 * conversions are auto-triggered and this ISR will trigger at 31.25KHz
 * ADC result is left adjusted, so only ADCH needs to be read
 *
 */
ISR(ADC_vect)
{
    // ADCH read voltage from ADC register
}

/* ----------------------------------------------------------------------------
 * main() control functions
 *
 * initialize IO: TWI, UART, timer and IO pins for H-bridge control
 * - TWI interface in master mode to read angle info from MPU_6050
 * - timer0 provides 2 PWM signals for motor control
 * - IO pins to control H-bridge direction
 * - UART to send/receive commands from host RaspberryPi
 *
 */
int main(void)
{
    static char commandLine[CLI_BUFFER] = {0};
    static int     nCliIndex;
    static uint8_t inChar;

    // initialize command line buffer
    memset(commandLine, 0, CLI_BUFFER);
    nCliIndex = 0;
    
    // initialize IO devices
    ioinit();

    // clear console and output message
    vprintfunc("%c[2J", 27);
    printstr_p(PSTR("robot arm\n"));
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
                    uart_putchr(inChar);                        // output a CR-LF
                    uart_putchr(LF);
                    if ( process_cli(commandLine) == -1 )       // -- process command --
                        printstr_p(PSTR(SYNTAX_ERR));
                    memset(commandLine, 0, CLI_BUFFER);         // reinitialize command line buffer for next command
                    nCliIndex = 0;
                    if ( nDoPrompt )
                        printstr_p(PSTR(PROMPT));               // output a prompt if required
                    break;
                    
                default:
                    if ( nCliIndex < CLI_BUFFER )
                    {
                        if ( inChar != BS )                     // is character a back-space?
                            commandLine[nCliIndex++] = inChar;  // no, then store in command line buffer
                        else if ( nCliIndex > 0 )               // yes, it is a back-space, but do we have characters to remove?
                        {
                            nCliIndex--;                        // yes, remove the character
                            commandLine[nCliIndex] = 0;
                            uart_putchr(BS);
                            uart_putchr(SPACE);
                        }
                        else
                            inChar = 0;                         // no, so do nothing
                    }
                    else
                        inChar = BELL;
                    
                    uart_putchr(inChar);                        // echo character to console
            }
        } /* UART character input */
    } /* endless while loop */

    return 0;
}
