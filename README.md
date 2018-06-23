# oArm, a 5-DoF robotic arm
This program is the control program for a 5-DoF robot arm modeled after uARM. The robot is managed through by an ATmega328P, controller through RS232.
The ATmega328P AVR performs these tasks:
(1) stepper motor commands to 3 stepper motors (arm, boom, turret)
(2) servo command to gripper (rotate, open/close)
(3) limit switch inputs
(4) {optional} inverse kinematics calculation 
(5) {optional} G-Code interpreter

```
 +-----+             +-----+
 |     |             |     |
 | PC  |             | AVR |
 |     +--< UART >---+     |
 |     |             |     |
 |     |             |     |
 |     |             |     |
 +-----+             +--+--+
                        |
              2x PWM channels to gripper servos
              3x limit micro switches
              12bit IO to 3 stepper motor drivers
              {optional} miscellaneous indicators/switches
```
## ATmega AVR IO
| Function            | AVR       | Pin            | I/O |
|---------------------|-----------|----------------|-----|
| Gripper rotate      | OCR1B     | pin 16         | out |
| Gripper open/close  | OCR1A     | pin 15         | out |
| Turret stepper      | PD4..PD7  | pin 6,11..13   | out |
| Arm stepper         | PB4..PB7  | pin 18,19,9,10 | out |
| Boom stepper        | PC0..PB3  | pin 23..26     | out |
| Turret limit switch | PD2       | pin 4          | in  |
| Arm limit switch    | PC4       | pin 27         | in  |
| Boom limit switch   | PC5       | pin 28         | in  |
| UART                | RXD       | pin 2          | in  |
| UART                | TXD       | pin 3          | out |
| LED                 | PD3       | pin 5          | out |
| RESET               | PC6       | pin 1          | in  |
### Port B bit assignment
```
 b7 b6 b5 b4 b3 b2 b1 b0
 |  |  |  |  |  |  |  |
 |  |  |  |  |  |  |  +--- 'i'
 |  |  |  |  |  |  +------ 'o' OCR1A gripper open/close servo PWM
 |  |  |  |  |  +--------- 'o' OCR1B gripper rotator servo PWM
 |  |  |  |  +------------ 'i'
 |  |  |  +--------------- 'o' \
 |  |  +------------------ 'o'  |
 |  +--------------------- 'o'  | arm stepper driver
 +------------------------ 'o' /
```
### Port C bit assignment
```
    b6 b5 b4 b3 b2 b1 b0
    |  |  |  |  |  |  |
    |  |  |  |  |  |  +--- 'o' \
    |  |  |  |  |  +------ 'o'  |
    |  |  |  |  +--------- 'o'  | boom stepper driver
    |  |  |  +------------ 'o' /
    |  |  +--------------- 'i' arm limit switch
    |  +------------------ 'i' boom limit switch
    +--------------------- 'i' RESET (power-on and push button)
```
### Port D bit assignment
```
 b7 b6 b5 b4 b3 b2 b1 b0
 |  |  |  |  |  |  |  |
 |  |  |  |  |  |  |  +--- 'i' UART Rx
 |  |  |  |  |  |  +------ 'o' UART Tx
 |  |  |  |  |  +--------- 'i' turret limit switch
 |  |  |  |  +------------ 'o' heart beat LED
 |  |  |  +--------------- 'o' \
 |  |  +------------------ 'o'  |
 |  +--------------------- 'o'  | turret stepper driver
 +------------------------ 'o' /
```
Note: all references to data sheet are for ATmega 328P Rev. 8161D–AVR–10/09

Command line list see source or type 'help'<CR>

