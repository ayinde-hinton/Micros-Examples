/****************************************************************************************
         CPET253 Lab3 - State Machine Review and Motor Drivers

 Jeanne Christman
 original version 6/5/2021

 This program uses a state machine to control the TI-RSLK robot to drive
 in a square

 It uses several function provided by TI as well as the Cortex SysTick
 peripheral
  •inc/Clock.c and inc/Cortex.c must be included
  •the function Clock_Init48MHz() makes the system clock 48MHz (period = 20.83 ns)
  •To use SysTick to create a delay, do the following
      -Set SysTick -> LOAD = the delay you want to create. Remember to multiply
       by 48 for each 1us of delay you want
      -Set SysTick -> VAL = 0 to start the count at 0
      -Set SysTick -> CTRL = 0x00000005 to enable the clock
      -Wait for 0x00010000 to be true to indicate time is up
      -See MSP432 datasheet for more information on SysTick

 To control the motors on the TI-RSLK robot, there are three outputs that need
 to be driven.
    :Pin    :Description            :Notes
    :=======:=======================:=========================
    : P5.5  : Right motor direction : 0=forwards, 1=backwards
    : P3.6  : Right motor sleep     : 0=sleep, 1=awake
    : P2.6  : Right motor PWM       : 0=stop, PWM signal = go
    : P5.4  : Left motor direction  : 0=forwards, 1=backwards
    : P3.7  : Left motor sleep      : 0=sleep, 1= awake
    : P2.7  : Left motor PWM        : 0=stop, PWM signal = go

 Functions in this code:
     -MotorInit(void) - enable the motor pins as outputs, put the motors to sleep
     -MotorForward(void) - set both motors to forward, use SysTick to create PWM
      with 10 ms period and 25% duty cycle
     -MotorBackward(void) - set both motors to backward, use SysTick to create PWM
      with 10 ms period and 25% duty cycle
     -MotorTurnRight(void) - set left motor to forward and right motor to sleep,
      use SysTick to create PWM with 10 ms period and 25% duty cycle
     -MotorTurnLeft(void) - set right motor to forward and left motor to sleep,
      use SysTick to create PWM with 10 ms period and 25% duty cycle
     -MotorStop(void) - nice but not required, used to stop the motors at end of square

The state machine has 4 states; forward, right, left, backward
use FSM to make a square: Forward, right turn 90 degrees, forward, right turn 90, ....
or backward, left turn 90 degrees, backward, left turn 90 degrees ......
*******************************************************************************************/

#include "msp.h"
#include <stdint.h>
#include <stdbool.h>
#include "..\inc\Clock.c"
#include "..\inc\CortexM.c"


//Constants for motor pins
// Are there an rules regarding these constants? what are they?
#define RIGHT_MOT_DIR      0x20     //p5.5
#define RIGHT_MOT_SLEEP    0x40     //p3.6
#define RIGHT_MOT_PWM      0x40     //p2.6
#define LEFT_MOT_DIR       0x08     //p5.4
#define LEFT_MOT_SLEEP     0x80     //p3.7
#define LEFT_MOT_PWM       0x80     //p2.7

void MotorInit (void)
//This function sets the motor pins as outputs and puts the motors to sleep
{
         //set direction pins as outputs
         //set sleep pins as outputs
         //set PWM pins as outputs
         P5DIR |= 0xFF; // configure port five (Direction Pins) as all outputs
         // When P5OUT is Set to 0, motor goes to forwards; When P5OUT is Set to 1, motor goes to backwards
         P3DIR |= 0xFF; // configure port three (Sleep Pins) as all outputs
         // When P3OUT is Set to 0, motor goes to sleep
         P2DIR |= 0xFF; // configure port two (PWM Pins) as all outputs
         // When P2OUT is Set to 0, motor doesn't move;  When P2OUT is Set to 1, motor does move;
         // Duty Cycle = (hightime/period)*100

         // Putting motors to sleep
         P3OUT |= ~0xC0;
    return;
}
void MotorStop (void)
//This function stops the motors by putting 0 on PWM pins and then puts
//motors to sleep
{
    // Do I need to initialize the ports?
    P2OUT &= ~RIGHT_MOT_PWM & ~LEFT_MOT_PWM;       //stop motors
    P3OUT &= ~RIGHT_MOT_SLEEP & ~LEFT_MOT_SLEEP;   //put motors to sleep
    return;
}
void MotorForward (void)
//This function is used to drive both motors in the forward direction.
//It uses SysTick to create a PWM wave with a period of 10ms and 25% duty cycle
//The PWM signal is high for 2.5 ms and low for 7.5 ms
//Each time this function is called, one cycle of the PWM is output on the PWM pin
{
     P3OUT |= RIGHT_MOT_SLEEP | LEFT_MOT_SLEEP;  //wake up motors
     //P3OUT = P3OUT | (RIGHT_MOT_SLEEP | LEFT_MOT_SLEEP);
     //What is this starting off as? Doesn't Matter because anding?
     P5OUT &= ~RIGHT_MOT_DIR & ~LEFT_MOT_DIR;  //motors forward
     //P5OUT = P5OUT & ~(RIGHT_MOT_DIR & LEFT_MOT_DIR);
     P2OUT |= RIGHT_MOT_PWM | LEFT_MOT_PWM;  //drive pins high for PWM
     // wait high time
     // since the clock is 48Mhz, every 48 counts is 1 us
     // high time of 2500 us is 25% duty cycle
         SysTick -> LOAD = 48 * 2500;  //2500us = 2.5ms
         SysTick -> VAL = 0;           //clear the count to 0
         SysTick -> CTRL = 0x00000005; //enable the timer
         while (!(SysTick -> CTRL & 0x00010000)); //wait for flag that time is up
     P2OUT &= ~RIGHT_MOT_PWM & ~LEFT_MOT_PWM;  //drive pins low for PWM
         // now low time
         SysTick -> LOAD = 48 * 7500;  //7.5ms
         SysTick -> VAL = 0;
         SysTick -> CTRL = 0x00000005;
         while (!(SysTick -> CTRL & 0x00010000));
     return;
}
void MotorBackward (void)
//This function is used to drive both motors in the backward direction.
//It uses SysTick to create a PWM wave with a period of 10ms and 25% duty cycle
//The PWM signal is high for 2.5 ms and low for 7.5 ms
//Each time this function is called, one cycle of the PWM is output on the PWM pin
{
     P3OUT |= RIGHT_MOT_SLEEP | LEFT_MOT_SLEEP;  //wake up motors
     //removed the inversion to change direction???
     P5OUT &= RIGHT_MOT_DIR & LEFT_MOT_DIR;  //motors backward
     P2OUT |= RIGHT_MOT_PWM | LEFT_MOT_PWM;  //drive pins high for PWM
     // wait high time
     // since the clock is 48Mhz, every 48 counts is 1 us
     // high time of 2500 us is 25% duty cycle
         SysTick -> LOAD = 48 * 2500;  //2500us = 2.5ms
         SysTick -> VAL = 0;           //clear the count to 0
         SysTick -> CTRL = 0x00000005; //enable the timer
         while (!(SysTick -> CTRL & 0x00010000)); //wait for flag that time is up
     P2OUT &= ~RIGHT_MOT_PWM & ~LEFT_MOT_PWM;  //drive pins low for PWM
         // now low time
         SysTick -> LOAD = 48 * 7500;  //7.5ms
         SysTick -> VAL = 0;
         SysTick -> CTRL = 0x00000005;
         while (!(SysTick -> CTRL & 0x00010000));
     return;
}

void MotorTurnLeft (void)
//This function is used to drive left motor forward and right motor to sleep.
//It uses SysTick to create a PWM wave with a period of 10ms and 25% duty cycle
//The PWM signal is high for 2.5 ms and low for 7.5 ms
//Each time this function is called, one cycle of the PWM is output on the PWM pin
{
     P3OUT |= RIGHT_MOT_SLEEP | ~LEFT_MOT_SLEEP; //wake up motors
     P5OUT &= ~RIGHT_MOT_DIR;  //motors forward
     P2OUT |= RIGHT_MOT_PWM;  //drive pins high for PWM
     // wait high time; wait for PWM to go high??
     // since the clock is 48Mhz, every 48 counts is 1 us
     // high time of 2500 us is 25% duty cycle; T = 10ms???
         SysTick -> LOAD = 48 * 1250;  //1250us = 1.25ms
         SysTick -> VAL = 0;           //clear the count to 0
         SysTick -> CTRL = 0x00000005; //enable the timer
         while (!(SysTick -> CTRL & 0x00010000)); //wait for flag that time is up
     P2OUT &= ~RIGHT_MOT_PWM;  //drive pins low for PWM
         // now low time
         SysTick -> LOAD = 48 * 8750;  //8.75ms
         SysTick -> VAL = 0;
         SysTick -> CTRL = 0x00000005;
         while (!(SysTick -> CTRL & 0x00010000));
     return;
}

void MotorTurnRight (void)
//This function is used to drive right motor forward and left motor to sleep.
//It uses SysTick to create a PWM wave with a period of 10ms and 25% duty cycle
//The PWM signal is high for 2.5 ms and low for 7.5 ms
//Each time this function is called, one cycle of the PWM is output on the PWM pin
{
     P3OUT |= ~RIGHT_MOT_SLEEP | LEFT_MOT_SLEEP;  //wake up motors
     P5OUT &= ~LEFT_MOT_DIR;  //motors forward
     P2OUT |= LEFT_MOT_PWM;  //drive pins high for PWM
     // wait high time
     // since the clock is 48Mhz, every 48 counts is 1 us
     // high time of 2500 us is 25% duty cycle
         SysTick -> LOAD = 48 * 1250;  //1250us = 1.25ms
         SysTick -> VAL = 0;           //clear the count to 0
         SysTick -> CTRL = 0x00000005; //enable the timer
         while (!(SysTick -> CTRL & 0x00010000)); //wait for flag that time is up
     P2OUT &= ~LEFT_MOT_PWM;  //drive pins low for PWM
         // now low time
         SysTick -> LOAD = 48 * 8750;  //8.75ms
         SysTick -> VAL = 0;
         SysTick -> CTRL = 0x00000005;
         while (!(SysTick -> CTRL & 0x00010000));
     return;
}

void main(void)
{


       WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer
       Clock_Init48MHz();  // makes bus clock 48 MHz
       MotorInit();

       //These are the four states of the state machine
       //should there be an idle state also???
       enum motor_states {forwards, backwards, left, right} state, prevState;

       state = forwards;            //start state
       prevState = !forwards;       //used to know when the state has changed
       uint16_t stateTimer = 0;       //used to stay in a state
       bool isNewState;           //true when the state has switched
       // should this be volatile??
       //volatile uint16_t i; //iteration counter for loop delay

       //through this while loop, every time one of the motor functions is called
       //it takes 10ms. Assume that the delay in each state is 10ms
       //time spent in any direction = stateTimer * 10ms
       while(1)
       {
           isNewState = (state != prevState);
           prevState = state;  //save state for next time

          switch (state) {
          //each case below should have entry housekeeping, state business and exit housekeeping
          //remember to reset the stateTimer each time you enter a new state
          //you must assign a new state when stateTer reaches the correct value
          case forwards:
              // entry housekeeping
              //P2OUT &= 0xFF;
              //P3OUT &= 0xFF;
              //P5OUT &= 0xFF;
              if(isNewState){
                  stateTimer = 0;
              }
              MotorForward();
              stateTimer++;
              // exit condition/housekeeping
              if(stateTimer >= 250) {
                  state = right; //change this back to right later
              }
              else {
                  state = forwards;
              }
                  break;

          case right:
              if(isNewState){
                  stateTimer = 0;
              }
              MotorTurnRight();
              stateTimer++;
              if(stateTimer >= 275) {
                  state = forwards;
              }
              else {
                  state = right;
              }
                  break;
          /*
          case backwards:
              for (i = 100; i > 0; i--) {
              MotorBackward();
              }
                  break;

          case left:
              for (i = 100; i > 0; i--) {
              MotorLeft();
              }
                  break;
          */
          default:
              state = forwards;
              ;

          } //switch
       } //while(1)
   } //main()
