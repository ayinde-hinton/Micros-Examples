/****************************************************************************************
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

The state machine has 4 states; forward, right, left, backward
use FSM to make a square: Forward, right turn 90 degrees, forward, right turn 90, ....
or backward, left turn 90 degrees, backward, left turn 90 degrees ......

*******************************************************************************************/
/*   DESIGNER NAME:  Yin Hinton
   ASSIGNMENT NAME:  Lab 4
         FILE NAME:  Lab 4 main.c

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
     -MotorForward(void) - set both motors to forward, use SysTick to create PWM
      with 10 ms period and 25% duty cycle
     -MotorBackward(void) - set both motors to backward, use SysTick to create PWM
      with 10 ms period and 25% duty cycle
     -MotorTurnRight(void) - set left motor to forward and right motor to sleep,
      use SysTick to create PWM with 10 ms period and 25% duty cycle
     -MotorTurnLeft(void) - set right motor to forward and left motor to sleep,
      use SysTick to create PWM with 10 ms period and 25% duty cycle

  Altered Lab 3 Code to adjust pwm timer based on variable input
*/
// SMCLK is initiallized to 12MHz or 1/4 of CPU Clk which initiallizes to 48MHz

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
         P2DIR |= 0xC0; // configure port two (PWM Pins) as all outputs
         // P2.6 and P2.7 are wired to TA0.3 and TA0.4 respectively, so we want to initialize those port directions as outputs
         // When P2OUT is Set to 0, motor doesn't move;  When P2OUT is Set to 1, motor does move;
         // Duty Cycle = (hightime/period)*100

         // Putting motors to sleep
         P3OUT |= ~0xC0;
    return;
}

void TimerInit(void)
{
    // working in the timer a 0 (TA0) control register
    //initialize TimerA0 for PWM
    //Since the motors are connected to P2.6 and P2.7, use TimerA0, compare blocks 3 and 4
    //?????
    // PxSEL in this configuration says we're using port 2's primary module function instead of the general purpose I/O
    P2SEL1 &= 0x3F; //??
    P2SEL0 |= 0xC0; //??
    

    TA0CTL = 0x0240; 
    //Bits 15 - 12 Reserved
    //Bits 11 - 10 Reserved
    //Bits 9 - 8 TASSEL = 0b10; select smclk as source
    //Bits 7 - 6 ID = 0b01; input divider set to 2
    //Bits 5 - 4 TAMC = 0b00; select mode as stop timer
    //Bits 3 - 0 Not yet covered; ignore using zeroes

    TA0CCR0 = 59999;  //number of counts for 0.25 seconds
    TA0CCTL3 |= 0x00E0; //output mode 7 reset/set
    TA0CCTL4 |= 0x00E0; //output mode 7 reset/set
    TA0CCR3 = 14999; //15 000 counts for a 25% duty cycle
    TA0CCR4 = 14999; //15 000 counts for a 25% duty cycle
    TA0CTL |= 0x0010; //set timer to up mode; starts timer
  return;
}

void Delay_Xms(){


    //working example for 0.1s, 100 ms
    TA1CTL &= ~0x0030;                      //stop the timer
    TA1CTL |= 0x0200; TA1CTL &= ~0x0100;    //choose smclk for clock source
    TA1CTL |= 0x0080;                       //choose clock divider of 4; ID = 10
    TA1EX0 |= 0x0004;                       //choose second clock divider of 5 for a total divison of 20
    TA1EX0 &= ~0x0003;                      //sets zeroes in the divider value
    TA1CCR0 = 59999;                        //number of counts for 0.25 seconds
    TA1R   = 0;                             //clear timer
    TA1CTL |= 0x0010;                       //set timer for up mode; statrs the timer
    while (!(TA1CCTL0 & 0x0001)) {}         //wait for flag to know time is up
    TA1CCTL0 &= ~0x0001;                    //clear the flag
    TA1CTL   &= ~0x0030;                    //stop the timer

    TA1CTL = 0x0280;
    TA1EX0  = 0x0004;
    TA0CCR0 = 59999;// 0 to 59 999 = 60 000 counts
    //Period of a 10ms clock based on timer PWM calculations
    //Caputre/Compare 0 compare mode : holds value for comparison to timer TA0R
    //sets period for PWM
}


void Motor_Forward(uint16_t duty1 /*Right*/, uint16_t duty2/*left*/)
{
    //Run TimerA0 in PWM mode(any mode that isn't stop mode) with provided duty cycle
    //duty one and duty two are in units of us


    //where is TAxCCR0 set? in the cctl register? yess?
    //turn on PWM and set duty cycle
    //fixed period of 10ms
    //Set Motor Controls for forward

    P3OUT = P3OUT | (RIGHT_MOT_SLEEP | LEFT_MOT_SLEEP);
    // wake up motors
    P5OUT = P5OUT & ~(RIGHT_MOT_DIR & LEFT_MOT_DIR);
    //motors forward
    P2OUT = P2OUT | (RIGHT_MOT_PWM | LEFT_MOT_PWM);
    //drive pins high for PWM

    TA0CTL |= 0x0010;
    //changes the mode nibble to 01
    //control   bits 5 and 4 are mode control 00 to stop, 01 for up counting
    //          bits 7 and 6 are clock divider, where 01 = /2
    //          bits 9 and 8 choose clock source, where 10 = smclk
    TA0R = 0;
    //counter starts at zero once started
    TA0CCR3 = duty1; //14 999
    //Caputre/Compare 3 compare mode : holds value for comparison to timer TA0R
    //sets high time for right motor(P2.6 -> TA0.3 -> Right Motor)
    TA0CCR4 = duty2; //14 999
    //Caputre/Compare 4 compare mode : holds value for comparison to timer TA0R
    //sets high time for left motor(P2.7 -> TA0.4 -> Left Motor)
    return;
}

void Motor_Backward(uint16_t duty1/*Right*/, uint16_t duty2/*left*/)
{
    //Run TimerA0 in PWM mode(any mode that isn't stop mode) with provided duty cycle
    //duty one and duty two are in units of us


    //where is TAxCCR0 set? in the cctl register? yess?
    //turn on PWM and set duty cycle
    //fixed period of 10ms
    //Set Motor Controls for forward

    P3OUT = P3OUT | (RIGHT_MOT_SLEEP | LEFT_MOT_SLEEP);
    // wake up motors
    P5OUT = P5OUT & (RIGHT_MOT_DIR & LEFT_MOT_DIR);
    //motors forward
    P2OUT = P2OUT | (RIGHT_MOT_PWM | LEFT_MOT_PWM);
    //drive pins high for PWM

    TA0CTL |= 0x0010;
    //changes the mode nibble to 01
    //control   bits 5 and 4 are mode control 00 to stop, 01 for up counting
    //          bits 7 and 6 are clock divider, where 01 = /2
    //          bits 9 and 8 choose clock source, where 10 = smclk
    TA0R = 0;
    //counter starts at zero once started
    TA0CCR3 = duty1; //14 999
    //Caputre/Compare 3 compare mode : holds value for comparison to timer TA0R
    //sets high time for right motor(P2.6 -> TA0.3 -> Right Motor)
    TA0CCR4 = duty2; //14 999
    //Caputre/Compare 4 compare mode : holds value for comparison to timer TA0R
    //sets high time for right motor(P2.7 -> TA0.4 -> Left Motor)
    return;
}

void Turn_Left(uint16_t duty1/*Right*/, uint16_t duty2/*left*/)
{
    //Run TimerA0 in PWM mode(any mode that isn't stop mode) with provided duty cycle
    //duty one and duty two are in units of us


    //where is TAxCCR0 set? in the cctl register? yess?
    //turn on PWM and set duty cycle
    //fixed period of 10ms
    //Set Motor Controls for forward

    P3OUT = P3OUT | (RIGHT_MOT_SLEEP | ~LEFT_MOT_SLEEP);
    // wake up motors
    P5OUT = P5OUT & (RIGHT_MOT_DIR & LEFT_MOT_DIR);
    //motors forward
    P2OUT = P2OUT | (RIGHT_MOT_PWM | LEFT_MOT_PWM);
    //drive pins high for PWM

    TA0CTL |= 0x0010;
    //changes the mode nibble to 01
    //control   bits 5 and 4 are mode control 00 to stop, 01 for up counting
    //          bits 7 and 6 are clock divider, where 01 = /2
    //          bits 9 and 8 choose clock source, where 10 = smclk
    TA0R = 0;
    //counter starts at zero once started
    TA0CCR3 = duty1; //14 999
    //Caputre/Compare 3 compare mode : holds value for comparison to timer TA0R
    //sets high time for right motor(P2.6 -> TA0.3 -> Right Motor)
    TA0CCR4 = duty2; //14 999
    //Caputre/Compare 4 compare mode : holds value for comparison to timer TA0R
    //sets high time for right motor(P2.7 -> TA0.4 -> Left Motor)
    return;
}

void Turn_Right(uint16_t duty1/*Right*/, uint16_t duty2/*left*/)
{
    //Run TimerA0 in PWM mode(any mode that isn't stop mode) with provided duty cycle
    //duty one and duty two are in units of us


    //where is TAxCCR0 set? in the cctl register? yess?
    //turn on PWM and set duty cycle
    //fixed period of 10ms
    //Set Motor Controls for forward

    P3OUT = P3OUT | (~RIGHT_MOT_SLEEP | LEFT_MOT_SLEEP);
    // wake up motors
    P5OUT = P5OUT & (RIGHT_MOT_DIR & LEFT_MOT_DIR);
    //motors forward
    P2OUT = P2OUT | (RIGHT_MOT_PWM | LEFT_MOT_PWM);
    //drive pins high for PWM

    TA0CTL |= 0x0010;
    //changes the mode nibble to 01
    //control   bits 5 and 4 are mode control 00 to stop, 01 for up counting
    //          bits 7 and 6 are clock divider, where 01 = /2
    //          bits 9 and 8 choose clock source, where 10 = smclk
    TA0R = 0;
    //counter starts at zero once started
    TA0CCR3 = duty1; //14 999
    //Caputre/Compare 3 compare mode : holds value for comparison to timer TA0R
    //sets high time for right motor(P2.6 -> TA0.3 -> Right Motor)
    TA0CCR4 = duty2; //14 999
    //Caputre/Compare 4 compare mode : holds value for comparison to timer TA0R
    //sets high time for left motor(P2.7 -> TA0.4 -> Left Motor)
    return;
}

void main(void)
{


       WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer
       Clock_Init48MHz();  // makes bus clock 48 MHz
       MotorInit();
       TimerInit();
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
              Motor_Forward(15000, 14900);
              stateTimer++;
              // exit condition/housekeeping
              if (stateTimer >= 25) {
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
              Turn_Right(0, 14900);
              stateTimer++;
              if (stateTimer >= 9) {
                state = forwards; //change this back to right later
              }
                else {
                state = right;
              }
                  break;
           /*
          case backwards:
              for (i = 100; i > 0; i--) {
              Motor_Backward();
              }
                  break;

          case left:
              for (i = 100; i > 0; i--) {
              Turn_Left();
              }
                  break;
          */
          default:
              state = forwards;
              ;
            return;
          } //switch
           Delay_Xms();
           P3DIR |= 0x01;
           P3OUT ^= 0x01; // Toggles port 3.0
       } //while(1)
   } //main()

