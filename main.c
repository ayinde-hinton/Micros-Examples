/*   DESIGNER NAME:  Yin Hinton
   ASSIGNMENT NAME:  Lab 5
         FILE NAME:  Lab 5 main.c

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
*/
#include <stdbool.h>
#include <stdint.h>
#include "..\inc\Clock.h"
#include "..\inc\CortexM.h"
#include "..\inc\BumpInt.h"
#include "..\inc\Motor.h"
#include "msp.h"

// when I use the include folder I shouldn't need to define wasInterrupt or bool; Right????
extern uint8_t wasInterrupt; // interrupt happened flag
void main(void){
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer
	Motor_Init();
	TimerInit();
    DisableInterrupts();
    Clock_Init48MHz();
    BumpInt_Init();
    EnableInterrupts();
    // maybe move the following three lines to the bottom?
    // or should WaitForInterrupt be at the top/bottom of the state machine?
    //while(1){
    //    WaitForInterrupt();
    //}

       //These are the four states of the state machine
       //should there be an idle state also???
       enum motor_states {idle, reverse, leftHit, rightHit} state, prevState;

       state = idle;            //start state
       prevState = !idle;       //used to know when the state has changed
       uint16_t stateTimer = 0;       //used to stay in a state

       bool isNewState;           //true when the state has switched
       extern bool leftSide;
       extern bool rightSide;
       wasInterrupt = 1;   // tells main about an interrupt flag
       P4IFG       &= 0x00; // clear interrupt flags
       //static uint16_t value_now = 0;
       //uint16_t register_value = 0;
       //register_value = P4IV;
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
          case idle:
              leftSide = false;
              rightSide = false;
              // entry housekeeping
              //P2OUT &= 0xFF;
              //P3OUT &= 0xFF;
              //P5OUT &= 0xFF;
              Motor_Forward(15000, 14900);
              // exit condition/housekeeping
              if (1 == wasInterrupt) {
                state = reverse;
              }
              else {
                state = idle;
              }
              break;

          case reverse:
              if(isNewState){
                  stateTimer = 0;
              }
              Motor_Backward(14950, 15000);
              stateTimer++;
              if (stateTimer >= 20) { // change this back to 20 after debug
                  //if (register_value == 0x02 || 0x06 || 0x08) {
                  if (leftSide == true) {
                      state = leftHit;
                  }
                  //else if (register_value ==  0x0C || 0x0E || 0x10) {
                  else{
                      state = rightHit;
                  }
              }
              else {
               state = reverse;
              }
              break;

          case rightHit:
              if(isNewState){
                  stateTimer = 0;
              }
              Motor_Right(0, 15050);
              stateTimer++;
              if (stateTimer >= 11) {
                wasInterrupt = 0;
                state = idle;
              }
              else {
                state = rightHit;
              }
              break;

          case leftHit:
              if(isNewState){
                  stateTimer = 0;
              }
              Motor_Left(15000, 0);
              stateTimer++;
              if (stateTimer >= 11) {
                wasInterrupt = 0;
                state = idle;
              }
              else {
                state = leftHit;
              }
              break;
          default:
              state = idle;
           // return;
          } //switch
           Delay_Xms();
           P3DIR |= 0x01;
           P3OUT ^= 0x01;

       } //while(1)

   } //main()

