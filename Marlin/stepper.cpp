/*
  stepper.c - stepper motor driver: executes motion plans using stepper motors
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

/* The timer calculations of this module informed by the 'RepRap cartesian firmware' by Zack Smith
   and Philipp Tiefenbacher. */

#include "Marlin.h"
#include "stepper.h"
#include "planner.h"
#include "temperature.h"
#include "ultralcd.h"
#include "language.h"
#include "cardreader.h"
#include "speed_lookuptable.h"
#if DIGIPOTSS_PIN > -1
#include <SPI.h>
#endif


//===========================================================================
//=============================public variables  ============================
//===========================================================================
block_t *current_block;  // A pointer to the block currently being traced


//===========================================================================
//=============================private variables ============================
//===========================================================================
//static makes it inpossible to be called from outside of this file by extern.!

// Variables used by The Stepper Driver Interrupt
static unsigned char out_bits;        // The next stepping-bits to be output
static long counter_x,       // Counter variables for the bresenham line tracer
            counter_y, 
            counter_z,       
            counter_e;
volatile static unsigned long step_events_completed; // The number of step events executed in the current block
#ifdef C_COMPENSATION
static long advance; // tracks how many steps ahead we need to be to compensate for compressed filament
static long initial_advance; // number of steps ahead we need to be when starting the block
static long target_advance; // number of steps ahead we need to be when done accelerating
static long final_advance; // number of steps ahead we need to be when done with the block
static long old_advance; // number of steps ahead we were when updated the steps scheduled last time
static long initial_to_target_advance; // difference from initial to target
static long target_to_final_advance; // difference from target to final
static long e_steps[EXTRUDERS]; // steps scheduled to be done by ISR0
static uint8_t old_extruder; // extruder of the previously executed block
#endif // C_COMPENSATION
static long acceleration_time, deceleration_time;
//static unsigned long accelerate_until, decelerate_after, acceleration_rate, initial_rate, final_rate, nominal_rate;
static unsigned short acc_step_rate; // needed for deccelaration start point
static char step_loops;
static unsigned short OCR1A_nominal;
static unsigned short step_loops_nominal;

volatile long endstops_trigsteps[3]={0,0,0};
volatile long endstops_stepsTotal,endstops_stepsDone;
static volatile bool endstop_x_hit=false;
static volatile bool endstop_y_hit=false;
static volatile bool endstop_z_hit=false;
#ifdef ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED
bool abort_on_endstop_hit = false;
#endif

static bool old_x_min_endstop=false;
static bool old_x_max_endstop=false;
static bool old_y_min_endstop=false;
static bool old_y_max_endstop=false;
static bool old_z_min_endstop=false;
static bool old_z_max_endstop=false;

static bool check_endstops = true;

volatile long count_position[NUM_AXIS] = { 0, 0, 0, 0};
volatile signed char count_direction[NUM_AXIS] = { 1, 1, 1, 1};

//===========================================================================
//=============================functions         ============================
//===========================================================================

#define CHECK_ENDSTOPS  if(check_endstops)

// intRes = intIn1 * intIn2 >> 16
// uses:
// r26 to store 0
// r27 to store the byte 1 of the 24 bit result
#define MultiU16X8toH16(intRes, charIn1, intIn2) \
asm volatile ( \
"clr r26 \n\t" \
"mul %A1, %B2 \n\t" \
"movw %A0, r0 \n\t" \
"mul %A1, %A2 \n\t" \
"add %A0, r1 \n\t" \
"adc %B0, r26 \n\t" \
"lsr r0 \n\t" \
"adc %A0, r26 \n\t" \
"adc %B0, r26 \n\t" \
"clr r1 \n\t" \
: \
"=&r" (intRes) \
: \
"d" (charIn1), \
"d" (intIn2) \
: \
"r26" \
)

// intRes = longIn1 * longIn2 >> 24
// uses:
// r26 to store 0
// r27 to store the byte 1 of the 48bit result
#define MultiU24X24toH16(intRes, longIn1, longIn2) \
asm volatile ( \
"clr r26 \n\t" \
"mul %A1, %B2 \n\t" \
"mov r27, r1 \n\t" \
"mul %B1, %C2 \n\t" \
"movw %A0, r0 \n\t" \
"mul %C1, %C2 \n\t" \
"add %B0, r0 \n\t" \
"mul %C1, %B2 \n\t" \
"add %A0, r0 \n\t" \
"adc %B0, r1 \n\t" \
"mul %A1, %C2 \n\t" \
"add r27, r0 \n\t" \
"adc %A0, r1 \n\t" \
"adc %B0, r26 \n\t" \
"mul %B1, %B2 \n\t" \
"add r27, r0 \n\t" \
"adc %A0, r1 \n\t" \
"adc %B0, r26 \n\t" \
"mul %C1, %A2 \n\t" \
"add r27, r0 \n\t" \
"adc %A0, r1 \n\t" \
"adc %B0, r26 \n\t" \
"mul %B1, %A2 \n\t" \
"add r27, r1 \n\t" \
"adc %A0, r26 \n\t" \
"adc %B0, r26 \n\t" \
"lsr r27 \n\t" \
"adc %A0, r26 \n\t" \
"adc %B0, r26 \n\t" \
"clr r1 \n\t" \
: \
"=&r" (intRes) \
: \
"d" (longIn1), \
"d" (longIn2) \
: \
"r26" , "r27" \
)

// Some useful constants

#define ENABLE_STEPPER_DRIVER_INTERRUPT()  TIMSK1 |= (1<<OCIE1A)
#define DISABLE_STEPPER_DRIVER_INTERRUPT() TIMSK1 &= ~(1<<OCIE1A)


void checkHitEndstops()
{
 if( endstop_x_hit || endstop_y_hit || endstop_z_hit) {
   SERIAL_ECHO_START;
   SERIAL_ECHOPGM(MSG_ENDSTOPS_HIT);
   if(endstop_x_hit) {
     SERIAL_ECHOPAIR(" X:",(float)endstops_trigsteps[X_AXIS]/axis_steps_per_unit[X_AXIS]);
     LCD_MESSAGEPGM(MSG_ENDSTOPS_HIT "X");
   }
   if(endstop_y_hit) {
     SERIAL_ECHOPAIR(" Y:",(float)endstops_trigsteps[Y_AXIS]/axis_steps_per_unit[Y_AXIS]);
     LCD_MESSAGEPGM(MSG_ENDSTOPS_HIT "Y");
   }
   if(endstop_z_hit) {
     SERIAL_ECHOPAIR(" Z:",(float)endstops_trigsteps[Z_AXIS]/axis_steps_per_unit[Z_AXIS]);
     LCD_MESSAGEPGM(MSG_ENDSTOPS_HIT "Z");
   }
   SERIAL_ECHOLN("");
   endstop_x_hit=false;
   endstop_y_hit=false;
   endstop_z_hit=false;
#ifdef ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED
   if (abort_on_endstop_hit)
   {
     card.sdprinting = false;
     card.closefile();
     quickStop();
     setTargetHotend0(0);
     setTargetHotend1(0);
     setTargetHotend2(0);
   }
#endif
 }
}

void endstops_hit_on_purpose()
{
  endstop_x_hit=false;
  endstop_y_hit=false;
  endstop_z_hit=false;
}

void enable_endstops(bool check)
{
  check_endstops = check;
}

//         __________________________
//        /|                        |\     _________________         ^
//       / |                        | \   /|               |\        |
//      /  |                        |  \ / |               | \       s
//     /   |                        |   |  |               |  \      p
//    /    |                        |   |  |               |   \     e
//   +-----+------------------------+---+--+---------------+----+    e
//   |               BLOCK 1            |      BLOCK 2          |    d
//
//                           time ----->
// 
//  The trapezoid is the shape the speed curve over time. It starts at block->initial_rate, accelerates 
//  first block->accelerate_until step_events_completed, then keeps going at constant speed until 
//  step_events_completed reaches block->decelerate_after after which it decelerates until the trapezoid generator is reset.
//  The slope of acceleration is calculated with the leib ramp alghorithm.

void st_wake_up() {
  //  TCNT1 = 0;
  ENABLE_STEPPER_DRIVER_INTERRUPT();  
}

void step_wait(){
    for(int8_t i=0; i < 6; i++){
    }
}
  

FORCE_INLINE unsigned short calc_timer(unsigned short step_rate) {
  unsigned short timer;
  if(step_rate > MAX_STEP_FREQUENCY) step_rate = MAX_STEP_FREQUENCY;
  
  if(step_rate > 20000) { // If steprate > 20kHz >> step 4 times
    step_rate = (step_rate >> 2)&0x3fff;
    step_loops = 4;
  }
  else if(step_rate > 10000) { // If steprate > 10kHz >> step 2 times
    step_rate = (step_rate >> 1)&0x7fff;
    step_loops = 2;
  }
  else {
    step_loops = 1;
  } 
  
  if(step_rate < (F_CPU/500000)) step_rate = (F_CPU/500000);
  step_rate -= (F_CPU/500000); // Correct for minimal speed
  if(step_rate >= (8*256)){ // higher step rate 
    unsigned short table_address = (unsigned short)&speed_lookuptable_fast[(unsigned char)(step_rate>>8)][0];
    unsigned char tmp_step_rate = (step_rate & 0x00ff);
    unsigned short gain = (unsigned short)pgm_read_word_near(table_address+2);
    MultiU16X8toH16(timer, tmp_step_rate, gain);
    timer = (unsigned short)pgm_read_word_near(table_address) - timer;
  }
  else { // lower step rates
    unsigned short table_address = (unsigned short)&speed_lookuptable_slow[0][0];
    table_address += ((step_rate)>>1) & 0xfffc;
    timer = (unsigned short)pgm_read_word_near(table_address);
    timer -= (((unsigned short)pgm_read_word_near(table_address+2) * (unsigned char)(step_rate & 0x0007))>>3);
  }
  if(timer < 100) { timer = 100; MYSERIAL.print(MSG_STEPPER_TO_HIGH); MYSERIAL.println(step_rate); }//(20kHz this should never happen)
  return timer;
}

// Initializes the trapezoid generator from the current block. Called whenever a new 
// block begins.
FORCE_INLINE void trapezoid_generator_reset(uint8_t current_e) {
#ifdef C_COMPENSATION
  advance = initial_advance = current_block->initial_advance;
  target_advance = current_block->target_advance;
  final_advance = current_block->final_advance;
  initial_to_target_advance = target_advance - initial_advance;
  target_to_final_advance = final_advance - target_advance;
  // If extruder changes the old_advance is no longer valid and we assume that 
  // there was no compressed filament in the current extruder or it has run out.
  if(old_extruder != current_e) {
    old_advance = 0;
  }
  // Do E steps + advance steps
  e_steps[current_e] += advance - old_advance;
  old_advance = advance;
  old_extruder = current_e;
#endif // C_COMPENSATION
  deceleration_time = 0;
  // step_rate to timer interval
  OCR1A_nominal = calc_timer(current_block->nominal_rate);
  // make a note of the number of step loops required at nominal speed
  step_loops_nominal = step_loops;
  acc_step_rate = current_block->initial_rate;
  acceleration_time = calc_timer(acc_step_rate);
  OCR1A = acceleration_time;
  
#if defined(C_COMPENSATION) && defined(ENABLE_DEBUG)
  if((debug_flags & C_COMPENSATION_DEBUG) != 0) {
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM("old_advance:");
    SERIAL_ECHO(old_advance);
    SERIAL_ECHOPGM(" initial_advance:");
    SERIAL_ECHO(initial_advance);
    SERIAL_ECHOPGM(" target_advance:");
    SERIAL_ECHO(target_advance);
    SERIAL_ECHOPGM(" final_advance:");
    SERIAL_ECHOLN(final_advance);
  }
#endif // defined(C_COMPENSATION) && defined(C_COMPENSATION_DEBUG)
}

// Called once at each block init time to set the direction of the move
FORCE_INLINE void set_directions(uint8_t current_e)
{
  out_bits = current_block->direction_bits;

  if ((out_bits & (1<<X_AXIS)) != 0) {   // stepping along -X axis
    #if !defined COREXY  //NOT COREXY
      #if !defined(DUAL_X_DRIVE) || EXTRUDERS==1
        WRITE(X_DIR_PIN, INVERT_X_DIR);
      #else
        if(current_e==0 || (follow_me & 1)!=0) { WRITE(X0_DIR_PIN, INVERT_X0_DIR); }
        if(current_e==1 || (follow_me & 2)!=0) { WRITE(X1_DIR_PIN, INVERT_X1_DIR); }
      #endif
    #endif
    count_direction[X_AXIS]=-1;
  }
  else { // +direction
    #if !defined COREXY  //NOT COREXY
      #if !defined(DUAL_X_DRIVE) || EXTRUDERS==1
        WRITE(X_DIR_PIN, INVERT_X_DIR);
      #else
        if(current_e==0 || (follow_me & 1)!=0) { WRITE(X0_DIR_PIN, !INVERT_X0_DIR); }
        if(current_e==1 || (follow_me & 2)!=0) { WRITE(X1_DIR_PIN, !INVERT_X1_DIR); }
      #endif
    #endif
    count_direction[X_AXIS]=1;
  }

  if ((out_bits & (1<<Y_AXIS)) != 0) {   // -direction
    #if !defined COREXY  //NOT COREXY
      #if !defined(DUAL_Y_DRIVE) || EXTRUDERS==1
        WRITE(Y_DIR_PIN, INVERT_Y_DIR);
      #else
        if(current_e==0 || (follow_me & 1)!=0) { WRITE(Y0_DIR_PIN, INVERT_Y0_DIR); }
        if(current_e==1 || (follow_me & 2)!=0) { WRITE(Y1_DIR_PIN, INVERT_Y1_DIR); }
      #endif
    #endif
    count_direction[Y_AXIS]=-1;
  }
  else { // +direction
    #if !defined COREXY  //NOT COREXY
      #if !defined(DUAL_Y_DRIVE) || EXTRUDERS==1
        WRITE(Y_DIR_PIN, !INVERT_Y_DIR);
      #else
        if(current_e==0 || (follow_me & 1)!=0) { WRITE(Y0_DIR_PIN, !INVERT_Y0_DIR); }
        if(current_e==1 || (follow_me & 2)!=0) { WRITE(Y1_DIR_PIN, !INVERT_Y1_DIR); }
      #endif
    #endif
    count_direction[Y_AXIS]=1;
  }
  
  #ifdef COREXY  //coreXY kinematics defined
  if((current_block->steps_x >= current_block->steps_y)&&((out_bits & (1<<X_AXIS)) == 0)){  //+X is major axis
    WRITE(X_DIR_PIN, !INVERT_X_DIR);
    WRITE(Y_DIR_PIN, !INVERT_Y_DIR);
  }
  if((current_block->steps_x >= current_block->steps_y)&&((out_bits & (1<<X_AXIS)) != 0)){  //-X is major axis
    WRITE(X_DIR_PIN, INVERT_X_DIR);
    WRITE(Y_DIR_PIN, INVERT_Y_DIR);
  }      
  if((current_block->steps_y > current_block->steps_x)&&((out_bits & (1<<Y_AXIS)) == 0)){  //+Y is major axis
    WRITE(X_DIR_PIN, !INVERT_X_DIR);
    WRITE(Y_DIR_PIN, INVERT_Y_DIR);
  }        
  if((current_block->steps_y > current_block->steps_x)&&((out_bits & (1<<Y_AXIS)) != 0)){  //-Y is major axis
    WRITE(X_DIR_PIN, INVERT_X_DIR);
    WRITE(Y_DIR_PIN, !INVERT_Y_DIR);
  }  
  #endif //coreXY
  
  if ((out_bits & (1<<Z_AXIS)) != 0) {   // -direction
    WRITE(Z_DIR_PIN,INVERT_Z_DIR);
    #ifdef Z_DUAL_STEPPER_DRIVERS
    WRITE(Z2_DIR_PIN,INVERT_Z_DIR);
    #endif
    count_direction[Z_AXIS]=-1;
  }
  else { // +direction
    WRITE(Z_DIR_PIN,!INVERT_Z_DIR);
    #ifdef Z_DUAL_STEPPER_DRIVERS
    WRITE(Z2_DIR_PIN,!INVERT_Z_DIR);
    #endif
    count_direction[Z_AXIS]=1;
  }

  #ifndef C_COMPENSATION
  if ((out_bits & (1<<E_AXIS)) != 0) {  // -direction
    #if EXTRUDERS > 2
    if(current_e==2 || (follow_me & 4)!=0) { WRITE(E2_DIR_PIN, INVERT_E2_DIR); }
    #endif
    #if EXTRUDERS > 1
    if(current_e==1 || (follow_me & 2)!=0) { WRITE(E1_DIR_PIN, INVERT_E2_DIR); }
    if(current_e==0 || (follow_me & 1)!=0) { WRITE(E0_DIR_PIN, INVERT_E2_DIR); }
    #else  // EXTRUDERS > 1
    WRITE(E0_DIR_PIN, INVERT_E2_DIR);
    #endif // EXTRUDERS > 1
    count_direction[E_AXIS]=-1;
  }
  else { // +direction
    #if EXTRUDERS > 2
    if(current_e==2 || (follow_me & 4)!=0) { WRITE(E2_DIR_PIN, !INVERT_E2_DIR); }
    #endif
    #if EXTRUDERS > 1
    if(current_e==1 || (follow_me & 2)!=0) { WRITE(E1_DIR_PIN, !INVERT_E2_DIR); }
    if(current_e==0 || (follow_me & 1)!=0) { WRITE(E0_DIR_PIN, !INVERT_E2_DIR); }
    #else  // EXTRUDERS > 1
    WRITE(E0_DIR_PIN, !INVERT_E2_DIR);
    #endif // EXTRUDERS > 1
    count_direction[E_AXIS]=1;
  }
  #endif //!C_COMPENSATION

  return;
}

// "The Stepper Driver Interrupt" - This timer interrupt is the workhorse.  
// It pops blocks from the block_buffer and executes them by pulsing the stepper pins appropriately. 
ISR(TIMER1_COMPA_vect)
{
  uint8_t current_e = current_block->active_extruder;
  
  // If there is no current block, attempt to pop one from the buffer
  if (current_block == NULL) {
    // Anything in the buffer?
    current_block = plan_get_current_block();
    if (current_block != NULL) {
      current_block->busy = true;
      trapezoid_generator_reset(current_e);
      counter_x = -(current_block->step_event_count >> 1);
      counter_y = counter_x;
      counter_z = counter_x;
      counter_e = counter_x;
      step_events_completed = 0; 
      
      #ifdef Z_LATE_ENABLE 
      if(current_block->steps_z > 0) {
        enable_z();
        OCR1A = 2000; //1ms wait
        return;
      }
      #endif
      
      // Set movement direction variables and prepare motors 
      set_directions(current_e);
    } 
    else {
      OCR1A=2000; // 1kHz.
      // The queue has emptied, so assume that enough time has passed 
      // for the compressed filament to run out...
      #ifdef C_COMPENSATION
      old_advance = 0;
      #endif
      return;
    }    
  } 

  // Set direction and check limit switches
  if ((out_bits & (1<<X_AXIS)) != 0) {   // stepping along -X axis
    #if X_MIN_PIN > -1
    CHECK_ENDSTOPS
    {
      bool x_min_endstop=(READ(X_MIN_PIN) != X_ENDSTOPS_INVERTING);
      if(x_min_endstop && old_x_min_endstop && (current_block->steps_x > 0)) {
        endstops_trigsteps[X_AXIS] = count_position[X_AXIS];
        endstop_x_hit=true;
        step_events_completed = current_block->step_event_count;
      }
      old_x_min_endstop = x_min_endstop;
    }
    #endif
  }
  else { // +direction
    #if X_MAX_PIN > -1
    CHECK_ENDSTOPS 
    {
      bool x_max_endstop=(READ(X_MAX_PIN) != X_ENDSTOPS_INVERTING);
      if(x_max_endstop && old_x_max_endstop && (current_block->steps_x > 0)){
        endstops_trigsteps[X_AXIS] = count_position[X_AXIS];
        endstop_x_hit=true;
        step_events_completed = current_block->step_event_count;
      }
      old_x_max_endstop = x_max_endstop;
    }
    #endif
  }

  if ((out_bits & (1<<Y_AXIS)) != 0) {   // -direction
    #if Y_MIN_PIN > -1
    CHECK_ENDSTOPS
    {
      bool y_min_endstop=(READ(Y_MIN_PIN) != Y_ENDSTOPS_INVERTING);
      if(y_min_endstop && old_y_min_endstop && (current_block->steps_y > 0)) {
        endstops_trigsteps[Y_AXIS] = count_position[Y_AXIS];
        endstop_y_hit=true;
        step_events_completed = current_block->step_event_count;
      }
      old_y_min_endstop = y_min_endstop;
    }
    #endif
  }
  else { // +direction
    #if Y_MAX_PIN > -1
    CHECK_ENDSTOPS
    {
      bool y_max_endstop=(READ(Y_MAX_PIN) != Y_ENDSTOPS_INVERTING);
      if(y_max_endstop && old_y_max_endstop && (current_block->steps_y > 0)){
        endstops_trigsteps[Y_AXIS] = count_position[Y_AXIS];
        endstop_y_hit=true;
        step_events_completed = current_block->step_event_count;
      }
      old_y_max_endstop = y_max_endstop;
    }
    #endif
  }
  
  
  if ((out_bits & (1<<Z_AXIS)) != 0) {   // -direction
    #if Z_MIN_PIN > -1
    CHECK_ENDSTOPS
    {
      bool z_min_endstop=(READ(Z_MIN_PIN) != Z_ENDSTOPS_INVERTING);
      if(z_min_endstop && old_z_min_endstop && (current_block->steps_z > 0)) {
        endstops_trigsteps[Z_AXIS] = count_position[Z_AXIS];
        endstop_z_hit=true;
        step_events_completed = current_block->step_event_count;
      }
      old_z_min_endstop = z_min_endstop;
    }
    #endif
  }
  else { // +direction
    #if Z_MAX_PIN > -1
    CHECK_ENDSTOPS
    {
      bool z_max_endstop=(READ(Z_MAX_PIN) != Z_ENDSTOPS_INVERTING);
      if(z_max_endstop && old_z_max_endstop && (current_block->steps_z > 0)) {
        endstops_trigsteps[Z_AXIS] = count_position[Z_AXIS];
        endstop_z_hit=true;
        step_events_completed = current_block->step_event_count;
      }
      old_z_max_endstop = z_max_endstop;
    }
    #endif
  }

  for(int8_t i=0; i < step_loops; i++) { // Take multiple steps per interrupt (For high speed moves) 
    #ifndef AT90USB
    MSerial.checkRx(); // Check for serial chars.
    #endif

    #if !defined(COREXY)
      counter_x += current_block->steps_x;
      if (counter_x > 0) {
        #if !defined(DUAL_X_DRIVE) || EXTRUDERS==1
        WRITE(X_STEP_PIN, !INVERT_X_STEP_PIN);
        #else
        if(current_e==0 || (follow_me & 1)!=0) { WRITE(X0_STEP_PIN, !INVERT_X_STEP_PIN); }
        if(current_e==1 || (follow_me & 2)!=0) { WRITE(X1_STEP_PIN, !INVERT_X_STEP_PIN); }
        #endif
        counter_x -= current_block->step_event_count;
        count_position[X_AXIS]+=count_direction[X_AXIS];   
        #if !defined(DUAL_X_DRIVE) || EXTRUDERS==1
        WRITE(X_STEP_PIN, INVERT_X_STEP_PIN);
        #else
        WRITE(X0_STEP_PIN, INVERT_X_STEP_PIN);
        WRITE(X1_STEP_PIN, INVERT_X_STEP_PIN);
        #endif
      }

      counter_y += current_block->steps_y;
      if (counter_y > 0) {
        #if !defined(DUAL_Y_DRIVE) || EXTRUDERS==1
        WRITE(Y_STEP_PIN, !INVERT_Y_STEP_PIN);
        #else
        if(current_e==0 || (follow_me & 1)!=0) { WRITE(Y0_STEP_PIN, !INVERT_Y_STEP_PIN); }
        if(current_e==1 || (follow_me & 2)!=0) { WRITE(Y1_STEP_PIN, !INVERT_Y_STEP_PIN); }
        #endif
        counter_y -= current_block->step_event_count; 
        count_position[Y_AXIS]+=count_direction[Y_AXIS]; 
        #if !defined(DUAL_Y_DRIVE) || EXTRUDERS==1
        WRITE(Y_STEP_PIN, INVERT_Y_STEP_PIN);
        #else
        WRITE(Y0_STEP_PIN, INVERT_Y_STEP_PIN);
        WRITE(Y1_STEP_PIN, INVERT_Y_STEP_PIN);
        #endif
      }
    #endif

    #ifdef COREXY
      counter_x += current_block->steps_x;        
      counter_y += current_block->steps_y;
      
      if ((counter_x > 0)&&!(counter_y>0)){  //X step only
        WRITE(X_STEP_PIN, !INVERT_X_STEP_PIN);
        WRITE(Y_STEP_PIN, !INVERT_Y_STEP_PIN);
        counter_x -= current_block->step_event_count; 
        count_position[X_AXIS]+=count_direction[X_AXIS];         
        WRITE(X_STEP_PIN, INVERT_X_STEP_PIN);
        WRITE(Y_STEP_PIN, INVERT_Y_STEP_PIN);
      }
      
      if (!(counter_x > 0)&&(counter_y>0)){  //Y step only
        WRITE(X_STEP_PIN, !INVERT_X_STEP_PIN);
        WRITE(Y_STEP_PIN, !INVERT_Y_STEP_PIN);
        counter_y -= current_block->step_event_count; 
        count_position[Y_AXIS]+=count_direction[Y_AXIS];
        WRITE(X_STEP_PIN, INVERT_X_STEP_PIN);
        WRITE(Y_STEP_PIN, INVERT_Y_STEP_PIN);
      }        
      
      if ((counter_x > 0)&&(counter_y>0)){  //step in both axes
        if (((out_bits & (1<<X_AXIS)) == 0)^((out_bits & (1<<Y_AXIS)) == 0)){  //X and Y in different directions
          WRITE(Y_STEP_PIN, !INVERT_Y_STEP_PIN);
          counter_x -= current_block->step_event_count;             
          WRITE(Y_STEP_PIN, INVERT_Y_STEP_PIN);
          step_wait();
          count_position[X_AXIS]+=count_direction[X_AXIS];
          count_position[Y_AXIS]+=count_direction[Y_AXIS];
          WRITE(Y_STEP_PIN, !INVERT_Y_STEP_PIN);
          counter_y -= current_block->step_event_count;
          WRITE(Y_STEP_PIN, INVERT_Y_STEP_PIN);
        }
        else{  //X and Y in same direction
          WRITE(X_STEP_PIN, !INVERT_X_STEP_PIN);
          counter_x -= current_block->step_event_count;             
          WRITE(X_STEP_PIN, INVERT_X_STEP_PIN) ;
          step_wait();
          count_position[X_AXIS]+=count_direction[X_AXIS];
          count_position[Y_AXIS]+=count_direction[Y_AXIS];
          WRITE(X_STEP_PIN, !INVERT_X_STEP_PIN); 
          counter_y -= current_block->step_event_count;    
          WRITE(X_STEP_PIN, INVERT_X_STEP_PIN);        
        }
      }
    #endif //corexy
    
    counter_z += current_block->steps_z;
    if (counter_z > 0) {
      WRITE(Z_STEP_PIN, !INVERT_Z_STEP_PIN);
      
      #ifdef Z_DUAL_STEPPER_DRIVERS
        WRITE(Z2_STEP_PIN, !INVERT_Z_STEP_PIN);
      #endif
      
      counter_z -= current_block->step_event_count;
      count_position[Z_AXIS]+=count_direction[Z_AXIS];
      WRITE(Z_STEP_PIN, INVERT_Z_STEP_PIN);
      
      #ifdef Z_DUAL_STEPPER_DRIVERS
        WRITE(Z2_STEP_PIN, INVERT_Z_STEP_PIN);
      #endif
    }

    counter_e += current_block->steps_e;
    if (counter_e > 0) {
      #ifdef C_COMPENSATION
      counter_e -= current_block->step_event_count;
      if ((out_bits & (1<<E_AXIS)) != 0) { // - direction
        #if EXTRUDERS > 1
        if(current_e==2 || (follow_me & 4)!=0) e_steps[2]--;
        if(current_e==1 || (follow_me & 2)!=0) e_steps[1]--;
        if(current_e==0 || (follow_me & 1)!=0) e_steps[0]--;
        #else  // EXTRUDERS > 1
        e_steps[0]--;
        #endif // EXTRUDERS > 1
        count_position[E_AXIS]--;
      }
      else {
        #if EXTRUDERS > 1
        if(current_e==2 || (follow_me & 4)!=0) e_steps[2]++;
        if(current_e==1 || (follow_me & 2)!=0) e_steps[1]++;
        if(current_e==0 || (follow_me & 1)!=0) e_steps[0]++;
        #else  // EXTRUDERS > 1
        e_steps[0]++;
        #endif // EXTRUDERS > 1
        count_position[E_AXIS]++;
      }
      #else // C_COMPENSATION
      #if EXTRUDERS > 2
      if(current_e==2 || (follow_me & 4)!=0) { WRITE(E2_STEP_PIN, !INVERT_E_STEP_PIN); }
      #endif
      #if EXTRUDERS > 1
      if(current_e==1 || (follow_me & 2)!=0) { WRITE(E1_STEP_PIN, !INVERT_E_STEP_PIN); }
      if(current_e==0 || (follow_me & 1)!=0) { WRITE(E0_STEP_PIN, !INVERT_E_STEP_PIN); }
      #else  // EXTRUDERS > 1
      WRITE(E0_STEP_PIN, !INVERT_E_STEP_PIN);
      #endif // EXTRUDERS > 1
      counter_e -= current_block->step_event_count;
      count_position[E_AXIS]+=count_direction[E_AXIS];
      #if EXTRUDERS > 2
      WRITE(E2_STEP_PIN, INVERT_E_STEP_PIN);
      #endif
      #if EXTRUDERS > 1
      WRITE(E1_STEP_PIN, INVERT_E_STEP_PIN);
      #endif
      WRITE(E0_STEP_PIN, INVERT_E_STEP_PIN);
      #endif // C_COMPENSATION
    }
    step_events_completed += 1;  
    if(step_events_completed >= current_block->step_event_count) break;
  }
  
  // Calculare new timer value
  unsigned short timer;
  unsigned short step_rate;
  if (step_events_completed <= (unsigned long int)current_block->accelerate_until) {
    
    MultiU24X24toH16(acc_step_rate, acceleration_time, current_block->acceleration_rate);
    acc_step_rate += current_block->initial_rate;
    
    // upper limit
    if(acc_step_rate > current_block->nominal_rate)
      acc_step_rate = current_block->nominal_rate;

    // step_rate to timer interval
    timer = calc_timer(acc_step_rate);
    OCR1A = timer;
    acceleration_time += timer;
    #ifdef C_COMPENSATION
      advance = initial_advance + (step_events_completed*initial_to_target_advance)/current_block->accelerate_until;
      // Do E steps + advance steps
      e_steps[current_block->active_extruder] += advance - old_advance;
      old_advance = advance;  
    #endif
  } 
  else if (step_events_completed > (unsigned long int)current_block->decelerate_after) {   
    MultiU24X24toH16(step_rate, deceleration_time, current_block->acceleration_rate);
    
    if(step_rate > acc_step_rate) { // Check step_rate stays positive
      step_rate = current_block->final_rate;
    }
    else {
      step_rate = acc_step_rate - step_rate; // Decelerate from aceleration end point.
    }

    // lower limit
    if(step_rate < current_block->final_rate)
      step_rate = current_block->final_rate;

    // step_rate to timer interval
    timer = calc_timer(step_rate);
    OCR1A = timer;
    deceleration_time += timer;
    #ifdef C_COMPENSATION
      long decel_steps_completed = step_events_completed - current_block->decelerate_after;
      long decel_steps_total = current_block->step_event_count - current_block->decelerate_after;
      advance = target_advance + (decel_steps_completed*target_to_final_advance)/decel_steps_total;
      // Do E steps + advance steps
      e_steps[current_block->active_extruder] += advance - old_advance;
      old_advance = advance;  
    #endif //C_COMPENSATION
  }
  else {
    OCR1A = OCR1A_nominal;
    // ensure we're running at the correct step rate, even if we just came off an acceleration
    step_loops = step_loops_nominal;
  }

  // If current block is finished, reset pointer 
  if (step_events_completed >= current_block->step_event_count) {
    current_block = NULL;
    plan_discard_current_block();
  }   
}

#ifdef C_COMPENSATION
// Timer interrupt for E. e_steps is set in the main routine;
// Timer 0 is shared with millies
ISR(TIMER0_COMPA_vect)
{
  // 5kHz interrupt (250000 / 50 = 5000Hz)
  // 10kHz interrupt (250000 / 25 = 10000Hz)
  OCR0A += C_COMPENSATION_E_RATE;

  // Set E direction (Depends on E direction + advance)
  for(unsigned char i=0; i<4; i++) {
    if (e_steps[0] != 0) {
      WRITE(E0_STEP_PIN, INVERT_E_STEP_PIN);
      if (e_steps[0] < 0) {
        WRITE(E0_DIR_PIN, INVERT_E0_DIR);
        e_steps[0]++;
      } 
      else {
        WRITE(E0_DIR_PIN, !INVERT_E0_DIR);
        e_steps[0]--;
      }
      WRITE(E0_STEP_PIN, !INVERT_E_STEP_PIN);
    }
    #if EXTRUDERS > 1
    if (e_steps[1] != 0) {
      WRITE(E1_STEP_PIN, INVERT_E_STEP_PIN);
      if (e_steps[1] < 0) {
        WRITE(E1_DIR_PIN, INVERT_E1_DIR);
        e_steps[1]++;
      } 
      else {
        WRITE(E1_DIR_PIN, !INVERT_E1_DIR);
        e_steps[1]--;
      }
      WRITE(E1_STEP_PIN, !INVERT_E_STEP_PIN);
    }
    #endif
    #if EXTRUDERS > 2
    if (e_steps[2] != 0) {
      WRITE(E2_STEP_PIN, INVERT_E_STEP_PIN);
      if (e_steps[2] < 0) {
        WRITE(E2_DIR_PIN, INVERT_E2_DIR);
        e_steps[2]++;
      } 
      else {
        WRITE(E2_DIR_PIN, !INVERT_E2_DIR);
        e_steps[2]--;
      }
      WRITE(E2_STEP_PIN, !INVERT_E_STEP_PIN);
    }
    #endif
  }
  #ifdef ENABLE_DEBUG
  if((debug_flags & C_COMPENSATION_DEBUG) != 0 && 
     e_steps[0] != 0 
     #if EXTRUDERS > 1
     || e_steps[1] != 0 
     #endif
     #if EXTRUDERS > 2
     || e_steps[2] != 0 
     #endif
    ) {
    extern bool insufficient_e_step_rate;
    insufficient_e_step_rate = true;
  }
  #endif // ENABLE_DEBUG
}
#endif // C_COMPENSATION

void st_init()
{
  digipot_init(); //Initialize Digipot Motor Current
  microstep_init(); //Initialize Microstepping Pins
  
  //Initialize Dir Pins
  #if defined(X_DIR_PIN) && X_DIR_PIN > -1
    SET_OUTPUT(X_DIR_PIN);
  #endif
  #if defined(X0_DIR_PIN) && X0_DIR_PIN > -1
    SET_OUTPUT(X0_DIR_PIN);
  #endif
  #if defined(X1_DIR_PIN) && X1_DIR_PIN > -1
    SET_OUTPUT(X1_DIR_PIN);
  #endif

  #if defined(Y_DIR_PIN) && Y_DIR_PIN > -1 
    SET_OUTPUT(Y_DIR_PIN);
  #endif
  #if defined(Y0_DIR_PIN) && Y0_DIR_PIN > -1 
    SET_OUTPUT(Y0_DIR_PIN);
  #endif
  #if defined(Y1_DIR_PIN) && Y1_DIR_PIN > -1 
    SET_OUTPUT(Y1_DIR_PIN);
  #endif

  #if Z_DIR_PIN > -1 
    SET_OUTPUT(Z_DIR_PIN);
    #if defined(Z_DUAL_STEPPER_DRIVERS) && (Z2_DIR_PIN > -1)
      SET_OUTPUT(Z2_DIR_PIN);
    #endif
  #endif

  #if E0_DIR_PIN > -1 
    SET_OUTPUT(E0_DIR_PIN);
  #endif
  #if defined(E1_DIR_PIN) && (E1_DIR_PIN > -1)
    SET_OUTPUT(E1_DIR_PIN);
  #endif
  #if defined(E2_DIR_PIN) && (E2_DIR_PIN > -1)
    SET_OUTPUT(E2_DIR_PIN);
  #endif

  //Initialize Enable Pins - steppers default to disabled.

  #if defined(X_ENABLE_PIN) && (X_ENABLE_PIN > -1)
    SET_OUTPUT(X_ENABLE_PIN);
    if(!X_ENABLE_ON) WRITE(X_ENABLE_PIN,HIGH);
  #endif
  #if defined(X0_ENABLE_PIN) && (X0_ENABLE_PIN > -1)
    SET_OUTPUT(X0_ENABLE_PIN);
    if(!X_ENABLE_ON) WRITE(X0_ENABLE_PIN,HIGH);
  #endif
  #if defined(X1_ENABLE_PIN) && (X1_ENABLE_PIN > -1)
    SET_OUTPUT(X1_ENABLE_PIN);
    if(!X_ENABLE_ON) WRITE(X1_ENABLE_PIN,HIGH);
  #endif

  #if defined(Y_ENABLE_PIN) && (Y_ENABLE_PIN > -1)
    SET_OUTPUT(Y_ENABLE_PIN);
    if(!Y_ENABLE_ON) WRITE(Y_ENABLE_PIN,HIGH);
  #endif
  #if defined(Y0_ENABLE_PIN) && (Y0_ENABLE_PIN > -1)
    SET_OUTPUT(Y0_ENABLE_PIN);
    if(!Y_ENABLE_ON) WRITE(Y0_ENABLE_PIN,HIGH);
  #endif
  #if defined(Y1_ENABLE_PIN) && (Y1_ENABLE_PIN > -1)
    SET_OUTPUT(Y1_ENABLE_PIN);
    if(!Y_ENABLE_ON) WRITE(Y1_ENABLE_PIN,HIGH);
  #endif

  #if (Z_ENABLE_PIN > -1)
    SET_OUTPUT(Z_ENABLE_PIN);
    if(!Z_ENABLE_ON) WRITE(Z_ENABLE_PIN,HIGH);
    
    #if defined(Z_DUAL_STEPPER_DRIVERS) && (Z2_ENABLE_PIN > -1)
      SET_OUTPUT(Z2_ENABLE_PIN);
      if(!Z_ENABLE_ON) WRITE(Z2_ENABLE_PIN,HIGH);
    #endif
  #endif

  #if (E0_ENABLE_PIN > -1)
    SET_OUTPUT(E0_ENABLE_PIN);
    if(!E_ENABLE_ON) WRITE(E0_ENABLE_PIN,HIGH);
  #endif
  #if defined(E1_ENABLE_PIN) && (E1_ENABLE_PIN > -1)
    SET_OUTPUT(E1_ENABLE_PIN);
    if(!E_ENABLE_ON) WRITE(E1_ENABLE_PIN,HIGH);
  #endif
  #if defined(E2_ENABLE_PIN) && (E2_ENABLE_PIN > -1)
    SET_OUTPUT(E2_ENABLE_PIN);
    if(!E_ENABLE_ON) WRITE(E2_ENABLE_PIN,HIGH);
  #endif

  //endstops and pullups
  
  #if X_MIN_PIN > -1
    SET_INPUT(X_MIN_PIN); 
    #ifdef ENDSTOPPULLUP_XMIN
      WRITE(X_MIN_PIN,HIGH);
    #endif
  #endif
      
  #if Y_MIN_PIN > -1
    SET_INPUT(Y_MIN_PIN); 
    #ifdef ENDSTOPPULLUP_YMIN
      WRITE(Y_MIN_PIN,HIGH);
    #endif
  #endif
  
  #if Z_MIN_PIN > -1
    SET_INPUT(Z_MIN_PIN); 
    #ifdef ENDSTOPPULLUP_ZMIN
      WRITE(Z_MIN_PIN,HIGH);
    #endif
  #endif
      
  #if X_MAX_PIN > -1
    SET_INPUT(X_MAX_PIN); 
    #ifdef ENDSTOPPULLUP_XMAX
      WRITE(X_MAX_PIN,HIGH);
    #endif
  #endif
      
  #if Y_MAX_PIN > -1
    SET_INPUT(Y_MAX_PIN); 
    #ifdef ENDSTOPPULLUP_YMAX
      WRITE(Y_MAX_PIN,HIGH);
    #endif
  #endif
  
  #if Z_MAX_PIN > -1
    SET_INPUT(Z_MAX_PIN); 
    #ifdef ENDSTOPPULLUP_ZMAX
      WRITE(Z_MAX_PIN,HIGH);
    #endif
  #endif

  //Initialize Step Pins
  #if defined(X_STEP_PIN) && (X_STEP_PIN > -1) 
    SET_OUTPUT(X_STEP_PIN);
    WRITE(X_STEP_PIN,INVERT_X_STEP_PIN);
    disable_x();
  #endif
  #if defined(X0_STEP_PIN) && (X0_STEP_PIN > -1) 
    SET_OUTPUT(X0_STEP_PIN);
    WRITE(X0_STEP_PIN,INVERT_X_STEP_PIN);
    disable_x0();
  #endif
  #if defined(X1_STEP_PIN) && (X1_STEP_PIN > -1) 
    SET_OUTPUT(X1_STEP_PIN);
    WRITE(X1_STEP_PIN,INVERT_X_STEP_PIN);
    disable_x1();
  #endif

  #if defined(Y_STEP_PIN) && (Y_STEP_PIN > -1) 
    SET_OUTPUT(Y_STEP_PIN);
    WRITE(Y_STEP_PIN,INVERT_Y_STEP_PIN);
    disable_y();
  #endif
  #if defined(Y0_STEP_PIN) && (Y0_STEP_PIN > -1) 
    SET_OUTPUT(Y0_STEP_PIN);
    WRITE(Y0_STEP_PIN,INVERT_Y_STEP_PIN);
    disable_y0();
  #endif
  #if defined(Y1_STEP_PIN) && (Y1_STEP_PIN > -1) 
    SET_OUTPUT(Y1_STEP_PIN);
    WRITE(Y1_STEP_PIN,INVERT_Y_STEP_PIN);
    disable_y1();
  #endif

  #if (Z_STEP_PIN > -1) 
    SET_OUTPUT(Z_STEP_PIN);
    WRITE(Z_STEP_PIN,INVERT_Z_STEP_PIN);
    #if defined(Z_DUAL_STEPPER_DRIVERS) && (Z2_STEP_PIN > -1)
      SET_OUTPUT(Z2_STEP_PIN);
      WRITE(Z2_STEP_PIN,INVERT_Z_STEP_PIN);
    #endif
    disable_z();
  #endif
  
  #if (E0_STEP_PIN > -1) 
    SET_OUTPUT(E0_STEP_PIN);
    WRITE(E0_STEP_PIN,INVERT_E_STEP_PIN);
    disable_e0();
  #endif  
  #if defined(E1_STEP_PIN) && (E1_STEP_PIN > -1) 
    SET_OUTPUT(E1_STEP_PIN);
    WRITE(E1_STEP_PIN,INVERT_E_STEP_PIN);
    disable_e1();
  #endif  
  #if defined(E2_STEP_PIN) && (E2_STEP_PIN > -1) 
    SET_OUTPUT(E2_STEP_PIN);
    WRITE(E2_STEP_PIN,INVERT_E_STEP_PIN);
    disable_e2();
  #endif  

  #ifdef CONTROLLERFAN_PIN
    SET_OUTPUT(CONTROLLERFAN_PIN); //Set pin used for driver cooling fan
  #endif
  
  // waveform generation = 0100 = CTC
  TCCR1B &= ~(1<<WGM13);
  TCCR1B |=  (1<<WGM12);
  TCCR1A &= ~(1<<WGM11); 
  TCCR1A &= ~(1<<WGM10);

  // output mode = 00 (disconnected)
  TCCR1A &= ~(3<<COM1A0); 
  TCCR1A &= ~(3<<COM1B0); 
  
  // Set the timer pre-scaler
  // Generally we use a divider of 8, resulting in a 2MHz timer
  // frequency on a 16MHz MCU. If you are going to change this, be
  // sure to regenerate speed_lookuptable.h with
  // create_speed_lookuptable.py
  TCCR1B = (TCCR1B & ~(0x07<<CS10)) | (2<<CS10);

  OCR1A = 0x4000;
  TCNT1 = 0;
  ENABLE_STEPPER_DRIVER_INTERRUPT();  

  #ifdef C_COMPENSATION
  #if defined(TCCR0A) && defined(WGM01)
    TCCR0A &= ~(1<<WGM01);
    TCCR0A &= ~(1<<WGM00);
  #endif  
    e_steps[0] = 0;
    e_steps[1] = 0;
    e_steps[2] = 0;
    TIMSK0 |= (1<<OCIE0A);
  #endif //C_COMPENSATION
  
  enable_endstops(true); // Start with endstops active. After homing they can be disabled
  sei();
}


// Block until all buffered steps are executed
void st_synchronize()
{
  while( blocks_queued()
    #ifdef C_COMPENSATION
           || e_steps[0] != 0
    #if EXTRUDERS > 1
           || e_steps[1] != 0
    #endif
    #if EXTRUDERS > 2
           || e_steps[2] != 0
    #endif
    #endif // C_COMPENSATION
       ) 
  {
    manage_heater();
    manage_inactivity();
    lcd_update();
  }
}

void st_set_position(const long &x, const long &y, const long &z, const long &e)
{
  CRITICAL_SECTION_START;
  count_position[X_AXIS] = x;
  count_position[Y_AXIS] = y;
  count_position[Z_AXIS] = z;
  count_position[E_AXIS] = e;
  CRITICAL_SECTION_END;
}

void st_set_e_position(const long &e)
{
  CRITICAL_SECTION_START;
  count_position[E_AXIS] = e;
  CRITICAL_SECTION_END;
}

long st_get_position(uint8_t axis)
{
  long count_pos;
  CRITICAL_SECTION_START;
  count_pos = count_position[axis];
  CRITICAL_SECTION_END;
  return count_pos;
}

void finishAndDisableSteppers()
{
  st_synchronize(); 
  disable_x();
  disable_y(); 
  disable_z(); 
  disable_e0(); 
  disable_e1(); 
  disable_e2(); 
}

void quickStop()
{
  DISABLE_STEPPER_DRIVER_INTERRUPT();
  while(blocks_queued())
    plan_discard_current_block();
  current_block = NULL;
  ENABLE_STEPPER_DRIVER_INTERRUPT();
}

void digipot_init() //Initialize Digipot Motor Current
{
#ifdef ENABLE_DIGITAL_POT_CONTROL
    const uint8_t digipot_motor_current[] = DIGIPOT_MOTOR_CURRENT;
    
    SPI.begin(); 
    pinMode(DIGIPOTSS_PIN, OUTPUT);    
    for(int i=0;i<=4;i++) 
      //digitalPotWrite(digipot_ch[i], digipot_motor_current[i]);
      digipot_current(i,digipot_motor_current[i]);
#endif // ENABLE_DIGITAL_POT_CONTROL
}

#ifdef ENABLE_DIGITAL_POT_CONTROL
void digitalPotWrite(int address, int value) // From Arduino DigitalPotControl example
{
    digitalWrite(DIGIPOTSS_PIN,LOW); // take the SS pin low to select the chip
    SPI.transfer(address); //  send in the address and value via SPI:
    SPI.transfer(value);
    digitalWrite(DIGIPOTSS_PIN,HIGH); // take the SS pin high to de-select the chip:
    //delay(10);
}

void digipot_current(uint8_t driver, int current)
{
    const uint8_t digipot_ch[] = DIGIPOT_CHANNELS;
    digitalPotWrite(digipot_ch[driver], current);
}
#endif // ENABLE_DIGITAL_POT_CONTROL

void microstep_init()
{
  #ifdef ENABLE_MICROSTEPPING_CONTROL
  const uint8_t microstep_modes[] = MICROSTEP_MODES;
  #ifndef DUAL_X_DRIVE
    pinMode(X_MS2_PIN,OUTPUT);
  #else // DUAL_X_DRIVE
    pinMode(X0_MS2_PIN,OUTPUT);
    pinMode(X1_MS2_PIN,OUTPUT);
  #endif // DUAL_X_DRIVE
  #ifndef DUAL_Y_DRIVE
    pinMode(Y_MS2_PIN,OUTPUT);
  #else // DUAL_Y_DRIVE
    pinMode(Y0_MS2_PIN,OUTPUT);
    pinMode(Y1_MS2_PIN,OUTPUT);
  #endif // DUAL_Y_DRIVE
  pinMode(Z_MS2_PIN,OUTPUT);
  pinMode(E0_MS2_PIN,OUTPUT);
  pinMode(E1_MS2_PIN,OUTPUT);
  for(int i=0;i<=4;i++) microstep_mode(i,microstep_modes[i]);
  #endif //ENABLE_MICROSTEPPING_CONTROL
}

#ifdef ENABLE_MICROSTEPPING_CONTROL
void microstep_ms(uint8_t driver, int8_t ms1, int8_t ms2)
{
  if(ms1 > -1) switch(driver)
  {
    case 0: 
    #ifndef DUAL_X_DRIVE
      digitalWrite( X_MS1_PIN,ms1); break;
    #else // DUAL_X_DRIVE
      digitalWrite( X0_MS1_PIN,ms1); break;
      digitalWrite( X1_MS1_PIN,ms1); break;
    #endif // DUAL_X_DRIVE
    case 1: 
    #ifndef DUAL_Y_DRIVE
      digitalWrite( Y_MS1_PIN,ms1); break;
    #else // DUAL_Y_DRIVE
      digitalWrite( Y0_MS1_PIN,ms1); break;
      digitalWrite( Y1_MS1_PIN,ms1); break;
    #endif // DUAL_Y_DRIVE
    case 2: digitalWrite( Z_MS1_PIN,ms1); break;
    case 3: digitalWrite(E0_MS1_PIN,ms1); break;
    case 4: digitalWrite(E1_MS1_PIN,ms1); break;
  }
  if(ms2 > -1) switch(driver)
  {
    case 0: 
    #ifndef DUAL_X_DRIVE
      digitalWrite( X_MS2_PIN,ms2); break;
    #else // DUAL_X_DRIVE
      digitalWrite( X0_MS2_PIN,ms2); break;
      digitalWrite( X1_MS2_PIN,ms2); break;
    #endif // DUAL_X_DRIVE
    case 1: 
    #ifndef DUAL_Y_DRIVE
      digitalWrite( Y_MS2_PIN,ms2); break;
    #else // DUAL_Y_DRIVE
      digitalWrite( Y0_MS2_PIN,ms2); break;
      digitalWrite( Y1_MS2_PIN,ms2); break;
    #endif // DUAL_Y_DRIVE
    case 2: digitalWrite( Z_MS2_PIN,ms2); break;
    case 3: digitalWrite(E0_MS2_PIN,ms2); break;
    case 4: digitalWrite(E1_MS2_PIN,ms2); break;
  }
}

void microstep_mode(uint8_t driver, uint8_t stepping_mode)
{
  switch(stepping_mode)
  {
    case 1: microstep_ms(driver,MICROSTEP1); break;
    case 2: microstep_ms(driver,MICROSTEP2); break;
    case 4: microstep_ms(driver,MICROSTEP4); break;
    case 8: microstep_ms(driver,MICROSTEP8); break;
    case 16: microstep_ms(driver,MICROSTEP16); break;
  }
}

void microstep_readings()
{
      SERIAL_PROTOCOLPGM("MS1,MS2 Pins\n");
      SERIAL_PROTOCOLPGM("X: ");
      #ifndef DUAL_Y_DRIVE
      SERIAL_PROTOCOL(   digitalRead(X_MS1_PIN));
      SERIAL_PROTOCOLLN( digitalRead(X_MS2_PIN));
      #else // DUAL_X_DRIVE
      SERIAL_PROTOCOLPGM("T0: ");
      SERIAL_PROTOCOL(   digitalRead(X0_MS1_PIN));
      SERIAL_PROTOCOLLN( digitalRead(X0_MS2_PIN));
      SERIAL_PROTOCOLPGM("T1: ");
      SERIAL_PROTOCOL(   digitalRead(X1_MS1_PIN));
      SERIAL_PROTOCOLLN( digitalRead(X1_MS2_PIN));
      #endif // DUAL_X_DRIVE
      SERIAL_PROTOCOLPGM("Y: ");
      #ifndef DUAL_Y_DRIVE
      SERIAL_PROTOCOL(   digitalRead(Y_MS1_PIN));
      SERIAL_PROTOCOLLN( digitalRead(Y_MS2_PIN));
      #else // DUAL_Y_DRIVE
      SERIAL_PROTOCOLPGM("T0: ");
      SERIAL_PROTOCOL(   digitalRead(Y0_MS1_PIN));
      SERIAL_PROTOCOLLN( digitalRead(Y0_MS2_PIN));
      SERIAL_PROTOCOLPGM("T1: ");
      SERIAL_PROTOCOL(   digitalRead(Y1_MS1_PIN));
      SERIAL_PROTOCOLLN( digitalRead(Y1_MS2_PIN));
      #endif // DUAL_Y_DRIVE
      SERIAL_PROTOCOLPGM("Z: ");
      SERIAL_PROTOCOL(   digitalRead(Z_MS1_PIN));
      SERIAL_PROTOCOLLN( digitalRead(Z_MS2_PIN));
      SERIAL_PROTOCOLPGM("E0: ");
      SERIAL_PROTOCOL(   digitalRead(E0_MS1_PIN));
      SERIAL_PROTOCOLLN( digitalRead(E0_MS2_PIN));
      SERIAL_PROTOCOLPGM("E1: ");
      SERIAL_PROTOCOL(   digitalRead(E1_MS1_PIN));
      SERIAL_PROTOCOLLN( digitalRead(E1_MS2_PIN));
}
#endif //ENABLE_MICROSTEPPING_CONTROL

