/*
  planner.h - buffers movement commands and manages the acceleration profile plan
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

// This module is to be considered a sub-module of stepper.c. Please don't include 
// this file from any other module.

#ifndef planner_h
#define planner_h

#include "Marlin.h"

// This struct is used when buffering the setup for each linear movement "nominal" values are as specified in 
// the source g-code and may never actually be reached if acceleration management is active.
typedef struct {
  // Fields used by the bresenham algorithm for tracing the line
  long steps_x, steps_y, steps_z, steps_e;  // Step count along each axis
  unsigned long step_event_count;           // The number of step events required to complete this block
  long accelerate_until;                    // The index of the step event on which to stop acceleration
  long decelerate_after;                    // The index of the step event on which to start decelerating
  long acceleration_rate;                   // The acceleration rate used for acceleration calculation
  unsigned char direction_bits;             // The direction bit set for this block (refers to *_DIRECTION_BIT in config.h)
  unsigned char active_extruder;            // Selects the active extruder
  union { // TODO: turn all those booleans into flags
    long non_printing;                      // Should cover all 3 below (at least for Arduino)
    struct {
      bool retract;                         // Identified as retract move block (not yet used)
      bool restore;                         // Identified as return move block
      bool travel;                          // Identified as travel move block
    };
  };
  #ifdef C_COMPENSATION
    bool ignore_ccomp;                      // Ignore compensation calculation for this block
    long prev_target_advance;               // Steps ahead from previous block
    long target_advance;                    // Steps ahead during the move (at nominal speed)
    long final_advance;                     // Steps ahead at the end
    unsigned short advance_step_rate;       // How fast to advance in this block (when at nominal speed phase)
  #endif // C_COMPENSATION

  // Fields used by the motion planner to manage acceleration
  float nominal_speed;                               // The nominal speed for this block in mm/sec 
  float entry_speed;                                 // Entry speed at previous-current junction in mm/sec
  float max_entry_speed;                             // Maximum allowable junction entry speed in mm/sec
  float millimeters;                                 // The total travel of this block in mm
  float acceleration;                                // acceleration mm/sec^2
  unsigned char recalculate_flag;                    // Planner flag to recalculate trapezoids on entry junction
  unsigned char nominal_length_flag;                 // Planner flag for nominal speed always reached

  // Settings for the trapezoid generator
  unsigned long nominal_rate;                        // The nominal step rate for this block in step_events/sec 
  unsigned long initial_rate;                        // The jerk-adjusted step rate at start of block  
  unsigned long final_rate;                          // The minimal rate at exit
  unsigned long acceleration_st;                     // acceleration steps/sec^2
  unsigned char fan_speed;                           // fan speed at the block
  volatile char busy;
} block_t;

// Initialize the motion plan subsystem      
void plan_init();

// Add a new linear movement to the buffer. x, y and z is the signed, absolute target position in 
// millimaters. Feed rate specifies the speed of the motion.
void plan_buffer_line(const float &x, const float &y, const float &z, const float &e, float feed_rate, const uint8_t &extruder);

// Set position. Used for G92 instructions.
void plan_set_position(const float &x, const float &y, const float &z, const float &e);
void plan_set_e_position(const float &e);

#ifdef ENABLE_DEBUG
void planner_print_plan();
#endif

void check_axes_activity();
uint8_t movesplanned(); //return the nr of buffered moves

extern unsigned long minsegmenttime;
extern float max_feedrate[3 + EXTRUDERS]; // set the max speeds
extern float axis_steps_per_unit[3 + EXTRUDERS];
extern unsigned long max_acceleration_units_per_sq_second[3 + EXTRUDERS]; // Use M201 to override by software
extern float minimumfeedrate;
extern float acceleration;         // Normal acceleration mm/s^2  THIS IS THE DEFAULT ACCELERATION for all moves. M204 SXXXX
extern float retract_acceleration[EXTRUDERS]; // mm/s^2, per extruder filament pull-pack and push-forward  while standing still in the other axis M204 TXXXX
extern float max_e_jerk[EXTRUDERS]; // mm/s - initial speed for extruder retract moves
extern float max_xy_jerk; //speed than can be stopped at once, if i understand correctly.
extern float max_z_jerk;
extern float mintravelfeedrate;

#ifdef AUTOTEMP
    extern bool autotemp_enabled;
    extern float autotemp_max;
    extern float autotemp_min;
    extern float autotemp_factor;
#endif

    


extern block_t block_buffer[BLOCK_BUFFER_SIZE];            // A ring buffer for motion instfructions
extern volatile unsigned char block_buffer_head;           // Index of the next block to be pushed
extern volatile unsigned char block_buffer_tail; 
// Called when the current block is no longer needed. Discards the block and makes the memory
// availible for new blocks.    
FORCE_INLINE void plan_discard_current_block()  
{
  if (block_buffer_head != block_buffer_tail) {
    block_buffer_tail = (block_buffer_tail + 1) & (BLOCK_BUFFER_SIZE - 1);  
  }
}

// Gets the current block. Returns NULL if buffer empty
FORCE_INLINE block_t *plan_get_current_block() 
{
  if (block_buffer_head == block_buffer_tail) { 
    return(NULL); 
  }
  block_t *block = &block_buffer[block_buffer_tail];
  block->busy = true;
  return(block);
}

// Returns true if there are blocks to process
FORCE_INLINE bool blocks_queued() 
{
  if (block_buffer_head == block_buffer_tail) { 
    return false; 
  }
  return true;
}

// Returns true if there is only one block in the queue
FORCE_INLINE bool is_last_block() 
{
  if (block_buffer_head == (block_buffer_tail+1) & (BLOCK_BUFFER_SIZE-1)) { 
    return true; 
  }
  return false;
}

// Returns number of blocks queued
FORCE_INLINE int num_blocks_queued()
{
  return (block_buffer_head-block_buffer_tail + BLOCK_BUFFER_SIZE) & 
         (BLOCK_BUFFER_SIZE - 1);
}

void allow_cold_extrudes(bool allow);
#endif
