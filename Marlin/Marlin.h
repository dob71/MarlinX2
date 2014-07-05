// Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
// Licence: GPL

#ifndef MARLIN_H
#define MARLIN_H

#define  FORCE_INLINE __attribute__((always_inline)) inline

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>


#include "fastio.h"
#include "Configuration.h"
#include "pins.h"

#ifndef AT90USB
#define  HardwareSerial_h // trick to disable the standard HWserial
#endif

#if (ARDUINO >= 100)
# include "Arduino.h"
#else
# include "WProgram.h"
  //Arduino < 1.0.0 does not define this, so we need to do it ourselfs
# define analogInputToDigitalPin(p) ((p) + A0)
#endif

#include "MarlinSerial.h"

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#include "WString.h"

#ifdef AT90USB
  #define MYSERIAL Serial
#else
  #define MYSERIAL MSerial
#endif

#define SERIAL_PROTOCOL(x) MYSERIAL.print(x)
#define SERIAL_PROTOCOL_F(x,y) MYSERIAL.print(x,y)
#define SERIAL_PROTOCOLPGM(x) serialprintPGM(PSTR(x))
#define SERIAL_PROTOCOLLN(x) {MYSERIAL.print(x);MYSERIAL.write('\n');}
#define SERIAL_PROTOCOLLNPGM(x) {serialprintPGM(PSTR(x));MYSERIAL.write('\n');}

extern const char errormagic[];
extern const char echomagic[];

#define SERIAL_ERROR_START serialprintPGM(errormagic)
#define SERIAL_ERROR(x) SERIAL_PROTOCOL(x)
#define SERIAL_ERRORPGM(x) SERIAL_PROTOCOLPGM(x)
#define SERIAL_ERRORLN(x) SERIAL_PROTOCOLLN(x)
#define SERIAL_ERRORLNPGM(x) SERIAL_PROTOCOLLNPGM(x)

#ifdef NO_ECHO_WHILE_PRINTING
#  define SERIAL_ECHO_START ((!machine_printing || (debug_flags & ECHO_WHILE_PRINTING != 0)) ? \
                              ((serialprintPGM(echomagic)),do_print=true) : (do_print=false))
#  define SERIAL_ECHO(x) (do_print ? (SERIAL_PROTOCOL(x)) : (void)0)
#  define SERIAL_ECHOPGM(x) (do_print ? (SERIAL_PROTOCOLPGM(x)) : (void)0)
#  define SERIAL_ECHOPAIR(name,value) (do_print ? (serial_echopair_P(PSTR(name),(value))) : (void)0)
#  define SERIAL_ECHOLN(x) (do_print ? (SERIAL_PROTOCOLLN(x)) : (void)(do_print=true))
#  define SERIAL_ECHOLNPGM(x) (do_print ? (SERIAL_PROTOCOLLNPGM(x)) : (void)(do_print=true))
#else  // NO_ECHO_WHILE_PRINTING
#  define SERIAL_ECHO_START serialprintPGM(echomagic)
#  define SERIAL_ECHO(x) SERIAL_PROTOCOL(x)
#  define SERIAL_ECHOPGM(x) SERIAL_PROTOCOLPGM(x)
#  define SERIAL_ECHOPAIR(name,value) (serial_echopair_P(PSTR(name),(value)))
#  define SERIAL_ECHOLN(x) SERIAL_PROTOCOLLN(x)
#  define SERIAL_ECHOLNPGM(x) SERIAL_PROTOCOLLNPGM(x)
#endif // NO_ECHO_WHILE_PRINTING

// The min number of blocks that if queued machine is assumed to be
// printing. Calculated on the input of the new command.
#define MACHINE_PRINTING_BLOCKS 1
// Flag controlling printing of echo strings  
extern bool machine_printing;
// Flag controlling printing of echo strings  
extern bool do_print;

// Macro for getting current active extruder
#define ACTIVE_EXTRUDER ((int)active_extruder)

// Things to write to serial from program memory. saves 400 to 2k of RAM.
FORCE_INLINE void serialprintPGM(const char *str)
{
  char ch=pgm_read_byte(str);
  while(ch)
  {
    MYSERIAL.write(ch);
    ch=pgm_read_byte(++str);
  }
}

template <typename T> void serial_echopair_P(const char *s_P, T v)
    { serialprintPGM(s_P); SERIAL_ECHO(v); }

void get_command();
void process_commands();
void manage_inactivity();

#if !defined(DUAL_X_DRIVE) && (X_ENABLE_PIN > -1)
  #define enable_x() WRITE(X_ENABLE_PIN, X_ENABLE_ON)
  #define disable_x() WRITE(X_ENABLE_PIN,!X_ENABLE_ON)
#elif defined(DUAL_X_DRIVE) && (X0_ENABLE_PIN > -1) && (X1_ENABLE_PIN > -1)
  #define enable_x0()  WRITE(X0_ENABLE_PIN, X_ENABLE_ON)
  #define disable_x0() WRITE(X0_ENABLE_PIN,!X_ENABLE_ON)
  #define enable_x1()  WRITE(X1_ENABLE_PIN, X_ENABLE_ON)
  #define disable_x1() WRITE(X1_ENABLE_PIN,!X_ENABLE_ON)
  #define enable_x()   { enable_x0(); enable_x1(); }
  #define disable_x()  { disable_x0(); disable_x1(); }
#else
  #define enable_x() /* nothing */
  #define disable_x() /* nothing */
  #define enable_x0() /* nothing */
  #define disable_x0() /* nothing */
  #define enable_x1() /* nothing */
  #define disable_x1() /* nothing */
#endif

#if !defined(DUAL_Y_DRIVE) && (Y_ENABLE_PIN > -1)
  #define enable_y() WRITE(Y_ENABLE_PIN, Y_ENABLE_ON)
  #define disable_y() WRITE(Y_ENABLE_PIN,!Y_ENABLE_ON)
#elif defined(DUAL_Y_DRIVE) && (Y0_ENABLE_PIN > -1) && (Y1_ENABLE_PIN > -1)
  #define enable_y0()  WRITE(Y0_ENABLE_PIN, Y_ENABLE_ON)
  #define disable_y0() WRITE(Y0_ENABLE_PIN,!Y_ENABLE_ON)
  #define enable_y1()  WRITE(Y1_ENABLE_PIN, Y_ENABLE_ON)
  #define disable_y1() WRITE(Y1_ENABLE_PIN,!Y_ENABLE_ON)
  #define enable_y()   { enable_y0(); enable_y1(); }
  #define disable_y()  { disable_y0(); disable_y1(); }
#else
  #define enable_y() /* nothing */ 
  #define disable_y() /* nothing */
  #define enable_y0() /* nothing */
  #define disable_y0() /* nothing */
  #define enable_y1() /* nothing */
  #define disable_y1() /* nothing */
#endif

#if Z_ENABLE_PIN > -1
  #ifdef Z_DUAL_STEPPER_DRIVERS
    #define  enable_z() { WRITE(Z_ENABLE_PIN, Z_ENABLE_ON); WRITE(Z2_ENABLE_PIN, Z_ENABLE_ON); }
    #define disable_z() { WRITE(Z_ENABLE_PIN,!Z_ENABLE_ON); WRITE(Z2_ENABLE_PIN,!Z_ENABLE_ON); }
  #else
    #define  enable_z() WRITE(Z_ENABLE_PIN, Z_ENABLE_ON)
    #define disable_z() WRITE(Z_ENABLE_PIN,!Z_ENABLE_ON)
  #endif
#else
  #define enable_z() /* nothing */
  #define disable_z() /* nothing */
#endif

#if defined(E0_ENABLE_PIN) && (E0_ENABLE_PIN > -1)
  #define enable_e0() WRITE(E0_ENABLE_PIN, E_ENABLE_ON)
  #define disable_e0() WRITE(E0_ENABLE_PIN,!E_ENABLE_ON)
#else
  #define enable_e0()  /* nothing */
  #define disable_e0() /* nothing */
#endif

#if (EXTRUDERS > 1) && defined(E1_ENABLE_PIN) && (E1_ENABLE_PIN > -1)
  #define enable_e1() WRITE(E1_ENABLE_PIN, E_ENABLE_ON)
  #define disable_e1() WRITE(E1_ENABLE_PIN,!E_ENABLE_ON)
#else
  #define enable_e1()  /* nothing */
  #define disable_e1() /* nothing */
#endif

#if (EXTRUDERS > 2) && defined(E2_ENABLE_PIN) && (E2_ENABLE_PIN > -1)
  #define enable_e2() WRITE(E2_ENABLE_PIN, E_ENABLE_ON)
  #define disable_e2() WRITE(E2_ENABLE_PIN,!E_ENABLE_ON)
#else
  #define enable_e2()  /* nothing */
  #define disable_e2() /* nothing */
#endif


enum AxisEnum {X_AXIS=0, Y_AXIS=1, Z_AXIS=2, E_AXIS=3};


void FlushSerialRequestResend();
void ClearToSend();

void get_coordinates();
void prepare_move();
void kill();
void Stop();

bool IsStopped();

void enquecommand(const char *cmd); //put an ascii command at the end of the current buffer.
void enquecommand_P(const char *cmd); //put an ascii command at the end of the current buffer, read from flash
void prepare_arc_move(char isclockwise);
void clamp_to_software_endstops(float target[3]);

#ifdef FAST_PWM_FAN
   void setPwmFrequency(uint8_t pin, int val);
#endif

#ifndef CRITICAL_SECTION_START
  #define CRITICAL_SECTION_START  unsigned char _sreg = SREG; cli();
  #define CRITICAL_SECTION_END    SREG = _sreg;
#endif //CRITICAL_SECTION_START

extern float homing_feedrate[];
extern bool  axis_relative_modes[];
extern int   feedmultiply;
extern float current_position[NUM_AXIS];
extern float extruder_offset[2][EXTRUDERS];
extern unsigned char fanSpeed[EXTRUDERS];

#ifdef ENABLE_ADD_HOMEING
  extern float add_homeing[EXTRUDERS][3];
#endif // ENABLE_ADD_HOMEING

#ifdef DUAL_X_DRIVE
  extern float gXMaxPos[EXTRUDERS];
  extern float gXMinPos[EXTRUDERS];
#endif // DUAL_X_DRIVE
  
#ifdef DUAL_Y_DRIVE
  extern float gYMaxPos[EXTRUDERS];
  extern float gYMinPos[EXTRUDERS];
#endif // DUAL_Y_DRIVE

#ifdef C_COMPENSATION
  extern float gCComp[][EXTRUDERS][2];
  extern int gCComp_size[EXTRUDERS];
  extern int gCComp_max_size;
  extern float gCCom_min_speed[EXTRUDERS];
  extern float gCCom_max_speed[EXTRUDERS];
  #ifdef C_COMPENSATION_AUTO_RETRACT_DST
  extern float gCCom_retr_dst[EXTRUDERS];
  #endif // C_COMPENSATION_AUTO_RETRACT_DST
  #ifdef C_COMPENSATION_OVERCOMPENSATE_RATIO
  extern float gCCom_overcomp[EXTRUDERS];
  #endif // C_COMPENSATION_OVERCOMPENSATE_RATIO
  #ifdef C_COMPENSATION_NO_COMP_TRAVEL_DST
  extern float gCCom_no_comp_dst[EXTRUDERS];
  #endif // C_COMPENSATION_NO_COMP_TRAVEL_DST
  #ifdef C_COMPENSATION_PROP_COMP_TRAVEL_DST
  extern float gCCom_prop_comp_dst[EXTRUDERS];
  #endif // C_COMPENSATION_PROP_COMP_TRAVEL_DST
  #ifdef C_COMPENSATION_CH_LIMIT
  extern float gCCom_ch_limit[EXTRUDERS];
  #endif // C_COMPENSATION_CH_LIMIT
#endif // C_COMPENSATION

#ifdef FWRETRACT
  extern bool  autoretract_enabled;
  extern bool  retracted;
  extern float retract_length, retract_feedrate, retract_zlift;
  extern float retract_recover_length, retract_recover_feedrate;
#endif

#ifdef PER_EXTRUDER_FANS
  extern int fan_pin[EXTRUDERS];
#endif

#if EXTRUDERS > 1
  extern uint8_t follow_me; // Bitmask of the follow me mode state
  extern bool follow_me_heater; // Follw the hotend temperature changes
  #ifdef PER_EXTRUDER_FANS
    extern bool follow_me_fan; // Follw the fan speed changes
  #endif // PER_EXTRUDER_FANS
  #if defined(DUAL_X_DRIVE) || defined(DUAL_Y_DRIVE)
    extern uint8_t follow_mir; // Bitmask of the follow me mirror mode state
  #endif
#endif // EXTRUDERS > 1

extern unsigned long starttime;
extern unsigned long stoptime;

#ifdef ENABLE_DEBUG
  // These are debug flags that when set enable printing out various info
  // during printer operation. Use with M504 S<DBG_SET_FLAGS>
  extern unsigned int debug_flags;
  #define ECHO_WHILE_PRINTING  0x0001
  #define FAN_DEBUG            0x0002
  #define C_COMPENSATION_DEBUG 0x0004
  #define C_COMP_STEPS_DEBUG   0x0008
  #define ACCEL_STEPS_DEBUG    0x0010
  #define PID_DEBUG            0x0020
  // These are debug flags that can be used with M504 P<DBG_PRINT_FLAGS> 
  // for printing out information by the M504 command itself.
  #define DEBUG_PRINT_PLAN     0x0001
#endif // ENABLE_DEBUG

// Handling multiple extruders pins
extern uint8_t active_extruder;

#endif
