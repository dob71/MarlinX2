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

<<<<<<< HEAD
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

#define SERIAL_PROTOCOL(x) MYSERIAL.print(x);
#define SERIAL_PROTOCOL_F(x,y) MYSERIAL.print(x,y);
#define SERIAL_PROTOCOLPGM(x) serialprintPGM(PSTR(x));
#define SERIAL_PROTOCOLLN(x) {MYSERIAL.print(x);MYSERIAL.write('\n');}
#define SERIAL_PROTOCOLLNPGM(x) {serialprintPGM(PSTR(x));MYSERIAL.write('\n');}

const char errormagic[] PROGMEM ="Error:";
const char echomagic[] PROGMEM ="echo:";

#define SERIAL_ERROR_START serialprintPGM(errormagic);
#define SERIAL_ERROR(x) SERIAL_PROTOCOL(x)
#define SERIAL_ERRORPGM(x) SERIAL_PROTOCOLPGM(x)
#define SERIAL_ERRORLN(x) SERIAL_PROTOCOLLN(x)
#define SERIAL_ERRORLNPGM(x) SERIAL_PROTOCOLLNPGM(x)

#define SERIAL_ECHO_START serialprintPGM(echomagic);
#define SERIAL_ECHO(x) SERIAL_PROTOCOL(x)
#define SERIAL_ECHOPGM(x) SERIAL_PROTOCOLPGM(x)
#define SERIAL_ECHOLN(x) SERIAL_PROTOCOLLN(x)
#define SERIAL_ECHOLNPGM(x) SERIAL_PROTOCOLLNPGM(x)

#define SERIAL_ECHOPAIR(name,value) (serial_echopair_P(PSTR(name),(value)))

void serial_echopair_P(const char *s_P, float v);
void serial_echopair_P(const char *s_P, double v);
void serial_echopair_P(const char *s_P, unsigned long v);

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

void get_command();
void process_commands();
void manage_inactivity();

#if !defined(DUAL_X_DRIVE) && (X_ENABLE_PIN > -1)
  #define enable_x() WRITE(X_ENABLE_PIN, X_ENABLE_ON)
  #define disable_x() WRITE(X_ENABLE_PIN,!X_ENABLE_ON)
#elif defined(DUAL_X_DRIVE) && (X0_ENABLE_PIN > -1) && (X1_ENABLE_PIN > -1)
  #define enable_x() ((ACTIVE_EXTRUDER == 0)?(WRITE(X0_ENABLE_PIN, X_ENABLE_ON)):\
                                             (WRITE(X1_ENABLE_PIN, X_ENABLE_ON)))
  #define disable_x() ((ACTIVE_EXTRUDER == 0)?(WRITE(X0_ENABLE_PIN, X_ENABLE_ON)):\
                                              (WRITE(X1_ENABLE_PIN, X_ENABLE_ON)))
  #define enable_x0()  WRITE(X0_ENABLE_PIN, X_ENABLE_ON)
  #define disable_x0() WRITE(X0_ENABLE_PIN,!X_ENABLE_ON)
  #define enable_x1()  WRITE(X1_ENABLE_PIN, X_ENABLE_ON)
  #define disable_x1() WRITE(X1_ENABLE_PIN,!X_ENABLE_ON)
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
  #define enable_y() ((ACTIVE_EXTRUDER==0)?(WRITE(Y0_ENABLE_PIN, Y_ENABLE_ON)):\
                                           (WRITE(Y1_ENABLE_PIN, Y_ENABLE_ON)))
  #define disable_y() ((ACTIVE_EXTRUDER==0)?(WRITE(Y0_ENABLE_PIN, Y_ENABLE_ON)):\
                                            (WRITE(Y1_ENABLE_PIN, Y_ENABLE_ON)))
  #define enable_y0()  WRITE(Y0_ENABLE_PIN, Y_ENABLE_ON)
  #define disable_y0() WRITE(Y0_ENABLE_PIN,!Y_ENABLE_ON)
  #define enable_y1()  WRITE(Y1_ENABLE_PIN, Y_ENABLE_ON)
  #define disable_y1() WRITE(Y1_ENABLE_PIN,!Y_ENABLE_ON)
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
extern int   extrudemultiply; // Sets extrude multiply factor (in percent)
extern float current_position[NUM_AXIS] ;
#ifdef ENABLE_ADD_HOMEING
extern float add_homeing[EXTRUDERS][3];
#endif // ENABLE_ADD_HOMEING
extern float extruder_offset[2][EXTRUDERS];
extern unsigned char fanSpeed[EXTRUDERS];

#ifdef DUAL_X_DRIVE
  extern int gInvertXDir[EXTRUDERS] = {INVERT_X0_DIR, INVERT_X1_DIR};
  extern int gXHomeDir[EXTRUDERS] = {X0_HOME_DIR, X1_HOME_DIR};
  extern float gXMaxPos[EXTRUDERS] = {X0_MAX_POS, X1_MAX_POS};
  extern float gXMinPos[EXTRUDERS] = {X0_MIN_POS, X1_MIN_POS};
  #ifdef MANUAL_HOME_POSITIONS
    extern float gManualXHomePos[EXTRUDERS] = {MANUAL_X0_HOME_POS, MANUAL_X1_HOME_POS};
  #endif // MANUAL_HOME_POSITIONS
  extern float gXHomeRetract[EXTRUDERS] = {X0_HOME_RETRACT_MM, X1_HOME_RETRACT_MM};
#endif // DUAL_X_DRIVE
  
#ifdef DUAL_Y_DRIVE
  extern int gInvertYDir[EXTRUDERS] = {INVERT_Y0_DIR, INVERT_Y1_DIR};
  extern int gYHomeDir[EXTRUDERS] = {Y0_HOME_DIR, Y1_HOME_DIR};
  extern float gYMaxPos[EXTRUDERS] = {Y0_MAX_POS, Y1_MAX_POS};
  extern float gYMinPos[EXTRUDERS] = {Y0_MIN_POS, Y1_MIN_POS};
  #ifdef MANUAL_HOME_POSITIONS
    extern float gManualYHomePos[EXTRUDERS] = {MANUAL_Y0_HOME_POS, MANUAL_Y1_HOME_POS};
  #endif // MANUAL_HOME_POSITIONS
  extern float gYHomeRetract[EXTRUDERS] = {Y0_HOME_RETRACT_MM, Y1_HOME_RETRACT_MM};
#endif // DUAL_Y_DRIVE

#ifdef FWRETRACT
extern bool  autoretract_enabled;
extern bool  retracted;
extern float retract_length, retract_feedrate, retract_zlift;
extern float retract_recover_length, retract_recover_feedrate;
#endif

#ifdef PER_EXTRUDER_FANS
extern int fan_pin[EXTRUDERS];
#endif

extern unsigned long starttime;
extern unsigned long stoptime;

extern unsigned int debug_flags;

// Handling multiple extruders pins
extern uint8_t active_extruder;

#endif
