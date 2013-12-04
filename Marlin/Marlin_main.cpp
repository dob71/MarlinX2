/* -*- c++ -*- */

/*
    Reprap firmware based on Sprinter and grbl.
 Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 This firmware is a mashup between Sprinter and grbl.
  (https://github.com/kliment/Sprinter)
  (https://github.com/simen/grbl/tree)
 
 It has preliminary support for Matthew Roberts advance algorithm 
    http://reprap.org/pipermail/reprap-dev/2011-May/003323.html
 */

#include "Marlin.h"

#include "ultralcd.h"
#include "planner.h"
#include "stepper.h"
#include "temperature.h"
#include "motion_control.h"
#include "cardreader.h"
#include "watchdog.h"
#include "ConfigurationStore.h"
#include "language.h"
#include "pins_arduino.h"
#include "stdio.h"

#if DIGIPOTSS_PIN > -1
#include <SPI.h>
#endif

#define VERSION_STRING  "1.1.0 X2 Beta 1"

// look here for descriptions of gcodes: http://linuxcnc.org/handbook/gcode/g-code.html
// http://objects.reprap.org/wiki/Mendel_User_Manual:_RepRapGCodes

//Implemented Codes
//-------------------
// G0  -> G1
// G1  - Coordinated Movement X Y Z E
// G2  - CW ARC
// G3  - CCW ARC
// G4  - Dwell S<seconds> or P<milliseconds>
// G10 - retract filament according to settings of M207
// G11 - retract recover filament according to settings of M208
// G28 - Home all Axis
// G90 - Use Absolute Coordinates
// G91 - Use Relative Coordinates
// G92 - Set current position to cordinates given

//RepRap M Codes
// M0   - Unconditional stop - Wait for user to press a button on the LCD (Only if ULTRA_LCD is enabled)
// M1   - Same as M0
// M104 - Set extruder target temp (use T# to explicitly specify extruder, 
//        H<deg> and L<deg> can be used to increase/decrease the target temperature)
// M105 - Read current temp (use A1 to read for all extruders, T# for specific one)
// M106 - Fan on (use T# for dual drive machines to tun the fan on/off for specific extruder, A1 for all)
// M107 - Fan off (use T# for dual drive machines to tun the fan on/off for specific extruder, A1 for all)
// M109 - Wait for extruder to reach target temp (use A1 to wait for all extruders, W<sec> to change dwell time)
// M114 - Display current position

//Custom M Codes
// M17  - Enable/Power all stepper motors
// M18  - Disable all stepper motors; same as M84
// M20  - List SD card
// M21  - Init SD card
// M22  - Release SD card
// M23  - Select SD file (M23 filename.g)
// M24  - Start/resume SD print
// M25  - Pause SD print
// M26  - Set SD position in bytes (M26 S12345)
// M27  - Report SD print status
// M28  - Start SD write (M28 filename.g)
// M29  - Stop SD write
// M30  - Delete file from SD (M30 filename.g)
// M31  - Output time since last M109 or SD card start to serial
// M42  - Change pin status via gcode Use M42 Px Sy to set pin x to value y, when omitting Px the onboard led will be used.
// M80  - Turn on Power Supply
// M81  - Turn off Power Supply
// M82  - Set E codes absolute (default)
// M83  - Set E codes relative while in Absolute Coordinates (G90) mode
// M84  - Disable steppers until next move, 
//        or use S<seconds> to specify an inactivity timeout, after which the steppers will be disabled.  S0 to disable the timeout.
// M85  - Set inactivity shutdown timer with parameter S<seconds>. To disable set zero (default)
// M92  - Set axis_steps_per_unit - same syntax as G92
// M114 - Output current position to serial port 
// M115 - Capabilities string
// M117 - display message
// M119 - Output Endstop status to serial port
// M140 - Set bed target temp
// M190 - Wait for bed current temp to reach target temp.
// M200 - Set filament diameter
// M201 - Set max acceleration in units/s^2 for print moves (M201 X1000 Y1000)
// M202 - Set max acceleration in units/s^2 for travel moves (M202 X1000 Y1000) Unused in Marlin!!
// M203 - Set maximum feedrate that your machine can sustain (M203 X200 Y200 Z300 E10000) in mm/sec
// M204 - Set default acceleration: S normal moves T filament only moves (M204 S3000 T7000) im mm/sec^2  also sets minimum segment time in ms (B20000) to prevent buffer underruns and M20 minimum feedrate
// M205 - Advanced settings:  minimum travel speed S=while printing V=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk, E=maximum E jerk (for retracts)
// M206 - set additional homeing offset
// M207 - set retract length S[positive mm] F[feedrate mm/sec] Z[additional zlift/hop]
// M208 - set recover=unretract length S[positive mm surplus to the M207 S*] F[feedrate mm/sec]
// M209 - S<1=true/0=false> enable automatic retract detect if the slicer did not support G10/11: every normal extrude-only move will be classified as retract depending on the direction.
// M218 - Set hotend offset (in mm): T<extruder_number> X<offset_on_X> Y<offset_on_Y>
// M220 - S<factor in percent>- set speed factor override percentage
// M221 - S<factor in percent>- set extrude factor override percentage
// M240 - Trigger a camera to take a photograph
// M301 - Set PID parameters P I and D
// M302 - Allow cold extrudes
// M303 - PID relay autotune S<temperature> sets the target temperature. (default target temperature = 150C)
// M304 - Set bed PID parameters P I and D
// M322 - Turn the "follow me" mode on or off for an extruder (paramters: T<extruder>  S<1-on/0-off>), 
//        it's automatically off for the active extruder. If used without S parameter prints current settings.
//        If used without T paramter applies to all extruders.
// M331 - Save current position coordinates (all axes, for active extruder).
//        S<SLOT> - specifies memory slot # (0-based) to save into (default 0)
// M332 - Apply/restore saved coordinates to the active extruder. X<0|1>,Y<0|1>,Z<0|1>,E<0|1> - use 1 to filter the axis in (default), 0 to filter it out. 
//        F<speed> - make move to the restored position, if 'F' is not used the restored coordinates set as current position. 
//        S<SLOT> - specifies memory slot # (0-based) to restore from (default 0)
// M340 - Set filament compression (bowden drive) compensation table paramters. P<0-N> - table entry position, 
//        S<speed> - E speed in mm/sec, C<compensation> - length (in mm) of the filament compressed in the guiding 
//        tube when extruding at the given speed. The table entries should be ordered by E speed value. 
//        Set E speed to 0 for the last entry if need less that max size entries.
// M350 - Set microstepping mode.
// M351 - Toggle MS1 MS2 pins directly.
// M400 - Finish all moves
// M500 - stores paramters in EEPROM
// M501 - reads parameters from EEPROM (if you need reset them after you changed them temporarily).  
// M502 - reverts to the default "factory settings".  You still need to store them in EEPROM afterwards if you want to.
// M503 - print the current settings (from memory not from eeprom)
// M540 - Use S[0|1] to enable or disable the stop SD card print on endstop hit (requires ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)
// M600 - Pause for filament change X[pos] Y[pos] Z[relative lift] E[initial retract] L[later retract distance for removal]
// M907 - Set digital trimpot motor current using axis codes.
// M908 - Control digital trimpot directly.
// M999 - Restart after being stopped by error

// T<NUM> [F<NUM>] [S<NUM>] - change the extruder, the feedrate might be set 
//                            to control the speed of the re-positioning move. 
//                            S# allows to choose what E position the just
//                            selected extruder should start from (# - the 
//                            extruder number to pick the position from). If 
//                            S is not specified the last known position for 
//                            the selected extruder is used.

// Stepper Movement Variables

//===========================================================================
//=============================imported variables============================
//===========================================================================


//===========================================================================
//=============================public variables=============================
//===========================================================================
#ifdef SDSUPPORT
CardReader card;
#endif
float homing_feedrate[] = HOMING_FEEDRATE;
bool axis_relative_modes[] = AXIS_RELATIVE_MODES;
int feedmultiply=100; //100->1 200->2
int saved_feedmultiply;
int extrudemultiply=100; //100->1 200->2
float current_position[NUM_AXIS];
float e_last_position[EXTRUDERS];
#ifdef DUAL_X_DRIVE
  float x_last_position[EXTRUDERS];
#endif
#ifdef DUAL_Y_DRIVE
  float y_last_position[EXTRUDERS];
#endif
#ifdef ENABLE_ADD_HOMEING
float add_homeing[EXTRUDERS][3];
#endif // ENABLE_ADD_HOMEING
uint8_t active_extruder;
uint8_t fanSpeed[EXTRUDERS];
// Extruder offset, only in XY plane
#if EXTRUDERS > 1
float extruder_offset[2][EXTRUDERS] = { 
#if defined(EXTRUDER_OFFSET_X) && defined(EXTRUDER_OFFSET_Y)
  EXTRUDER_OFFSET_X, EXTRUDER_OFFSET_Y 
#endif
}; 
#endif // EXTRUDERS > 1

#ifdef ENABLE_DEBUG
  unsigned int debug_flags;
#endif // ENABLE_DEBUG

#ifdef DUAL_X_DRIVE
  float gXMaxPos[EXTRUDERS] = {X0_MAX_POS, X1_MAX_POS};
  float gXMinPos[EXTRUDERS] = {X0_MIN_POS, X1_MIN_POS};
#endif // DUAL_X_DRIVE
  
#ifdef DUAL_Y_DRIVE
  float gYMaxPos[EXTRUDERS] = {Y0_MAX_POS, Y1_MAX_POS};
  float gYMinPos[EXTRUDERS] = {Y0_MIN_POS, Y1_MIN_POS};
#endif // DUAL_Y_DRIVE

#ifdef C_COMPENSATION
  float gCComp[][EXTRUDERS][2] = { C_COMPENSATION };
  int gCComp_size[EXTRUDERS];
  int gCComp_max_size = sizeof(gCComp) / ((EXTRUDERS * sizeof(float)) << 1);
  float gCCom_min_speed[EXTRUDERS] = C_COMPENSATION_MIN_SPEED;
#endif // C_COMPENSATION

#ifdef FWRETRACT
  bool autoretract_enabled=true;
  bool retracted=false;
  float retract_length=3, retract_feedrate=17*60, retract_zlift=0.8;
  float retract_recover_length=0, retract_recover_feedrate=8*60;
#endif

#if EXTRUDERS > 1
  uint8_t follow_me = 0; // Bitmask of the follow me mode state
  bool follow_me_heater; // Follow the hotend temperature changes
  #ifdef PER_EXTRUDER_FANS
  bool follow_me_fan; // Follw the fan speed changes
  #endif // PER_EXTRUDER_FANS
#endif

const char errormagic[] PROGMEM ="Error:";
const char echomagic[] PROGMEM ="echo:";

//===========================================================================
//=============================private variables=============================
//===========================================================================
const char axis_codes[NUM_AXIS] = {'X', 'Y', 'Z', 'E'};
static float destination[NUM_AXIS] = {  0.0, 0.0, 0.0, 0.0};
static float offset[3] = {0.0, 0.0, 0.0};
static bool home_all_axis = true;
static float feedrate = 1500.0, next_feedrate, saved_feedrate;
static long gcode_N, gcode_LastN, Stopped_gcode_LastN = 0;

static bool relative_mode = false;  //Determines Absolute or Relative Coordinates

static char cmdbuffer[BUFSIZE][MAX_CMD_SIZE];
static bool fromsd[BUFSIZE];
static int bufindr = 0;
static int bufindw = 0;
static int buflen = 0;
static char serial_char;
static int serial_count = 0;
static int recovery_count = 0;
static boolean comment_mode = false;
static char *strchr_pointer; // just a pointer to find chars in the cmd string like X, Y, Z, E, etc

const int sensitive_pins[] = SENSITIVE_PINS; // Sensitive pin list for M42

//Tracks how many times temperature was adjusted up/down
static int temp_adjustment = 0;

//Inactivity shutdown variables
static unsigned long previous_millis_cmd = 0;
static unsigned long max_inactive_time = 0;
static unsigned long stepper_inactive_time = DEFAULT_STEPPER_DEACTIVE_TIME*1000l;

unsigned long starttime=0;
unsigned long stoptime=0;

static uint8_t tmp_extruder;

bool Stopped=false;

bool pos_saved=false;
float saved_position[EXTRUDERS][NUM_AXIS];

//===========================================================================
//=============================ROUTINES=============================
//===========================================================================

void get_arc_coordinates();
bool setTargetedHotend(int code);

extern "C"{
  extern unsigned int __bss_end;
  extern unsigned int __heap_start;
  extern void *__brkval;

  int freeMemory() {
    int free_memory;

    if((int)__brkval == 0)
      free_memory = ((int)&free_memory) - ((int)&__bss_end);
    else
      free_memory = ((int)&free_memory) - ((int)__brkval);

    return free_memory;
  }
}

//adds an command to the main command buffer
//thats really done in a non-safe way.
//needs overworking someday
void enquecommand(const char *cmd)
{
  if(buflen < BUFSIZE)
  {
    //this is dangerous if a mixing of serial and this happsens
    strcpy(&(cmdbuffer[bufindw][0]),cmd);
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM("enqueing \"");
    SERIAL_ECHO(cmdbuffer[bufindw]);
    SERIAL_ECHOLNPGM("\"");
    bufindw= (bufindw + 1)%BUFSIZE;
    buflen += 1;
  }
}

void enquecommand_P(const char *cmd)
{
  if(buflen < BUFSIZE)
  {
    //this is dangerous if a mixing of serial and this happsens
    strcpy_P(&(cmdbuffer[bufindw][0]),cmd);
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM("enqueing \"");
    SERIAL_ECHO(cmdbuffer[bufindw]);
    SERIAL_ECHOLNPGM("\"");
    bufindw= (bufindw + 1)%BUFSIZE;
    buflen += 1;
  }
}

void setup_killpin()
{
  #if( KILL_PIN>-1 )
    pinMode(KILL_PIN,INPUT);
    WRITE(KILL_PIN,HIGH);
  #endif
}
    
void setup_photpin()
{
  #ifdef PHOTOGRAPH_PIN
    #if (PHOTOGRAPH_PIN > -1)
    SET_OUTPUT(PHOTOGRAPH_PIN);
    WRITE(PHOTOGRAPH_PIN, LOW);
    #endif
  #endif 
}

void setup_powerhold()
{
 #ifdef SUICIDE_PIN
   #if (SUICIDE_PIN> -1)
      SET_OUTPUT(SUICIDE_PIN);
      WRITE(SUICIDE_PIN, HIGH);
   #endif
 #endif
 #if (PS_ON_PIN > -1)
   SET_OUTPUT(PS_ON_PIN);
   WRITE(PS_ON_PIN, PS_ON_AWAKE);
 #endif
}

void suicide()
{
 #ifdef SUICIDE_PIN
    #if (SUICIDE_PIN> -1) 
      SET_OUTPUT(SUICIDE_PIN);
      WRITE(SUICIDE_PIN, LOW);
    #endif
  #endif
}

void setup()
{
  setup_killpin(); 
  setup_powerhold();
  MYSERIAL.begin(BAUDRATE);
  SERIAL_PROTOCOLLNPGM("start");
  SERIAL_ECHO_START;

  // Check startup - does nothing if bootloader sets MCUSR to 0
  byte mcu = MCUSR;
  if(mcu & 1) SERIAL_ECHOLNPGM(MSG_POWERUP);
  if(mcu & 2) SERIAL_ECHOLNPGM(MSG_EXTERNAL_RESET);
  if(mcu & 4) SERIAL_ECHOLNPGM(MSG_BROWNOUT_RESET);
  if(mcu & 8) SERIAL_ECHOLNPGM(MSG_WATCHDOG_RESET);
  if(mcu & 32) SERIAL_ECHOLNPGM(MSG_SOFTWARE_RESET);
  MCUSR=0;

  SERIAL_ECHOPGM(MSG_MARLIN);
  SERIAL_ECHOLNPGM(VERSION_STRING);
  #ifdef STRING_VERSION_CONFIG_H
    #ifdef STRING_CONFIG_H_AUTHOR
      SERIAL_ECHO_START;
      SERIAL_ECHOPGM(MSG_CONFIGURATION_VER);
      SERIAL_ECHOPGM(STRING_VERSION_CONFIG_H);
      SERIAL_ECHOPGM(MSG_AUTHOR);
      SERIAL_ECHOLNPGM(STRING_CONFIG_H_AUTHOR);
      SERIAL_ECHOPGM("Compiled: ");
      SERIAL_ECHOLNPGM(__DATE__);
    #endif
  #endif
  SERIAL_ECHO_START;
  SERIAL_ECHOPGM(MSG_FREE_MEMORY);
  SERIAL_ECHO(freeMemory());
  SERIAL_ECHOPGM(MSG_PLANNER_BUFFER_BYTES);
  SERIAL_ECHOLN((int)sizeof(block_t)*BLOCK_BUFFER_SIZE);
  for(int8_t i = 0; i < BUFSIZE; i++)
  {
    fromsd[i] = false;
  }
  // Figure the number of useable entries in the compression compensation table
  #ifdef C_COMPENSATION
  for(uint8_t e = 0; e < EXTRUDERS; e++) {
    int size;
    for(size = 0; 
        size < gCComp_max_size && gCComp[size][e][0] > 0.0;
        size++);
    gCComp_size[e] = size;
  }
  #endif // C_COMPENSATION
  
  Config_RetrieveSettings(); // loads data from EEPROM if available

  tp_init();    // Initialize temperature loop 
  plan_init();  // Initialize planner;
  watchdog_init();
  st_init();    // Initialize stepper, this enables interrupts!
  setup_photpin();
  
  lcd_init();

  { /* Give it 1/2 of a second to accumulate temperature readings */
     unsigned long m = millis();
     while(millis() - m < 500) {
       manage_heater();
       lcd_update();
     }
  }  
}

/* Prints the temperatures state string, the new mode output includes 
   the extruder number(s) and target temperature(s). The active extruder 
   always printed first, the 'e' parametr is used for the old mode only. */
void printHeatersState(bool new_mode, uint8_t e)
{
  if(!new_mode) {
    SERIAL_PROTOCOLPGM("T:");
    SERIAL_PROTOCOL((int)(degHotend(e) + 0.5));
  }
  else {
    for(int8_t i = -1; i < EXTRUDERS; i++) {
      e = (i >= 0) ? i : active_extruder;
      if(i == active_extruder) {
        continue;
      }
      SERIAL_PROTOCOL('T');
      SERIAL_PROTOCOL((int)e);
      SERIAL_PROTOCOL(':');
      SERIAL_PROTOCOL((int)(degHotend(e) + 0.5));
      SERIAL_PROTOCOL('/');
      SERIAL_PROTOCOL((int)(degTargetHotend(e) + 0.5));
      SERIAL_PROTOCOL(' ');
    }
  }
  #if TEMP_BED_PIN > -1
  SERIAL_PROTOCOLPGM(" B:");
  SERIAL_PROTOCOL((int)(degBed() + 0.5)); 
  if(new_mode) {
    SERIAL_PROTOCOL('/');
    SERIAL_PROTOCOL((int)(degTargetBed() + 0.5));
  }
  #endif // TEMP_BED_PIN > -1
  SERIAL_PROTOCOL('\n');
  return;
}

void loop()
{
  if(buflen < (BUFSIZE-1))
    get_command();
  #ifdef SDSUPPORT
  card.checkautostart(false);
  #endif
  if(buflen)
  {
    #ifdef SDSUPPORT
    if(card.saving)
    {
      if(strstr_P(cmdbuffer[bufindr], PSTR("M29")) == NULL)
      {
        card.write_command(cmdbuffer[bufindr]);
        SERIAL_PROTOCOLLNPGM(MSG_OK);
      }
      else
      {
        card.closefile();
        SERIAL_PROTOCOLLNPGM(MSG_FILE_SAVED);
      }
    }
    else
    {
      process_commands();
    }
    #else
    process_commands();
    #endif //SDSUPPORT
    buflen = (buflen-1);
    bufindr = (bufindr + 1)%BUFSIZE;
  }
  //check heater every n milliseconds
  manage_heater();
  manage_inactivity();
  checkHitEndstops();
  lcd_update();
}

void get_command() 
{ 
  while( MYSERIAL.available() > 0  && buflen < BUFSIZE) {
    serial_char = MYSERIAL.read();
    if(serial_char == '\n' || 
       serial_char == '\r' || 
       (serial_char == ':' && !comment_mode) || 
       serial_count >= (MAX_CMD_SIZE - 1) ) 
    {
      comment_mode = false; // clear comment mode for the next line
      if(!serial_count) { //if empty line
        return;
      }
      cmdbuffer[bufindw][serial_count] = 0; //terminate string
      fromsd[bufindw] = false;
      strchr_pointer = strchr(cmdbuffer[bufindw], 'N');
      if(strchr_pointer != NULL)
      {
        gcode_N = (strtol(strchr_pointer + 1, NULL, 10));
        if(gcode_N != (gcode_LastN + 1) && (strstr(cmdbuffer[bufindw], "M110") == NULL)) {
          if(recovery_count <= 0) {
            SERIAL_ERROR_START;
            SERIAL_ERRORPGM(MSG_ERR_LINE_NO);
            SERIAL_ERRORLN(gcode_LastN);
            FlushSerialRequestResend();
          } else {
            --recovery_count;
          }
          serial_count = 0;
          return;
        }

        byte checksum = 0;
        for(strchr_pointer = cmdbuffer[bufindw]; *strchr_pointer != '*'; strchr_pointer++)
        {
          if(!*strchr_pointer)
          {
            SERIAL_ERROR_START;
            SERIAL_ERRORPGM(MSG_ERR_NO_CHECKSUM);
            SERIAL_ERRORLN(gcode_LastN);
            FlushSerialRequestResend();
            serial_count = 0;
            return;
          }
          checksum = checksum^(*strchr_pointer);
        }
        if(strtol(strchr_pointer + 1, NULL, 10) != checksum)
        {
          SERIAL_ERROR_START;
          SERIAL_ERRORPGM(MSG_ERR_CHECKSUM_MISMATCH);
          SERIAL_ERRORLN(gcode_LastN);
          FlushSerialRequestResend();
          serial_count = 0;
          return;
        }
        gcode_LastN = gcode_N;
        // No errors, continue parsing
      }
      else  // if we don't receive 'N' but still see '*'
      {
        if((strchr(cmdbuffer[bufindw], '*') != NULL))
        {
          SERIAL_ERROR_START;
          SERIAL_ERRORPGM(MSG_ERR_NO_LINENUMBER_WITH_CHECKSUM);
          SERIAL_ERRORLN(gcode_LastN);
          serial_count = 0;
          return;
        }
      }
      strchr_pointer = strchr(cmdbuffer[bufindw], 'G');
      if(strchr_pointer != NULL)
      {
        switch(strtol(strchr_pointer + 1, NULL, 10)) {
        case 0:
        case 1:
        case 2:
        case 3:
          if(Stopped == false) { // If printer is stopped by an error the G[0-3] codes are ignored.
            #ifdef SDSUPPORT
            if(card.saving)
              break;
            #endif //SDSUPPORT
            SERIAL_PROTOCOLLNPGM(MSG_OK); 
          }
          else {
            SERIAL_ERRORLNPGM(MSG_ERR_STOPPED);
            LCD_MESSAGEPGM(MSG_STOPPED);
          }
          break;
        default:
          break;
        }
      }
      bufindw = (bufindw + 1)%BUFSIZE;
      buflen += 1;
      serial_count = 0; //clear buffer
    }
    else
    {
      if(serial_char == ';') comment_mode = true;
      if(!comment_mode) cmdbuffer[bufindw][serial_count++] = serial_char;
    }
  }
  #ifdef SDSUPPORT
  if(!card.sdprinting || serial_count!=0){
    return;
  }
  while(!card.eof() && buflen < BUFSIZE) 
  {
    int16_t n=card.get();
    serial_char = (char)n;
    if(serial_char == '\n' || 
       serial_char == '\r' || 
       (serial_char == ':' && comment_mode == false) || 
       serial_count >= (MAX_CMD_SIZE - 1)||n==-1) 
    {
      comment_mode = false; //for new command
      if(card.eof()){
        SERIAL_PROTOCOLLNPGM(MSG_FILE_PRINTED);
        stoptime=millis();
        char time[30];
        unsigned long t=(stoptime-starttime)/1000;
        int hours, minutes;
        minutes=(t/60)%60;
        hours=t/60/60;
        sprintf_P(time, PSTR("%i hours %i minutes"),hours, minutes);
        SERIAL_ECHO_START;
        SERIAL_ECHOLN(time);
        lcd_setstatus(time);
        card.printingHasFinished();
        card.checkautostart(true);
      }
      if(!serial_count)
      {
        return; //if empty line
      }
      cmdbuffer[bufindw][serial_count] = 0; //terminate string
      fromsd[bufindw] = true;
      buflen += 1;
      bufindw = (bufindw + 1)%BUFSIZE;
      serial_count = 0; //clear buffer
    }
    else
    {
      if(serial_char == ';') comment_mode = true;
      if(!comment_mode) cmdbuffer[bufindw][serial_count++] = serial_char;
    }
  }
  #endif //SDSUPPORT
}


float code_value() 
{ 
  return (strtod(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL)); 
}

long code_value_long() 
{ 
  return (strtol(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL, 10)); 
}

bool code_seen(char code)
{
  strchr_pointer = strchr(cmdbuffer[bufindr], code);
  return (strchr_pointer != NULL);  //Return True if a character was found
}

#define DEFINE_PGM_READ_ANY(type, reader)		\
    static inline type pgm_read_any(const type *p)	\
	{ return pgm_read_##reader##_near(p); }

DEFINE_PGM_READ_ANY(float, float);
DEFINE_PGM_READ_ANY(signed char, byte);

#define XYZ_CONSTS_FROM_CONFIG(type, array, CONFIG) \
  const type array##_P[3] PROGMEM =            \
    { X_##CONFIG, Y_##CONFIG, Z_##CONFIG };  \
  static inline type array(int axis)           \
    { return pgm_read_any(&array##_P[axis]); }

XYZ_CONSTS_FROM_CONFIG(float, max_length, MAX_LENGTH);
XYZ_CONSTS_FROM_CONFIG(float, home_retract_mm, HOME_RETRACT_MM);

#if !defined(DUAL_X_DRIVE) && !defined(DUAL_Y_DRIVE)
  XYZ_CONSTS_FROM_CONFIG(float, base_home_pos, HOME_POS);
  XYZ_CONSTS_FROM_CONFIG(signed char, home_dir, HOME_DIR);
#else 
  #ifndef DUAL_X_DRIVE
  #  define X0_HOME_POS X_HOME_POS
  #  define X1_HOME_POS X_HOME_POS
  #  define X0_HOME_DIR X_HOME_DIR
  #  define X1_HOME_DIR X_HOME_DIR
  #endif // DUAL_X_DRIVE
  #ifndef DUAL_Y_DRIVE
  #  define Y0_HOME_POS Y_HOME_POS
  #  define Y1_HOME_POS Y_HOME_POS
  #  define Y0_HOME_DIR Y_HOME_DIR
  #  define Y1_HOME_DIR Y_HOME_DIR
  #endif // DUAL_Y_DRIVE
  const float base_home_pos_P[EXTRUDERS][3] PROGMEM = {
               { X0_HOME_POS, Y0_HOME_POS, Z_HOME_POS },
               { X1_HOME_POS, Y1_HOME_POS, Z_HOME_POS }		
  };
  static inline float base_home_pos(int axis)
               { return pgm_read_any(&base_home_pos_P[active_extruder][axis]); }
  const signed char home_dir_P[EXTRUDERS][3] PROGMEM = {
               { X0_HOME_DIR, Y0_HOME_DIR, Z_HOME_DIR },
               { X1_HOME_DIR, Y1_HOME_DIR, Z_HOME_DIR }
  };
  static inline signed char home_dir(int axis)
               { return pgm_read_any(&home_dir_P[active_extruder][axis]); }
#endif // !defined(DUAL_X_DRIVE) && !defined(DUAL_Y_DRIVE)

static void axis_is_at_home(int axis) {
  current_position[axis] = base_home_pos(axis);
  #ifdef ENABLE_ADD_HOMEING
  current_position[axis] += add_homeing[ACTIVE_EXTRUDER][axis];
  #endif // ENABLE_ADD_HOMEING
}

static void homeaxis(int axis) {
#define HOMEAXIS_DO(LETTER) \
  ((LETTER##_MIN_PIN > -1 && home_dir(LETTER##_AXIS) < 0) || (LETTER##_MAX_PIN > -1 && home_dir(LETTER##_AXIS) > 0))

  if (axis==X_AXIS ? HOMEAXIS_DO(X) :
        axis==Y_AXIS ? HOMEAXIS_DO(Y) :
          axis==Z_AXIS ? HOMEAXIS_DO(Z) : false) 
  {
    current_position[axis] = 0;
    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
    destination[axis] = 1.5 * max_length(axis) * home_dir(axis);
    feedrate = homing_feedrate[axis];
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
    st_synchronize();

    current_position[axis] = 0;
    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
    destination[axis] = -home_retract_mm(axis) * home_dir(axis);
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
    st_synchronize();

    destination[axis] = 2 * home_retract_mm(axis) * home_dir(axis);
    feedrate = homing_feedrate[axis]/2 ; 
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
    st_synchronize();

    axis_is_at_home(axis);					
    destination[axis] = current_position[axis];
    feedrate = 0.0;
    endstops_hit_on_purpose();
  }
}
#define HOMEAXIS(LETTER) homeaxis(LETTER##_AXIS)

#if EXTRUDERS > 1
// Makes all the necessary actions for switching extruders
static void set_active_extruder(uint8_t to_extruder, 
                                uint8_t e_starts_from_extruder, bool make_move)
{
  if(to_extruder != active_extruder) {
    // Save current position to return to after applying extruder offset
    memcpy(destination, current_position, sizeof(destination));
    // Offset extruder (only by XY)
    for(uint8_t i = 0; i < 2; i++) {
       current_position[i] = current_position[i] - 
                             extruder_offset[i][active_extruder] +
                             extruder_offset[i][to_extruder];
    }
    // Save the current coordinate of the active extruder
    e_last_position[active_extruder] = current_position[E_AXIS];
    #ifdef DUAL_X_DRIVE
    x_last_position[active_extruder] = current_position[X_AXIS];
    current_position[X_AXIS] = x_last_position[to_extruder];
    #endif // DUAL_X_DRIVE
    #ifdef DUAL_Y_DRIVE
    y_last_position[active_extruder] = current_position[Y_AXIS];
    current_position[Y_AXIS] = y_last_position[to_extruder];
    #endif // DUAL_Y_DRIVE
    // Clear the follow me mode for the selected extruder
    follow_me &= ~((int)to_extruder);
    // Set the active extruder
    active_extruder = to_extruder;
    // Restore E coordinate of the specified extruder
    current_position[E_AXIS] = e_last_position[e_starts_from_extruder];
    destination[E_AXIS] = current_position[E_AXIS];
    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
    // Move to the old position if 'F' was in the parameters
    if(make_move) {
       prepare_move();
    }
    // Make sure we complete all the planned moves before continuing
    st_synchronize();
  }
}
#endif // EXTRUDERS > 1

void process_commands()
{
  unsigned long codenum; //throw away variable
  char *starpos = NULL;

  if(code_seen('G'))
  {
    switch((int)code_value())
    {
    case 0: // G0 -> G1
    case 1: // G1
      if(Stopped == false) {
        get_coordinates(); // For X Y Z E F
        prepare_move();
        //ClearToSend();
        return;
      }
      //break;
    case 2: // G2  - CW ARC
      if(Stopped == false) {
        get_arc_coordinates();
        prepare_arc_move(true);
        return;
      }
    case 3: // G3  - CCW ARC
      if(Stopped == false) {
        get_arc_coordinates();
        prepare_arc_move(false);
        return;
      }
    case 4: // G4 dwell
      LCD_MESSAGEPGM(MSG_DWELL);
      codenum = 0;
      if(code_seen('P')) codenum = code_value(); // milliseconds to wait
      if(code_seen('S')) codenum = code_value() * 1000; // seconds to wait
      
      st_synchronize();
      codenum += millis();  // keep track of when we started waiting
      previous_millis_cmd = millis();
      while(millis()  < codenum ){
        manage_heater();
        manage_inactivity();
        lcd_update();
      }
      break;
      #ifdef FWRETRACT  
      case 10: // G10 retract
      if(!retracted) 
      {
        destination[X_AXIS]=current_position[X_AXIS];
        destination[Y_AXIS]=current_position[Y_AXIS];
        destination[Z_AXIS]=current_position[Z_AXIS]; 
        current_position[Z_AXIS]+=-retract_zlift;
        destination[E_AXIS]=current_position[E_AXIS]-retract_length; 
        feedrate=retract_feedrate;
        retracted=true;
        prepare_move();
      }
      
      break;
      case 11: // G10 retract_recover
      if(!retracted) 
      {
        destination[X_AXIS]=current_position[X_AXIS];
        destination[Y_AXIS]=current_position[Y_AXIS];
        destination[Z_AXIS]=current_position[Z_AXIS]; 
        
        current_position[Z_AXIS]+=retract_zlift;
        current_position[E_AXIS]+=-retract_recover_length; 
        feedrate=retract_recover_feedrate;
        retracted=false;
        prepare_move();
      }
      break;
      #endif //FWRETRACT
    case 28: //G28 Home all Axis one at a time
      saved_feedrate = feedrate;
      saved_feedmultiply = feedmultiply;
      feedmultiply = 100;
      previous_millis_cmd = millis();
      
      enable_endstops(true);
      
      for(int8_t i=0; i < NUM_AXIS; i++) {
        destination[i] = current_position[i];
      }
      feedrate = 0.0;
      home_all_axis = !((code_seen(axis_codes[0])) || (code_seen(axis_codes[1])) || (code_seen(axis_codes[2])));
      
      #if Z_HOME_DIR > 0                      // If homing away from BED do Z first
      if((home_all_axis) || (code_seen(axis_codes[Z_AXIS]))) {
        HOMEAXIS(Z);
      }
      #endif
      
      #ifdef QUICK_HOME
      if((home_all_axis)||( code_seen(axis_codes[X_AXIS]) && code_seen(axis_codes[Y_AXIS])) )  //first diagonal move
      {
        current_position[X_AXIS] = 0; current_position[Y_AXIS] = 0;
        
        plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]); 
        destination[X_AXIS] = 1.5 * X_MAX_LENGTH * home_dir(X_AXIS);
        destination[Y_AXIS] = 1.5 * Y_MAX_LENGTH * home_dir(Y_AXIS);
        feedrate = homing_feedrate[X_AXIS]; 
        if(homing_feedrate[Y_AXIS]<feedrate)
          feedrate = homing_feedrate[Y_AXIS]; 
        plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, ACTIVE_EXTRUDER);
        st_synchronize();
        
        axis_is_at_home(X_AXIS);
        axis_is_at_home(Y_AXIS);
        plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
        destination[X_AXIS] = current_position[X_AXIS];
        destination[Y_AXIS] = current_position[Y_AXIS];
        plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, ACTIVE_EXTRUDER);
        feedrate = 0.0;
        st_synchronize();
        endstops_hit_on_purpose();
      }
      #endif
      
      if((home_all_axis) || (code_seen(axis_codes[X_AXIS]))) {
        HOMEAXIS(X);
      }
      
      if((home_all_axis) || (code_seen(axis_codes[Y_AXIS]))) {
        HOMEAXIS(Y);
      }
      
      #if Z_HOME_DIR < 0                      // If homing towards BED do Z last
      if((home_all_axis) || (code_seen(axis_codes[Z_AXIS]))) {
        HOMEAXIS(Z);
      }
      #endif
      
      if(code_seen(axis_codes[X_AXIS])) 
      {
        if(code_value_long() != 0) {
          current_position[X_AXIS] = code_value();
          #ifdef ENABLE_ADD_HOMEING
          current_position[X_AXIS] += add_homeing[ACTIVE_EXTRUDER][0];
          #endif // ENABLE_ADD_HOMEING
        }
      }
      
      if(code_seen(axis_codes[Y_AXIS])) {
        if(code_value_long() != 0) {
          current_position[Y_AXIS] = code_value();
          #ifdef ENABLE_ADD_HOMEING
          current_position[Y_AXIS] += add_homeing[ACTIVE_EXTRUDER][1];
          #endif // ENABLE_ADD_HOMEING
        }
      }
      
      if(code_seen(axis_codes[Z_AXIS])) {
        if(code_value_long() != 0) {
          current_position[Z_AXIS]=code_value();
          #ifdef ENABLE_ADD_HOMEING
          current_position[Z_AXIS] += add_homeing[ACTIVE_EXTRUDER][2];
          #endif // ENABLE_ADD_HOMEING
        }
      }
      plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
      
      #ifdef ENDSTOPS_ONLY_FOR_HOMING
        enable_endstops(false);
      #endif
      
      feedrate = saved_feedrate;
      feedmultiply = saved_feedmultiply;
      previous_millis_cmd = millis();
      endstops_hit_on_purpose();
      break;
    case 90: // G90
      relative_mode = false;
      break;
    case 91: // G91
      relative_mode = true;
      break;
    case 92: // G92
      if(!code_seen(axis_codes[E_AXIS]))
        st_synchronize();
      for(int8_t i=0; i < NUM_AXIS; i++) {
        if(code_seen(axis_codes[i])) {
           current_position[i] = code_value();
           if(i == E_AXIS) {
             plan_set_e_position(current_position[E_AXIS]);
           }
           else {
             plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
           }
        }
      }
      break;
    }
  }

  else if(code_seen('M'))
  {
    switch( (int)code_value() ) 
    {
#ifdef ULTIPANEL
    case 0: // M0 - Unconditional stop - Wait for user button press on LCD
    case 1: // M1 - Conditional stop - Wait for user button press on LCD
    {
      LCD_MESSAGEPGM(MSG_USERWAIT);
      codenum = 0;
      if(code_seen('P')) codenum = code_value(); // milliseconds to wait
      if(code_seen('S')) codenum = code_value() * 1000; // seconds to wait
      
      st_synchronize();
      previous_millis_cmd = millis();
      if (codenum > 0){
        codenum += millis();  // keep track of when we started waiting
        while(millis()  < codenum && !LCD_CLICKED){
          manage_heater();
          manage_inactivity();
          lcd_update();
        }
      }else{
        while(!LCD_CLICKED){
          manage_heater();
          manage_inactivity();
          lcd_update();
        }
      }
      LCD_MESSAGEPGM(MSG_RESUMING);
    }
    break;
#endif
    case 17:
        LCD_MESSAGEPGM(MSG_NO_MOVE);
        enable_x(); 
        enable_y(); 
        enable_z(); 
        enable_e0(); 
        enable_e1(); 
        enable_e2(); 
      break;

#ifdef SDSUPPORT
    case 20: // M20 - list SD card
      SERIAL_PROTOCOLLNPGM(MSG_BEGIN_FILE_LIST);
      card.ls();
      SERIAL_PROTOCOLLNPGM(MSG_END_FILE_LIST);
      break;
      
    case 21: // M21 - init SD card
      card.initsd();
      break;
      
    case 22: //M22 - release SD card
      card.release();
      break;
      
    case 23: //M23 - Select file
      starpos = (strchr(strchr_pointer + 4,'*'));
      if(starpos!=NULL)
        *(starpos-1)='\0';
      card.openFile(strchr_pointer + 4,true);
      break;
      
    case 24: //M24 - Start SD print
      card.startFileprint();
      starttime=millis();
      break;
      
    case 25: //M25 - Pause SD print
      card.pauseSDPrint();
      break;
      
    case 26: //M26 - Set SD index
      if(card.cardOK && code_seen('S')) {
        card.setIndex(code_value_long());
      }
      break;
      
    case 27: //M27 - Get SD status
      card.getStatus();
      break;
      
    case 28: //M28 - Start SD write
      starpos = (strchr(strchr_pointer + 4,'*'));
      if(starpos != NULL){
        char* npos = strchr(cmdbuffer[bufindr], 'N');
        strchr_pointer = strchr(npos,' ') + 1;
        *(starpos-1) = '\0';
      }
      card.openFile(strchr_pointer+4,false);
      break;
      
    case 29: //M29 - Stop SD write
      //processed in write to file routine above
      //card.saving = false;
      break;
      
    case 30: //M30 <filename> Delete File 
      if(card.cardOK) {
        card.closefile();
        starpos = (strchr(strchr_pointer + 4,'*'));
        if(starpos != NULL){
          char* npos = strchr(cmdbuffer[bufindr], 'N');
          strchr_pointer = strchr(npos,' ') + 1;
          *(starpos-1) = '\0';
        }
        card.removeFile(strchr_pointer + 4);
      }
      break;
#endif //SDSUPPORT

    case 31: //M31 take time since the start of the SD print or an M109 command
      {
      stoptime=millis();
      char time[30];
      unsigned long t=(stoptime-starttime)/1000;
      int sec,min;
      min=t/60;
      sec=t%60;
      sprintf_P(time, PSTR("%i min, %i sec"), min, sec);
      SERIAL_ECHO_START;
      SERIAL_ECHOLN(time);
      lcd_setstatus(time);
      autotempShutdown();
      }
      break;

    case 42: //M42 -Change pin status via gcode
      if (code_seen('S'))
      {
        int pin_status = code_value();
        int pin_number = LED_PIN;
        if (code_seen('P') && pin_status >= 0 && pin_status <= 255)
          pin_number = code_value();
        for(int8_t i = 0; i < (int8_t)sizeof(sensitive_pins); i++)
        {
          if (sensitive_pins[i] == pin_number)
          {
            pin_number = -1;
            break;
          }
        }
        if (pin_number > -1)
        {
          pinMode(pin_number, OUTPUT);
          digitalWrite(pin_number, pin_status);
          analogWrite(pin_number, pin_status);
        }
      }
     break;

    case 104: // M104
      if(setTargetedHotend(104)){
        break;
      }
      if (code_seen('S')) { 
        setTargetHotend(code_value(), tmp_extruder);
        temp_adjustment = 0;
      }
      // Allow only one temperature adjustment lower or higher.
      if (code_seen('L')) {
        if(temp_adjustment >= 0) {
          int deg = degTargetHotend(tmp_extruder);
          deg = (deg > code_value()) ? (deg - code_value()) : 0;
          setTargetHotend(deg, tmp_extruder);
          temp_adjustment--;
        } 
        else if(temp_adjustment < 0)
        {
          SERIAL_ECHO_START;
          SERIAL_ECHOLN(MSG_TEMP_ADJ_ERROR);
          break;
        }
      }
      if (code_seen('H')) {
        if (temp_adjustment <= 0) {
          int deg = degTargetHotend(tmp_extruder) + code_value();
          // if heating over max value should the machine will shut down
          setTargetHotend(deg, tmp_extruder);
          temp_adjustment++;
        }
        else if(temp_adjustment > 0)
        {
          SERIAL_ECHO_START;
          SERIAL_ECHOLN(MSG_TEMP_ADJ_ERROR);
          break;
        }
      }

      setWatch();

      SERIAL_ECHO_START;
      SERIAL_ECHOPGM(MSG_TEMPERATURE_TGT);
      SERIAL_ECHOPAIR(" T", (int)tmp_extruder);
      SERIAL_ECHOPAIR(":", (int)(degTargetHotend(tmp_extruder) + 0.5));
      SERIAL_ECHOLN("");
      break;

    case 140: // M140 set bed temp
      if (code_seen('S')) setTargetBed(code_value());
      break;

    case 105 : // M105
      if(setTargetedHotend(105)){
        break;
      }
      codenum = false;
      if(code_seen('A') && code_value() != 0) {
        codenum = true;
        tmp_extruder = active_extruder;
      }
      SERIAL_PROTOCOLPGM("ok ");
      printHeatersState((bool)codenum, tmp_extruder);
      break;

    case 109: 
    {// M109 - Wait for extruder heater to reach target.
      int start_e, end_e, deg;
      if(setTargetedHotend(109)) {
        break;
      }
      
      bool new_mode = (code_seen('A') && code_value() > 0);
    #if EXTRUDERS > 1
      if ((new_mode) || (follow_me_heater)) 
    #else // EXTRUDERS > 1
      if (new_mode) 
    #endif // EXTRUDERS > 1
      {
        start_e = 0;
        end_e = EXTRUDERS;
      }
      else
      {
        start_e = tmp_extruder; 
        end_e = tmp_extruder + 1;
      }
      LCD_MESSAGEPGM(MSG_HEATING);   
    #ifdef AUTOTEMP
      autotemp_enabled = false;
    #endif
      if (code_seen('S')) { 
        setTargetHotend((deg = code_value()), tmp_extruder);
        temp_adjustment = 0;
      }
      if (code_seen('L')) {
        if(temp_adjustment >= 0) {
          deg = degTargetHotend(tmp_extruder);
          deg = (deg > code_value()) ? (deg - code_value()) : 0;
          setTargetHotend(deg, tmp_extruder);
          temp_adjustment--;
        }
        else
        {
          SERIAL_ECHO_START;
          SERIAL_ECHOLN(MSG_TEMP_ADJ_ERROR);
          break;
        }
      }
      if (code_seen('H')) {
        if (temp_adjustment <= 0) {
          deg = degTargetHotend(tmp_extruder) + code_value();
          // if heating over max value should the machine will shut down
          setTargetHotend(deg, tmp_extruder);
          temp_adjustment++;
        }
        else
        {
          SERIAL_ECHO_START;
          SERIAL_ECHOLN(MSG_TEMP_ADJ_ERROR);
          break;
        }
      }

    #ifdef AUTOTEMP
      if (code_seen('S')) autotemp_min=deg;
      if (code_seen('G')) autotemp_max=code_value();
      if (code_seen('F')) 
      {
        autotemp_factor=code_value();
        autotemp_enabled=true;
      }
    #endif

      SERIAL_ECHO_START;
      SERIAL_ECHOPGM(MSG_TEMPERATURE_TGT);
      SERIAL_ECHOPAIR(" T", (int)tmp_extruder);
      SERIAL_ECHOPAIR(":", (int)degTargetHotend(tmp_extruder));
      SERIAL_ECHOLN("");
      
      setWatch();
      codenum = millis(); 

    #ifdef TEMP_RESIDENCY_TIME
      long residencyStart = -1;
      int residencyTime = (code_seen('W')) ? code_value() : TEMP_RESIDENCY_TIME;
    #endif //TEMP_RESIDENCY_TIME
      bool done_temp = false;
      while(!done_temp)
      {
        if((millis() - codenum) > 1000) 
        {
          int i;
          // See if target extruder(s) have reached the temperature 
          done_temp = true;
          for(i = start_e; i < end_e; i++)
          {
            done_temp &= (degTargetHotend(i) == 0) || 
                         (labs(degHotend(i) - degTargetHotend(i)) <= TEMP_WINDOW);
          }
          printHeatersState(new_mode, tmp_extruder);

          // If using old style (no A1 option) output, use echo to display all extruders info.
        #if (EXTRUDERS > 1) || TEMP_RESIDENCY_TIME
          SERIAL_ECHO_START;
        #endif // (EXTRUDERS > 1) || TEMP_RESIDENCY_TIME
        #if (EXTRUDERS > 1)
          if(!new_mode) {
            for(i = 0; i < EXTRUDERS; i++)
            {
              SERIAL_PROTOCOLPGM(" Ext");
              SERIAL_PROTOCOL(i); 
              SERIAL_PROTOCOLPGM(":");
              SERIAL_PROTOCOL((int)(degHotend(i) + 0.5)); 
            }
          }
        #endif
        #ifdef TEMP_RESIDENCY_TIME
          // Print the waiting counter
          SERIAL_PROTOCOLPGM(" Wait:");
          if(residencyStart >= 0)
          {
            codenum = residencyTime - ((millis() - residencyStart) / 1000);
            SERIAL_PROTOCOLLN(codenum);
          }
          else 
          {
            SERIAL_PROTOCOLLN("?");
          }
          // Logic to handle wait for temperature to stabilize
          if(!done_temp) // Still reaching the target teperature(s)
          {
            residencyStart = -1;
          }
          else if(residencyStart < 0) // Start waiting for stabilization
          {
            residencyStart = millis();
            done_temp = false;
          }
          else if(codenum > 0) // have to wait
          {
            done_temp = false;
          }
        #endif
          codenum = millis();
        }
        manage_heater();
        manage_inactivity();
        lcd_update();
      }
      LCD_MESSAGEPGM(MSG_HEATING_COMPLETE);
      starttime=millis();
    }
    break;

    case 190: // M190 - Wait for bed heater to reach target.
    {
    #if TEMP_BED_PIN > -1
      LCD_MESSAGEPGM(MSG_BED_HEATING);
      bool new_mode = (code_seen('A') && code_value() > 0);
      if (code_seen('S')) setTargetBed(code_value());
      codenum = millis(); 
      while(!isDoneHeatingBed())
      {
        if( (millis()-codenum) > 1000 ) //Print Temp Reading every 1 second while heating up.
        {
          printHeatersState(new_mode, tmp_extruder);
          codenum = millis(); 
        }
        manage_heater();
        manage_inactivity();
        lcd_update();
      }
      LCD_MESSAGEPGM(MSG_BED_DONE);
    #endif
    }
    break;

    case 106: //M106 Fan On
      if(setTargetedHotend(106)) {
        break;
      }
      if (code_seen('S')){
        fanSpeed[tmp_extruder] = constrain(code_value(),0,255);
      }
      else {
        fanSpeed[tmp_extruder] = 255;			
      }
      if (code_seen('A') && code_value() != 0) {
        for(int i=0; i < EXTRUDERS; i++) {
          fanSpeed[i] = fanSpeed[tmp_extruder];
        }
      }
      break;

    case 107: //M107 Fan Off
      if(setTargetedHotend(107)) {
        break;
      }
      if (code_seen('A') && code_value() != 0) {
        for(int i=0; i < EXTRUDERS; i++) fanSpeed[i] = 0;
      } else {
        fanSpeed[tmp_extruder] = 0;
      }
      break;

  #if (PS_ON_PIN > -1)
    case 80: // M80 - ATX Power On
      SET_OUTPUT(PS_ON_PIN); //GND
      WRITE(PS_ON_PIN, PS_ON_AWAKE);
      break;
  #endif

    case 81: // M81 - ATX Power Off
      #if defined SUICIDE_PIN && SUICIDE_PIN > -1
        st_synchronize();
        suicide();
      #elif (PS_ON_PIN > -1)
        SET_OUTPUT(PS_ON_PIN); 
        WRITE(PS_ON_PIN, PS_ON_ASLEEP);
      #endif
		break;
        
    case 82:
      axis_relative_modes[3] = false;
      break;

    case 83:
      axis_relative_modes[3] = true;
      break;

    case 18: //compatibility
    case 84: // M84
      if(code_seen('S')){ 
        stepper_inactive_time = code_value() * 1000; 
      }
      else
      { 
        bool all_axis = !((code_seen(axis_codes[0])) || (code_seen(axis_codes[1])) || (code_seen(axis_codes[2]))|| (code_seen(axis_codes[3])));
        if(all_axis)
        {
          st_synchronize();
          disable_e0();
          disable_e1();
          disable_e2();
          finishAndDisableSteppers();
        }
        else
        {
          st_synchronize();
          #ifndef DUAL_X_DRIVE
          if(code_seen('X')) disable_x();
          #else  // DUAL_X_DRIVE
          if(code_seen('X')) { disable_x0(); disable_x1(); }
          #endif // DUAL_X_DRIVE
          #ifndef DUAL_Y_DRIVE
          if(code_seen('Y')) disable_y();
          #else  // DUAL_Y_DRIVE
          if(code_seen('Y')) { disable_y0(); disable_y1(); }
          #endif // DUAL_Y_DRIVE
          if(code_seen('Z')) disable_z();
          #if ((E0_ENABLE_PIN != X_ENABLE_PIN) && (E1_ENABLE_PIN != Y_ENABLE_PIN)) // Only enable on boards that have seperate ENABLE_PINS
            if(code_seen('E')) {
              disable_e0();
              disable_e1();
              disable_e2();
            }
          #endif 
        }
      }
      break;
    case 85: // M85
      code_seen('S');
      max_inactive_time = code_value() * 1000; 
      break;
    case 92: // M92
      if(setTargetedHotend(92)) {
        break;
      }
      for(int8_t i=0; i < NUM_AXIS; i++) 
      {
        if(code_seen(axis_codes[i])) 
        {
          if(i == E_AXIS) { 
            axis_steps_per_unit[i + tmp_extruder] = code_value();
          }
          else {
            axis_steps_per_unit[i] = code_value();
          }
        }
      }
      break;
    case 115: // M115
      SERIAL_PROTOCOLPGM(MSG_M115_REPORT);
      break;
    case 117: // M117 display message
      starpos = (strchr(strchr_pointer + 5,'*'));
      if(starpos!=NULL)
        *(starpos-1)='\0';
      lcd_setstatus(strchr_pointer + 5);
      break;
    case 114: // M114
      SERIAL_PROTOCOLPGM("X:");
      SERIAL_PROTOCOL(current_position[X_AXIS]);
      SERIAL_PROTOCOLPGM("Y:");
      SERIAL_PROTOCOL(current_position[Y_AXIS]);
      SERIAL_PROTOCOLPGM("Z:");
      SERIAL_PROTOCOL(current_position[Z_AXIS]);
      SERIAL_PROTOCOLPGM("E:");
      SERIAL_PROTOCOL(current_position[E_AXIS]);
      
      SERIAL_PROTOCOLPGM(MSG_COUNT_X);
      SERIAL_PROTOCOL(float(st_get_position(X_AXIS))/axis_steps_per_unit[X_AXIS]);
      SERIAL_PROTOCOLPGM("Y:");
      SERIAL_PROTOCOL(float(st_get_position(Y_AXIS))/axis_steps_per_unit[Y_AXIS]);
      SERIAL_PROTOCOLPGM("Z:");
      SERIAL_PROTOCOL(float(st_get_position(Z_AXIS))/axis_steps_per_unit[Z_AXIS]);
      
      SERIAL_PROTOCOLLN("");
      break;
    case 120: // M120
      enable_endstops(false) ;
      break;
    case 121: // M121
      enable_endstops(true) ;
      break;
    case 119: // M119
      SERIAL_PROTOCOLLN(MSG_M119_REPORT);
      #if (X_MIN_PIN > -1)
        SERIAL_PROTOCOLPGM(MSG_X_MIN);
        SERIAL_PROTOCOLLN(((READ(X_MIN_PIN)^X_ENDSTOPS_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
      #endif
      #if (X_MAX_PIN > -1)
        SERIAL_PROTOCOLPGM(MSG_X_MAX);
        SERIAL_PROTOCOLLN(((READ(X_MAX_PIN)^X_ENDSTOPS_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
      #endif
      #if (Y_MIN_PIN > -1)
        SERIAL_PROTOCOLPGM(MSG_Y_MIN);
        SERIAL_PROTOCOLLN(((READ(Y_MIN_PIN)^Y_ENDSTOPS_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
      #endif
      #if (Y_MAX_PIN > -1)
        SERIAL_PROTOCOLPGM(MSG_Y_MAX);
        SERIAL_PROTOCOLLN(((READ(Y_MAX_PIN)^Y_ENDSTOPS_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
      #endif
      #if (Z_MIN_PIN > -1)
        SERIAL_PROTOCOLPGM(MSG_Z_MIN);
        SERIAL_PROTOCOLLN(((READ(Z_MIN_PIN)^Z_ENDSTOPS_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
      #endif
      #if (Z_MAX_PIN > -1)
        SERIAL_PROTOCOLPGM(MSG_Z_MAX);
        SERIAL_PROTOCOLLN(((READ(Z_MAX_PIN)^Z_ENDSTOPS_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
      #endif
      break;
      //TODO: update for all axis, use for loop
    case 201: // M201
      if(setTargetedHotend(201)) {
        break;
      }
      for(int8_t i=0; i < NUM_AXIS; i++) 
      {
        if(code_seen(axis_codes[i])) {
          if(i == E_AXIS) {
            max_acceleration_units_per_sq_second[i + tmp_extruder] = code_value();
          } 
          else {
            max_acceleration_units_per_sq_second[i] = code_value();
          }
        }
      }
      break;
    case 203: // M203 max feedrate mm/sec
      if(setTargetedHotend(203)) {
        break;
      }
      for(int8_t i=0; i < NUM_AXIS; i++) {
        if(code_seen(axis_codes[i])) {
          if(i == E_AXIS) {
            max_feedrate[i + tmp_extruder] = code_value();
          } else {
            max_feedrate[i] = code_value();
          }
        }
      }
      break;
    case 204: // M204 acclereration S normal moves T filmanent only moves
      if(setTargetedHotend(204)) {
        break;
      }
      if(code_seen('S')) acceleration = code_value();
      if(code_seen('R')) retract_acceleration[tmp_extruder] = code_value();
      break;
    case 205: // M205 advanced settings:  minimum travel speed S=while printing T=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk
    {
      if(setTargetedHotend(205)) {
        break;
      }
      if(code_seen('S')) minimumfeedrate = code_value();
      if(code_seen('V')) mintravelfeedrate = code_value();
      if(code_seen('B')) minsegmenttime = code_value() ;
      if(code_seen('X')) max_xy_jerk = code_value() ;
      if(code_seen('Z')) max_z_jerk = code_value() ;
      if(code_seen('E')) max_e_jerk[tmp_extruder] = code_value() ;
    }
    break;
    #ifdef ENABLE_ADD_HOMEING
    case 206: // M206 additional homeing offset
      if(setTargetedHotend(206)) {
        break;
      }
      for(int8_t i=0; i < 3; i++) 
      {
        if(code_seen(axis_codes[i])) add_homeing[tmp_extruder][i] = code_value();
      }
      break;
    #endif // ENABLE_ADD_HOMEING
    #ifdef FWRETRACT
    case 207: //M207 - set retract length S[positive mm] F[feedrate mm/sec] Z[additional zlift/hop]
    {
      if(code_seen('S')) 
      {
        retract_length = code_value() ;
      }
      if(code_seen('F')) 
      {
        retract_feedrate = code_value() ;
      }
      if(code_seen('Z')) 
      {
        retract_zlift = code_value() ;
      }
    }break;
    case 208: // M208 - set retract recover length S[positive mm surplus to the M207 S*] F[feedrate mm/sec]
    {
      if(code_seen('S')) 
      {
        retract_recover_length = code_value() ;
      }
      if(code_seen('F')) 
      {
        retract_recover_feedrate = code_value() ;
      }
    }break;
    case 209: // M209 - S<1=true/0=false> enable automatic retract detect if the slicer did not support G10/11: every normal extrude-only move will be classified as retract depending on the direction.
    {
      if(code_seen('S')) 
      {
        int t= code_value() ;
        switch(t)
        {
          case 0: autoretract_enabled=false;retracted=false;break;
          case 1: autoretract_enabled=true;retracted=false;break;
          default: 
            SERIAL_ECHO_START;
            SERIAL_ECHOPGM(MSG_UNKNOWN_COMMAND);
            SERIAL_ECHO(cmdbuffer[bufindr]);
            SERIAL_ECHOLNPGM("\"");
        }
      }
      
    }break;
    #endif // FWRETRACT
    #if EXTRUDERS > 1
    case 218: // M218 - set hotend offset (in mm), T<extruder_number> X<offset_on_X> Y<offset_on_Y>
    {
      if(setTargetedHotend(218)){
        break;
      }
      if(code_seen('X')) 
      {
        extruder_offset[X_AXIS][tmp_extruder] = code_value();
      }
      if(code_seen('Y'))
      {
        extruder_offset[Y_AXIS][tmp_extruder] = code_value();
      }
      SERIAL_ECHO_START;
      SERIAL_ECHOPGM(MSG_HOTEND_OFFSET);
      for(tmp_extruder = 0; tmp_extruder < EXTRUDERS; tmp_extruder++) 
      {
         SERIAL_ECHO(" ");
         SERIAL_ECHO(extruder_offset[X_AXIS][tmp_extruder]);
         SERIAL_ECHO(",");
         SERIAL_ECHO(extruder_offset[Y_AXIS][tmp_extruder]);
      }
      SERIAL_ECHOLN("");
    }
    break;
    #endif
    case 220: // M220 S<factor in percent>- set speed factor override percentage
    {
      if(code_seen('S')) 
      {
        feedmultiply = code_value() ;
      }
    }
    break;
    case 221: // M221 S<factor in percent>- set extrude factor override percentage
    {
      if(code_seen('S')) 
      {
        extrudemultiply = code_value() ;
      }
    }
    break;

    case 240: // M240  Triggers a camera by emulating a Canon RC-1 : http://www.doc-diy.net/photo/rc-1_hacked/
    {
      #ifdef PHOTOGRAPH_PIN
        #if (PHOTOGRAPH_PIN > -1)
        const uint8_t NUM_PULSES=16;
        const float PULSE_LENGTH=0.01524;
        for(int i=0; i < NUM_PULSES; i++) {
          WRITE(PHOTOGRAPH_PIN, HIGH);
          _delay_ms(PULSE_LENGTH);
          WRITE(PHOTOGRAPH_PIN, LOW);
          _delay_ms(PULSE_LENGTH);
        }
        delay(7.33);
        for(int i=0; i < NUM_PULSES; i++) {
          WRITE(PHOTOGRAPH_PIN, HIGH);
          _delay_ms(PULSE_LENGTH);
          WRITE(PHOTOGRAPH_PIN, LOW);
          _delay_ms(PULSE_LENGTH);
        }
        #endif
      #endif
    }
    break;
      
    #ifdef PIDTEMP
    case 301: // M301
      {
        if(code_seen('P')) Kp = code_value();
        if(code_seen('I')) Ki = code_value()*PID_dT;
        if(code_seen('D')) Kd = code_value()/PID_dT;
        #ifdef PID_ADD_EXTRUSION_RATE
        if(code_seen('C')) Kc = code_value();
        #endif
        #ifdef PID_FUNCTIONAL_RANGE
        if(code_seen('R')) Kr = code_value();
        #endif
        updatePID();
        SERIAL_PROTOCOL(MSG_OK);
        SERIAL_PROTOCOL(" p:");
        SERIAL_PROTOCOL(Kp);
        SERIAL_PROTOCOL(" i:");
        SERIAL_PROTOCOL(Ki/PID_dT);
        SERIAL_PROTOCOL(" d:");
        SERIAL_PROTOCOL(Kd*PID_dT);
        #ifdef PID_FUNCTIONAL_RANGE
        SERIAL_PROTOCOL(" r:");
        SERIAL_PROTOCOL(Kr);
        #endif
        #ifdef PID_ADD_EXTRUSION_RATE
        SERIAL_PROTOCOL(" c:");
        SERIAL_PROTOCOL(Kc*PID_dT);
        #endif
        SERIAL_PROTOCOLLN("");
      }
      break;
    #endif //PIDTEMP
    #ifdef PIDTEMPBED
    case 304: // M304
      {
        if(code_seen('P')) bedKp = code_value();
        if(code_seen('I')) bedKi = code_value()*PID_dT;
        if(code_seen('D')) bedKd = code_value()/PID_dT;
        updatePID();
        SERIAL_PROTOCOL(MSG_OK);
        SERIAL_PROTOCOL(" p:");
        SERIAL_PROTOCOL(bedKp);
        SERIAL_PROTOCOL(" i:");
        SERIAL_PROTOCOL(bedKi/PID_dT);
        SERIAL_PROTOCOL(" d:");
        SERIAL_PROTOCOL(bedKd*PID_dT);
        SERIAL_PROTOCOLLN("");
      }
      break;
    #endif //PIDTEMP

    case 302: // allow cold extrudes
    {
      allow_cold_extrudes(true);
    }
    break;
    case 303: // M303 PID autotune
    {
      float temp = 150.0;
      int e=0;
      int c=5;
      if (code_seen('E')) e=code_value();
      if (e<0) temp = 70;
      if (code_seen('S')) temp=code_value();
      if (code_seen('C')) c=code_value();
      PID_autotune(temp, e, c);
    }
    break;

    #if EXTRUDERS > 1
    case 322: // M322 set up or show follow me settings
    {
      int mask;
      if(!code_seen('T')) {
        mask = (1<<EXTRUDERS) - 1;
      } else if(setTargetedHotend(322)) {
        break;
      } else {
        mask = 1<<tmp_extruder;
      }
      if(code_seen('S')) {
        if(code_value() == 0) {
          follow_me &= ~mask;
        } else {
          follow_me |= mask;
        }
      }
      follow_me &= ~(1<<ACTIVE_EXTRUDER);
      // enable/disable following active hotend temperature settings 
      if(code_seen('H')) {
        follow_me_heater = (code_value() != 0);
      }
      #ifdef PER_EXTRUDER_FANS
      // enable/disable following active hotend fan speed
      if(code_seen('F')) {
        follow_me_fan = (code_value() != 0);
      }
      #endif // PER_EXTRUDER_FANS
      // Print the follow me setup
      SERIAL_ECHO_START;
      SERIAL_ECHOPGM(MSG_FOLLOWME_MODE);
      for(tmp_extruder = 0; tmp_extruder < EXTRUDERS; tmp_extruder++) 
      {
         SERIAL_ECHOPAIR(" T", (int)tmp_extruder);
         SERIAL_ECHOPAIR(":", (follow_me & (1<<tmp_extruder)) ? "on" : "off");
      }
      SERIAL_ECHOPAIR(" H:", follow_me_heater ? "on" : "off");
      #ifdef PER_EXTRUDER_FANS
      SERIAL_ECHOPAIR(" F:", follow_me_fan ? "on" : "off");
      #endif // PER_EXTRUDER_FANS
      SERIAL_ECHOLN("");
    }
    break;
    #endif // EXTRUDERS > 1

    case 331: // M331 - save current position
    {
      int slot = 0;
      if(code_seen('S')){
        slot = code_value();
      }
      if(slot < 0 || slot >= NUM_POSITON_SLOTS) {
        SERIAL_ECHOPAIR(MSG_INVALID_POS_SLOT, (int)NUM_POSITON_SLOTS);
        break;
      } 
      memcpy(saved_position[slot], current_position, sizeof(*saved_position));
      SERIAL_ECHO_START;
      SERIAL_ECHO(MSG_SAVED_POS);
      SERIAL_ECHOPAIR(" S", slot);
      SERIAL_ECHOPAIR("<-X:", saved_position[slot][X_AXIS]);
      SERIAL_ECHOPAIR(" Y:", saved_position[slot][Y_AXIS]);
      SERIAL_ECHOPAIR(" Z:", saved_position[slot][Z_AXIS]);
      SERIAL_ECHOPAIR(" E:", saved_position[slot][E_AXIS]);
      SERIAL_ECHOLN("");
    }
    break;

    case 332: // M332 - restore position
    {
      bool make_move = false;
      int slot = 0;
      if(code_seen('S')){
        slot = code_value();
      }
      if(slot < 0 || slot >= NUM_POSITON_SLOTS) {
        SERIAL_ECHOPAIR(MSG_INVALID_POS_SLOT, (int)NUM_POSITON_SLOTS);
        break;
      } 
      SERIAL_ECHO_START;
      SERIAL_ECHO(MSG_RESTORING_POS);
      SERIAL_ECHOPAIR(" S", slot);
      SERIAL_ECHO("->");
      if(code_seen('F') && (next_feedrate = code_value()) > 0.0) {
        feedrate = next_feedrate;
        make_move = true;
      }
      for(int i=0; i< NUM_AXIS; i++) {
        float coord = saved_position[slot][i];
        if(code_seen(axis_codes[i]) && code_value() == 0) {
          coord = current_position[i];
        }
        if(make_move) {
          destination[i] = coord;
        }
        else {
          current_position[i] = coord;
          if(i == E_AXIS) {
            plan_set_e_position(current_position[E_AXIS]);
          }
          else {
            plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
          }
        }
        SERIAL_ECHOPAIR(" ", axis_codes[i]);
        SERIAL_ECHOPAIR(":", coord);
      }
      SERIAL_ECHOLN("");
      if(make_move) {
         prepare_move();
         st_synchronize();
      }
    }
    break;
    
    #ifdef C_COMPENSATION
    case 340: // M340 configure filament compression compensation table
    {
      if(setTargetedHotend(340)){
        break;
      }
      int pos;
      if(code_seen('P')) {
        pos = code_value();
        if(pos < 0 || pos >= gCComp_max_size) {
          SERIAL_ECHO_START;
          SERIAL_ECHO(MSG_CCOMP_INVALID_POS " ");
          SERIAL_ECHOLN(pos);
          break;
        }
        if(code_seen('S')) {
          gCComp[pos][tmp_extruder][0] = code_value();
        }
        if(code_seen('C')) {
          gCComp[pos][tmp_extruder][1] = code_value();
        }
        // Figure out the compensation table size
        int size;
        for(size = 0; 
            size < gCComp_max_size && gCComp[size][tmp_extruder][0] > 0.0;
            size++);
        gCComp_size[tmp_extruder] = size;
      }
      if(code_seen('R')) {
        gCCom_min_speed[tmp_extruder] = code_value();
      }
      // Print the compensation table
      SERIAL_ECHO_START;
      SERIAL_ECHOLN(MSG_CCOMP_TABLE);
      SERIAL_ECHO_START;
      SERIAL_ECHOPAIR("T:", (int)tmp_extruder);
      SERIAL_ECHOPAIR(" R:", gCCom_min_speed[tmp_extruder]);
      SERIAL_ECHOLN("");
      for(pos = 0; pos < gCComp_size[tmp_extruder]; pos++) 
      {
         SERIAL_ECHO_START;
         SERIAL_ECHO(pos);
         SERIAL_ECHOPAIR(": S:", gCComp[pos][tmp_extruder][0]);
         SERIAL_ECHOPAIR(" C:", gCComp[pos][tmp_extruder][1]);
         SERIAL_ECHOLN("");
      }
    }
    break;
    #endif // C_COMPENSATION
    
    #ifdef ENABLE_MICROSTEPPING_CONTROL
    case 350: // M350 Set microstepping mode. Warning: Steps per unit remains unchanged. S code sets stepping mode for all drivers.
    {
        if(code_seen('S')) for(int i=0;i<=4;i++) microstep_mode(i,code_value()); 
        for(int i=0;i<=NUM_AXIS;i++) if(code_seen(axis_codes[i])) microstep_mode(i,(uint8_t)code_value());
        if(code_seen('B')) microstep_mode(4,code_value());
        microstep_readings();
    }
    break;
    case 351: // M351 Toggle MS1 MS2 pins directly, S# determines MS1 or MS2, X# sets the pin high/low.
    {
      if(code_seen('S')) switch((int)code_value())
      {
        case 1:
          for(int i=0;i<=NUM_AXIS;i++) if(code_seen(axis_codes[i])) microstep_ms(i,code_value(),-1);
          if(code_seen('B')) microstep_ms(4,code_value(),-1);
          break;
        case 2:
          for(int i=0;i<=NUM_AXIS;i++) if(code_seen(axis_codes[i])) microstep_ms(i,-1,code_value());
          if(code_seen('B')) microstep_ms(4,-1,code_value());
          break;
      }
      microstep_readings();
    }
    break;
    #endif // ENABLE_MICROSTEPPING_CONTROL
    case 400: // M400 finish all moves
    {
      st_synchronize();
    }
    break;
    case 500: // M500 Store settings in EEPROM
    {
        Config_StoreSettings();
    }
    break;
    case 501: // M501 Read settings from EEPROM
    {
        Config_RetrieveSettings();
    }
    break;
    case 502: // M502 Revert to default settings
    {
        Config_ResetDefault();
    }
    break;
    case 503: // M503 print settings currently in memory
    {
        Config_PrintSettings();
    }
    break;
    #ifdef ENABLE_DEBUG
    case 504: // set debug flags
    {
      if(code_seen('S')) 
      {
        debug_flags = code_value();
      }
      if(code_seen('P')) 
      {
        if((((unsigned short)code_value()) & DEBUG_PRINT_PLAN) != 0) {
          planner_print_plan();
        }
      }
      SERIAL_ECHO_START;
      SERIAL_ECHO(MSG_DBG_FLAG);
      SERIAL_ECHOLN(debug_flags);
    }
    break;
    #endif // ENABLE_DEBUG
    #ifdef ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED
    case 540:
    {
      if(code_seen('S')) abort_on_endstop_hit = code_value() > 0;
    }
    break;
    #endif
    #ifdef FILAMENTCHANGEENABLE
    case 600: //Pause for filament change X[pos] Y[pos] Z[relative lift] E[initial retract] L[later retract distance for removal]
    {
        float target[4];
        float lastpos[4];
        target[X_AXIS]=current_position[X_AXIS];
        target[Y_AXIS]=current_position[Y_AXIS];
        target[Z_AXIS]=current_position[Z_AXIS];
        target[E_AXIS]=current_position[E_AXIS];
        lastpos[X_AXIS]=current_position[X_AXIS];
        lastpos[Y_AXIS]=current_position[Y_AXIS];
        lastpos[Z_AXIS]=current_position[Z_AXIS];
        lastpos[E_AXIS]=current_position[E_AXIS];
        //retract by E
        if(code_seen('E')) 
        {
          target[E_AXIS]+= code_value();
        }
        else
        {
          #ifdef FILAMENTCHANGE_FIRSTRETRACT
            target[E_AXIS]+= FILAMENTCHANGE_FIRSTRETRACT ;
          #endif
        }
        plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder);
        
        //lift Z
        if(code_seen('Z')) 
        {
          target[Z_AXIS]+= code_value();
        }
        else
        {
          #ifdef FILAMENTCHANGE_ZADD
            target[Z_AXIS]+= FILAMENTCHANGE_ZADD ;
          #endif
        }
        plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder);
        
        //move xy
        if(code_seen('X')) 
        {
          target[X_AXIS]+= code_value();
        }
        else
        {
          #ifdef FILAMENTCHANGE_XPOS
            target[X_AXIS]= FILAMENTCHANGE_XPOS ;
          #endif
        }
        if(code_seen('Y')) 
        {
          target[Y_AXIS]= code_value();
        }
        else
        {
          #ifdef FILAMENTCHANGE_YPOS
            target[Y_AXIS]= FILAMENTCHANGE_YPOS ;
          #endif
        }
        
        plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder);
        
        if(code_seen('L'))
        {
          target[E_AXIS]+= code_value();
        }
        else
        {
          #ifdef FILAMENTCHANGE_FINALRETRACT
            target[E_AXIS]+= FILAMENTCHANGE_FINALRETRACT ;
          #endif
        }
        
        plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder);
        
        //finish moves
        st_synchronize();
        //disable extruder steppers so filament can be removed
        disable_e0();
        disable_e1();
        disable_e2();
        delay(100);
        LCD_ALERTMESSAGEPGM(MSG_FILAMENTCHANGE);
        uint8_t cnt=0;
        while(!LCD_CLICKED){
          cnt++;
          manage_heater();
          manage_inactivity();
          lcd_update();
          
          #if BEEPER > -1
          if(cnt==0)
          {
            SET_OUTPUT(BEEPER);
            
            WRITE(BEEPER,HIGH);
            delay(3);
            WRITE(BEEPER,LOW);
            delay(3);
          }
          #endif
        }
        
        //return to normal
        if(code_seen('L')) 
        {
          target[E_AXIS]+= -code_value();
        }
        else
        {
          #ifdef FILAMENTCHANGE_FINALRETRACT
            target[E_AXIS]+=(-1)*FILAMENTCHANGE_FINALRETRACT ;
          #endif
        }
        current_position[E_AXIS]=target[E_AXIS]; //the long retract of L is compensated by manual filament feeding
        plan_set_e_position(current_position[E_AXIS]);
        plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder); //should do nothing
        plan_buffer_line(lastpos[X_AXIS], lastpos[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder); //move xy back
        plan_buffer_line(lastpos[X_AXIS], lastpos[Y_AXIS], lastpos[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder); //move z back
        plan_buffer_line(lastpos[X_AXIS], lastpos[Y_AXIS], lastpos[Z_AXIS], lastpos[E_AXIS], feedrate/60, active_extruder); //final untretract
    }
    break;
    #endif //FILAMENTCHANGEENABLE    
    #ifdef ENABLE_DIGITAL_POT_CONTROL
    case 907: // M907 Set digital trimpot motor current using axis codes.
    {
        for(int i=0;i<=NUM_AXIS;i++) if(code_seen(axis_codes[i])) digipot_current(i,code_value());
        if(code_seen('B')) digipot_current(4,code_value());
        if(code_seen('S')) for(int i=0;i<=4;i++) digipot_current(i,code_value());
    }
    case 908: // M908 Control digital trimpot directly.
    {
        uint8_t channel,current;
        if(code_seen('P')) channel=code_value();
        if(code_seen('S')) current=code_value();
        digitalPotWrite(channel, current);
    }
    break;
    #endif // ENABLE_DIGITAL_POT_CONTROL
    case 999: // M999: Restart after being stopped
      Stopped = false;
      lcd_reset_alert_level();
      gcode_LastN = Stopped_gcode_LastN;
      FlushSerialRequestResend();
    break;
    }
  }

  else if(code_seen('T')) 
  {
    // By default E axis is set to the coordinate of the selected extruder
    uint8_t start_from_extruder = tmp_extruder = code_value();
    if(code_seen('S')) {
      start_from_extruder = code_value();
    }
    if(tmp_extruder >= EXTRUDERS) {
      SERIAL_ECHO_START;
      SERIAL_ECHO("T");
      SERIAL_ECHO(((int)tmp_extruder));
      SERIAL_ECHOLN(" " MSG_INVALID_EXTRUDER);
    }
    else if(start_from_extruder >= EXTRUDERS) {
      SERIAL_ECHO_START;
      SERIAL_ECHO("S");
      SERIAL_ECHO(((int)start_from_extruder));
      SERIAL_ECHOLN(" " MSG_INVALID_EXTRUDER);
    }
    else {
      boolean make_move = false;
      if(code_seen('F')) {
        make_move = true;
        next_feedrate = code_value();
        if(next_feedrate > 0.0) {
          feedrate = next_feedrate;
        }
      }
      #if EXTRUDERS > 1
      set_active_extruder(tmp_extruder, start_from_extruder, make_move);
      #endif
      SERIAL_ECHO_START;
      SERIAL_ECHO(MSG_ACTIVE_EXTRUDER);
      SERIAL_PROTOCOLLN(((int)active_extruder));
    }
  }

  else // Command letter unrecognized
  {
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM(MSG_UNKNOWN_COMMAND);
    SERIAL_ECHO(cmdbuffer[bufindr]);
    SERIAL_ECHOLNPGM("\"");
  }

  ClearToSend();
}

void FlushSerialRequestResend()
{
  //char cmdbuffer[bufindr][100]="Resend:";
  MYSERIAL.flush();
  SERIAL_PROTOCOLPGM(MSG_RESEND);
  SERIAL_PROTOCOLLN(gcode_LastN + 1);
  recovery_count = buflen + 1; // Give it a chance to grind through stuff received after the error
  ClearToSend();
}

void ClearToSend()
{
  previous_millis_cmd = millis();
  #ifdef SDSUPPORT
  if(fromsd[bufindr])
    return;
  #endif //SDSUPPORT
  SERIAL_PROTOCOLLNPGM(MSG_OK); 
}

void get_coordinates()
{
  bool seen[4]={false,false,false,false};
  for(int8_t i=0; i < NUM_AXIS; i++) {
    if(code_seen(axis_codes[i])) 
    {
      destination[i] = (float)code_value() + (axis_relative_modes[i] || relative_mode)*current_position[i];
      seen[i]=true;
    }
    else destination[i] = current_position[i]; //Are these else lines really needed?
  }
  if(code_seen('F')) {
    next_feedrate = code_value();
    if(next_feedrate > 0.0) feedrate = next_feedrate;
  }
  #ifdef FWRETRACT
  if(autoretract_enabled)
  if( !(seen[X_AXIS] || seen[Y_AXIS] || seen[Z_AXIS]) && seen[E_AXIS])
  {
    float echange=destination[E_AXIS]-current_position[E_AXIS];
    if(echange<-MIN_RETRACT) //retract
    {
      if(!retracted) 
      {
      
      destination[Z_AXIS]+=retract_zlift; //not sure why chaninging current_position negatively does not work.
      //if slicer retracted by echange=-1mm and you want to retract 3mm, corrrectede=-2mm additionally
      float correctede=-echange-retract_length;
      //to generate the additional steps, not the destination is changed, but inversely the current position
      current_position[E_AXIS]+=-correctede; 
      feedrate=retract_feedrate;
      retracted=true;
      }
      
    }
    else 
      if(echange>MIN_RETRACT) //retract_recover
    {
      if(retracted) 
      {
      //current_position[Z_AXIS]+=-retract_zlift;
      //if slicer retracted_recovered by echange=+1mm and you want to retract_recover 3mm, corrrectede=2mm additionally
      float correctede=-echange+1*retract_length+retract_recover_length; //total unretract=retract_length+retract_recover_length[surplus]
      current_position[E_AXIS]+=correctede; //to generate the additional steps, not the destination is changed, but inversely the current position
      feedrate=retract_recover_feedrate;
      retracted=false;
      }
    }
    
  }
  #endif //FWRETRACT
}

void get_arc_coordinates()
{
#ifdef SF_ARC_FIX
   bool relative_mode_backup = relative_mode;
   relative_mode = true;
#endif
   get_coordinates();
#ifdef SF_ARC_FIX
   relative_mode=relative_mode_backup;
#endif

   if(code_seen('I')) {
     offset[0] = code_value();
   } 
   else {
     offset[0] = 0.0;
   }
   if(code_seen('J')) {
     offset[1] = code_value();
   }
   else {
     offset[1] = 0.0;
   }
}

void clamp_to_software_endstops(float target[3])
{
  if (min_software_endstops) {
    if (target[X_AXIS] < X_MIN_POS) target[X_AXIS] = X_MIN_POS;
    if (target[Y_AXIS] < Y_MIN_POS) target[Y_AXIS] = Y_MIN_POS;
    if (target[Z_AXIS] < Z_MIN_POS) target[Z_AXIS] = Z_MIN_POS;
  }

  if (max_software_endstops) {
    if (target[X_AXIS] > X_MAX_POS) target[X_AXIS] = X_MAX_POS;
    if (target[Y_AXIS] > Y_MAX_POS) target[Y_AXIS] = Y_MAX_POS;
    if (target[Z_AXIS] > Z_MAX_POS) target[Z_AXIS] = Z_MAX_POS;
  }
}

void prepare_move()
{
  clamp_to_software_endstops(destination);

  previous_millis_cmd = millis(); 
  // Do not use feedmultiply for E or Z only moves
  if( (current_position[X_AXIS] == destination [X_AXIS]) && (current_position[Y_AXIS] == destination [Y_AXIS])) {
      plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
  }
  else {
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate*feedmultiply/60/100.0, active_extruder);
  }
  for(int8_t i=0; i < NUM_AXIS; i++) {
    current_position[i] = destination[i];
  }
}

void prepare_arc_move(char isclockwise) {
  float r = hypot(offset[X_AXIS], offset[Y_AXIS]); // Compute arc radius for mc_arc

  // Trace the arc
  mc_arc(current_position, destination, offset, X_AXIS, Y_AXIS, Z_AXIS, feedrate*feedmultiply/60/100.0, r, isclockwise, active_extruder);
  
  // As far as the parser is concerned, the position is now == target. In reality the
  // motion control system might still be processing the action and the real tool position
  // in any intermediate location.
  for(int8_t i=0; i < NUM_AXIS; i++) {
    current_position[i] = destination[i];
  }
  previous_millis_cmd = millis();
}

#ifdef CONTROLLERFAN_PIN
unsigned long lastMotor = 0; //Save the time for when a motor was turned on last
unsigned long lastMotorCheck = 0;

void controllerFan()
{
  if ((millis() - lastMotorCheck) >= 2500) //Not a time critical function, so we only check every 2500ms
  {
    lastMotorCheck = millis();
    
    if(!READ(X_ENABLE_PIN) || !READ(Y_ENABLE_PIN) || !READ(Z_ENABLE_PIN)
    #if EXTRUDERS > 2
       || !READ(E2_ENABLE_PIN)
    #endif
    #if EXTRUDER > 1
       || !READ(E2_ENABLE_PIN)
    #endif
       || !READ(E0_ENABLE_PIN)) //If any of the drivers are enabled...    
    {
      lastMotor = millis(); //... set time to NOW so the fan will turn on
    }
    
    if ((millis() - lastMotor) >= (CONTROLLERFAN_SEC*1000UL) || lastMotor == 0) //If the last time any driver was enabled, is longer since than CONTROLLERSEC...   
    {
      WRITE(CONTROLLERFAN_PIN, LOW); //... turn the fan off
    }
    else
    {
      WRITE(CONTROLLERFAN_PIN, HIGH); //... turn the fan on
    }
  }
}
#endif

void manage_inactivity() 
{ 
  if( (millis() - previous_millis_cmd) >  max_inactive_time ) 
    if(max_inactive_time) 
      kill(); 
  if(stepper_inactive_time)  {
    if( (millis() - previous_millis_cmd) >  stepper_inactive_time ) 
    {
      if(blocks_queued() == false) {
        st_synchronize();
        disable_x();
        disable_y();
        disable_z();
        disable_e0();
        disable_e1();
        disable_e2();
      }
    }
  }

  #if( KILL_PIN>-1 )
  if( 0 == READ(KILL_PIN) )
    kill();
  #endif
  #ifdef CONTROLLERFAN_PIN
  controllerFan(); //Check if fan should be turned on to cool stepper drivers down
  #endif

  #ifdef EXTRUDER_RUNOUT_PREVENT && (EXTRUDERS == 1) 
  if( (millis() - previous_millis_cmd) >  EXTRUDER_RUNOUT_SECONDS * 1000 ) 
    if(degHotend(0) > EXTRUDER_RUNOUT_MINTEMP)
    {
      bool oldstatus=READ(E0_ENABLE_PIN);
      enable_e0();
      float oldepos=current_position[E_AXIS];
      float oldedes=destination[E_AXIS];
      plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], 
                       current_position[E_AXIS]+EXTRUDER_RUNOUT_EXTRUDE*EXTRUDER_RUNOUT_ESTEPS/axis_steps_per_unit[E_AXIS], 
                       EXTRUDER_RUNOUT_SPEED/60.*EXTRUDER_RUNOUT_ESTEPS/axis_steps_per_unit[E_AXIS], active_extruder);
      current_position[E_AXIS]=oldepos;
      destination[E_AXIS]=oldedes;
      plan_set_e_position(oldepos);
      previous_millis_cmd=millis();
      st_synchronize();
      WRITE(E0_ENABLE_PIN,oldstatus);
    }
  #endif // EXTRUDER_RUNOUT_PREVENT && (EXTRUDERS == 1)
  
  check_axes_activity();
}

void kill()
{
  cli(); // Stop interrupts
  disable_heater();

  #ifndef DUAL_X_DRIVE
  disable_x();
  #else  // DUAL_X_DRIVE
  disable_x0();
  disable_x1();
  #endif // DUAL_X_DRIVE
  #ifndef DUAL_Y_DRIVE
  disable_y();
  #else  // DUAL_Y_DRIVE
  disable_y0();
  disable_y1();
  #endif // DUAL_Y_DRIVE
  disable_z();
  disable_e0();
  disable_e1();
  disable_e2();
  
  if(PS_ON_PIN > -1) pinMode(PS_ON_PIN,INPUT);
  SERIAL_ERROR_START;
  SERIAL_ERRORLNPGM(MSG_ERR_KILLED);
  LCD_ALERTMESSAGEPGM(MSG_KILLED);
  suicide();
  while(1) { /* Intentionally left empty */ } // Wait for reset
}

void Stop()
{
  disable_heater();
  if(Stopped == false) {
    Stopped = true;
    Stopped_gcode_LastN = gcode_LastN; // Save last g_code for restart
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM(MSG_ERR_STOPPED);
    LCD_MESSAGEPGM(MSG_STOPPED);
  }
}

bool IsStopped() { return Stopped; };

#ifdef FAST_PWM_FAN
void setPwmFrequency(uint8_t pin, int val)
{
  val &= 0x07;
  switch(digitalPinToTimer(pin))
  {
 
    #if defined(TCCR0A)
    case TIMER0A:
    case TIMER0B:
//         TCCR0B &= ~(_BV(CS00) | _BV(CS01) | _BV(CS02));
//         TCCR0B |= val;
         break;
    #endif

    #if defined(TCCR1A)
    case TIMER1A:
    case TIMER1B:
//         TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
//         TCCR1B |= val;
         break;
    #endif

    #if defined(TCCR2)
    case TIMER2:
    case TIMER2:
         TCCR2 &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
         TCCR2 |= val;
         break;
    #endif

    #if defined(TCCR2A)
    case TIMER2A:
    case TIMER2B:
         TCCR2B &= ~(_BV(CS20) | _BV(CS21) | _BV(CS22));
         TCCR2B |= val;
         break;
    #endif

    #if defined(TCCR3A)
    case TIMER3A:
    case TIMER3B:
    case TIMER3C:
         TCCR3B &= ~(_BV(CS30) | _BV(CS31) | _BV(CS32));
         TCCR3B |= val;
         break;
    #endif

    #if defined(TCCR4A) 
    case TIMER4A:
    case TIMER4B:
    case TIMER4C:
         TCCR4B &= ~(_BV(CS40) | _BV(CS41) | _BV(CS42));
         TCCR4B |= val;
         break;
    #endif

    #if defined(TCCR5A) 
    case TIMER5A:
    case TIMER5B:
    case TIMER5C:
         TCCR5B &= ~(_BV(CS50) | _BV(CS51) | _BV(CS52));
         TCCR5B |= val;
         break;
   #endif

  }
}
#endif //FAST_PWM_FAN

bool setTargetedHotend(int code){
  tmp_extruder = active_extruder;
  if(code_seen('T')) {
    tmp_extruder = code_value();
    if(tmp_extruder >= EXTRUDERS) {
      SERIAL_ECHO_START;
      SERIAL_ECHOPAIR("M", code);
      SERIAL_ECHO(" " MSG_INVALID_EXTRUDER " ");
      SERIAL_ECHOLN(tmp_extruder);
      return true;
    }
  }
  return false;
}
