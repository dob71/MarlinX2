#ifndef CONFIGURATION_ADV_H
#define CONFIGURATION_ADV_H

//===========================================================================
//=============================Thermal Settings  ============================
//===========================================================================

#define BED_HYSTERESIS 1 //only disable heating if T>target+BED_HYSTERESIS and enable heating if T>target-BED_HYSTERESIS
#define BED_CHECK_INTERVAL 5000 //ms between checks in bang-bang control

//// Heating sanity check:
// This waits for the watchperiod in milliseconds whenever an M104 or M109 increases the target temperature
// If the temperature has not increased at the end of that period, the target temperature is set to zero. 
// It can be reset with another M104/M109. This check is also only triggered if the target temperature and the current temperature
//  differ by at least 2x WATCH_TEMP_INCREASE
//#define WATCH_TEMP_PERIOD 40000 //40 seconds
//#define WATCH_TEMP_INCREASE 10  //Heat up at least 10 degree in 20 seconds

#ifdef PIDTEMP
  // this adds an experimental additional term to the heatingpower, proportional to the extrusion speed.
  // if Kc is choosen well, the additional required power due to increased melting should be compensated.
  #define PID_ADD_EXTRUSION_RATE  
  #ifdef PID_ADD_EXTRUSION_RATE
    #define  DEFAULT_Kc (1) //heatingpower=Kc*(e_speed)
  #endif
#endif

//automatic temperature: The hot end target temperature is calculated by all the buffered lines of gcode.
//The maximum buffered steps/sec of the extruder motor are called "se".
//You enter the autotemp mode by a M109 S<mintemp> T<maxtemp> F<factor>
// the target temperature is set to mintemp+factor*se[steps/sec] and limited by mintemp and maxtemp
// you exit the value by any M109 without F*
// Also, if the temperature is set to a value <mintemp, it is not changed by autotemp.
// on an ultimaker, some initial testing worked with M109 S215 B260 F1 in the start.gcode
#define AUTOTEMP
#ifdef AUTOTEMP
  #define AUTOTEMP_OLDWEIGHT 0.98
#endif

// Extruder run-out prevention. 
// If the machine is idle, and the temperature over MINTEMP, every couple of SECONDS some filament is extruded.
// Not supported for multiple extruder machines.
//#define EXTRUDER_RUNOUT_PREVENT  
//#define EXTRUDER_RUNOUT_MINTEMP 190  
//#define EXTRUDER_RUNOUT_SECONDS 30.
//#define EXTRUDER_RUNOUT_ESTEPS 14. //mm filament
//#define EXTRUDER_RUNOUT_SPEED 1500.  //extrusion speed
//#define EXTRUDER_RUNOUT_EXTRUDE 100

// These defines help to calibrate the AD595 sensor in case you get wrong temperature measurements.
// The measured temperature is defined as "actualTemp = (measuredTemp * TEMP_SENSOR_AD595_GAIN) + TEMP_SENSOR_AD595_OFFSET"
#define TEMP_SENSOR_AD595_OFFSET 0.0
#define TEMP_SENSOR_AD595_GAIN   1.0

// Enable this if in the follow-me mode you want the heaters of the 
// slave hotends use the temperature setting of the active one.
//#define FOLLOW_ME_HEATER

// This is for controlling a fan to cool down the stepper drivers
// it will turn on when any driver is enabled
// and turn off after the set amount of seconds from last driver being disabled again
//#define CONTROLLERFAN_PIN 23 //Pin used for the fan to cool controller, comment out to disable this function
#define CONTROLLERFAN_SEC 60 //How many seconds, after all motors were disabled, the fan should run

// When starting or increasing hotend fan speed, run it at full speed for the
// given number of milliseconds.  This starts the fan spinning before setting the PWM 
// value that otherwise won't be high enough to get it running.
// (Does not work with software PWM for fan on Sanguinololu)
#define FAN_KICKSTART_TIME 1000

//===========================================================================
//=============================Mechanical Settings===========================
//===========================================================================
#define ENDSTOPS_ONLY_FOR_HOMING // If defined the endstops will only be used for homing

// AUTOSET LOCATIONS OF LIMIT SWITCHES
#ifdef MANUAL_HOME_POSITIONS  // Use manual limit switch locations
  #define Z_HOME_POS MANUAL_Z_HOME_POS
  #ifdef DUAL_X_DRIVE
    #define X0_HOME_POS MANUAL_X0_HOME_POS
    #define X1_HOME_POS MANUAL_X1_HOME_POS
  #else
    #define X_HOME_POS MANUAL_X_HOME_POS
  #endif
  #ifdef DUAL_Y_DRIVE
    #define Y0_HOME_POS MANUAL_Y0_HOME_POS
    #define Y1_HOME_POS MANUAL_Y1_HOME_POS
  #else
    #define Y_HOME_POS MANUAL_Y_HOME_POS
  #endif
#else //Set min/max homing switch positions based upon homing direction and min/max travel limits
  //X axis
  #ifdef DUAL_X_DRIVE
    #if X0_HOME_DIR == -1
      #define X0_HOME_POS X0_MIN_POS
      #define X0_IGNORE_MAX_ENDSTOP true
      #define X0_IGNORE_MIN_ENDSTOP false
    #else    
      #define X0_HOME_POS X0_MAX_POS
      #define X0_IGNORE_MAX_ENDSTOP false
      #define X0_IGNORE_MIN_ENDSTOP true
    #endif //X0_HOME_DIR == -1
    #if X1_HOME_DIR == -1
      #define X1_HOME_POS X1_MIN_POS
      #define X1_IGNORE_MAX_ENDSTOP true
      #define X1_IGNORE_MIN_ENDSTOP false
    #else    
      #define X1_HOME_POS X1_MAX_POS
      #define X1_IGNORE_MAX_ENDSTOP false
      #define X1_IGNORE_MIN_ENDSTOP true
    #endif //X1_HOME_DIR == -1
  #else
    #if X_HOME_DIR == -1
      #define X_HOME_POS X_MIN_POS
    #else    
      #define X_HOME_POS X_MAX_POS
    #endif //X_HOME_DIR == -1
  #endif
  
  //Y axis
  #ifdef DUAL_Y_DRIVE
    #if Y0_HOME_DIR == -1
      #define Y0_HOME_POS Y0_MIN_POS
      #define Y0_IGNORE_MAX_ENDSTOP true
      #define Y0_IGNORE_MIN_ENDSTOP false
    #else    
      #define Y0_HOME_POS Y0_MAX_POS
      #define Y0_IGNORE_MAX_ENDSTOP false
      #define Y0_IGNORE_MIN_ENDSTOP true
    #endif //Y0_HOME_DIR == -1
    #if Y1_HOME_DIR == -1
      #define Y1_HOME_POS Y1_MIN_POS
      #define Y1_IGNORE_MAX_ENDSTOP true
      #define Y1_IGNORE_MIN_ENDSTOP false
    #else    
      #define Y1_HOME_POS Y1_MAX_POS
      #define Y1_IGNORE_MAX_ENDSTOP false
      #define Y1_IGNORE_MIN_ENDSTOP true
    #endif //Y1_HOME_DIR == -1
  #else
    #if Y_HOME_DIR == -1
      #define Y_HOME_POS Y_MIN_POS
    #else    
      #define Y_HOME_POS Y_MAX_POS
    #endif //Y_HOME_DIR == -1
  #endif
  
  // Z axis
  #if Z_HOME_DIR == -1
    #define Z_HOME_POS Z_MIN_POS
  #else    
    #define Z_HOME_POS Z_MAX_POS
  #endif //Z_HOME_DIR == -1
#endif //End auto min/max positions
//END AUTOSET LOCATIONS OF LIMIT SWITCHES -ZP

// If enabled M206 command can be used to add homeing offset
//#define ENABLE_ADD_HOMEING

//#define Z_LATE_ENABLE // Enable Z the last moment. Needed if your Z driver overheats.

// A single Z stepper driver is usually used to drive 2 stepper motors.
// Uncomment this define to utilize a separate stepper driver for each Z axis motor.
// Only a few motherboards support this, like RAMPS, which have dual extruder support (the 2nd, often unused, 
// extruder driver is used to control the 2nd Z axis stepper motor). The pins are currently only defined for 
// a RAMPS motherboards. On a RAMPS (or other 5 driver) motherboard, using this feature will limit you to 
// using 1 extruder. Make sure the pins are configured correctly and you are not trying to use the same pins 
// to drive the second extruder and second Z motor at the same time. 
//#define Z_DUAL_STEPPER_DRIVERS


// If this is defined, if both x and y are to be homed, a diagonal move will be performed initially.
#define QUICK_HOME

// Absolute or relative coordinate mode
#define AXIS_RELATIVE_MODES {false, false, false, false}

// Max step frequency for Ultimaker (5000 pps / half step)
#define MAX_STEP_FREQUENCY 40000

// By default pololu step drivers require an active high signal. However, some high power drivers require 
// an active low signal as step.
#define INVERT_X_STEP_PIN false
#define INVERT_Y_STEP_PIN false
#define INVERT_Z_STEP_PIN false
#define INVERT_E_STEP_PIN false

// default stepper release if idle
#define DEFAULT_STEPPER_DEACTIVE_TIME 600

// Minimum feedrate
#define DEFAULT_MINIMUMFEEDRATE       0.0
#define DEFAULT_MINTRAVELFEEDRATE     0.0

// Minimum time in microseconds that a movement needs to take if the buffer is emptied.
#define DEFAULT_MINSEGMENTTIME        20000

// If defined the movements slow down when the look ahead buffer is only half full
#define SLOWDOWN

// Frequency limit
// See nophead's blog for more info
// Not working O
//#define XY_FREQUENCY_LIMIT  15

// Minimum planner junction speed. Sets the default minimum speed the planner plans for at the end
// of the buffer and all stops. This should not be much greater than zero and should only be changed
// if unwanted behavior is observed on a user's machine when running at very slow speeds.
#define MINIMUM_PLANNER_SPEED 0.05 // (mm/sec)

// MS1 MS2 Stepper Driver Microstepping mode table
#define MICROSTEP1 LOW,LOW
#define MICROSTEP2 HIGH,LOW
#define MICROSTEP4 LOW,HIGH
#define MICROSTEP8 HIGH,HIGH
#define MICROSTEP16 HIGH,HIGH

// Microstep setting (Only functional when stepper driver microstep pins are connected to MCU.
#define MICROSTEP_MODES {16,16,16,16,16} // [1,2,4,8,16]

// Motor Current setting (Only functional when motor driver current ref pins are connected to a 
// digital trimpot on supported boards)
#define DIGIPOT_MOTOR_CURRENT {135,135,135,135,135} // Values 0-255 (RAMBO 135 = ~0.75A, 185 = ~1A)


//===========================================================================
//=============================Additional Features===========================
//===========================================================================

// If sd support and the file is finished: disable steppers?
#define SD_FINISHED_STEPPERRELEASE true  
// For some machines might want to keep the z enabled so your bed stays in place.
#define SD_FINISHED_RELEASECOMMAND "M84 X Y Z E"

// The hardware watchdog should reset the Microcontroller disabling all outputs, 
// in case the firmware gets stuck and doesn't do temperature regulation.
//#define USE_WATCHDOG

#ifdef USE_WATCHDOG
// If you have a watchdog reboot an ArduinoMega2560 then the device will hang forever, as a watchdog reset 
// will leave the watchdog on. The "WATCHDOG_RESET_MANUAL" goes around this by not using the hardware reset.
// However, THIS FEATURE IS UNSAFE!, as it will only work if interrupts are disabled. And the code could hang 
// in an interrupt routine with interrupts disabled.
//#define WATCHDOG_RESET_MANUAL
#endif

// Enable the option to stop SD printing when hitting and endstops, needs to be enabled from the 
// LCD menu when this option is enabled.
//#define ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED

// Arc interpretation settings:
#define MM_PER_ARC_SEGMENT 1
#define N_ARC_CORRECTION 25

// Defines the number of memory slots for saving/restoring position (M331/M332)
// The valuse should not be less than 1
#define NUM_POSITON_SLOTS 4

// Everything with less than this number of steps will be ignored as move and joined with the next movement
const unsigned int dropsegments = 5; 

// If you are using a RAMPS board or cheap E-bay purchased boards that do not detect when an SD card is inserted
// You can get round this by connecting a push button or single throw switch to the pin defined as SDCARDCARDDETECT 
// in the pins.h file.  When using a push button pulling the pin to ground this will need inverted.  This setting should
// be commented out otherwise. 
//#define SDCARDDETECTINVERTED 
//#ifdef ULTIPANEL
// #undef SDCARDDETECTINVERTED
//#endif

// Power Signal Control Definitions
// By default use ATX definition
#ifndef POWER_SUPPLY
  #define POWER_SUPPLY 1
#endif

// 1 = ATX
#if (POWER_SUPPLY == 1) 
  #define PS_ON_AWAKE  LOW
  #define PS_ON_ASLEEP HIGH
#endif

// 2 = X-Box 360 203W
#if (POWER_SUPPLY == 2) 
  #define PS_ON_AWAKE  HIGH
  #define PS_ON_ASLEEP LOW
#endif

// This is advanced filament compression compensation (C_COMPENSATION) parameter. 
// Uncomment the below define if expecting long slow moves with too many E-steps 
// per time slice due to the compensation. The define enables code splitting those 
// steps into chunks of 4 or less.
#define C_COMPENSATION_SPLIT_E_STEPS

//===========================================================================
//=============================Buffers           ============================
//===========================================================================

// The number of linear motions that can be in the plan at any give time.  
// THE BLOCK_BUFFER_SIZE NEEDS TO BE A POWER OF 2, i.g. 8,16,32 because shifts 
// and ors are used to do the ringbuffering.
#if defined SDSUPPORT
  #define BLOCK_BUFFER_SIZE 16   // SD,LCD,Buttons take more memory, block buffer needs to be smaller
#else
  #define BLOCK_BUFFER_SIZE 16 // maximize block buffer
#endif


// The ASCII buffer for receiving from the serial:
#define MAX_CMD_SIZE 96
#define BUFSIZE 4


// Firmware based and LCD controled retract
// M207 and M208 can be used to define parameters for the retraction. 
// The retraction can be called by the slicer using G10 and G11
// until then, intended retractions can be detected by moves that only extrude and the direction. 
// the moves are than replaced by the firmware controlled ones.
// ONLY PARTIALLY TESTED
// #define FWRETRACT 

// Minimum extruded mm to accept an automatic gcode retraction attempt
// #define MIN_RETRACT 0.1 

// Uncomment to enable M350/M351 microstepping control commands.
// The board pins for microstepping control should be defined if enabled
// (see code for the commands).
// #define ENABLE_MICROSTEPPING_CONTROL

// Uncomment to enable M907/M908 digital motor current control.
// The board pins for the  digital potentiometer should be defined if enabled
// (see code for the commands).
// #define ENABLE_DIGITAL_POT_CONTROL

//adds support for experimental filament exchange support M600; requires display
#ifdef ULTIPANEL
  //#define FILAMENTCHANGEENABLE
  #ifdef FILAMENTCHANGEENABLE
    #define FILAMENTCHANGE_XPOS 3
    #define FILAMENTCHANGE_YPOS 3
    #define FILAMENTCHANGE_ZADD 10
    #define FILAMENTCHANGE_FIRSTRETRACT -2
    #define FILAMENTCHANGE_FINALRETRACT -100
  #endif
#endif


//===========================================================================
//===============================  Debugging  ===============================
//===========================================================================

// Uncomment to enable debugging code and M504 (set debug flugs) 
// command. Look up the available flags in Marlin.h file.
#define ENABLE_DEBUG

// Uncomment if you see communication errors especially when 
// a lot of output comes from printer. This define will disable 
// printing of all the "echo" (i.e. human readable) output from the 
// machine while it is printing (detected by checking if multiple 
// moves are queued). There also is a debug flag (M504 S1) that can 
// be used to re-enable the output during printing without re-compilation.
#define NO_ECHO_WHILE_PRINTING

//===========================================================================
//=============================  Define Defines  ============================
//===========================================================================

#if TEMP_SENSOR_0 > 0
  #define THERMISTORHEATER_0 TEMP_SENSOR_0
  #define HEATER_0_USES_THERMISTOR
#endif
#if TEMP_SENSOR_1 > 0
  #define THERMISTORHEATER_1 TEMP_SENSOR_1
  #define HEATER_1_USES_THERMISTOR
#endif
#if TEMP_SENSOR_2 > 0
  #define THERMISTORHEATER_2 TEMP_SENSOR_2
  #define HEATER_2_USES_THERMISTOR
#endif
#if TEMP_SENSOR_BED > 0
  #define THERMISTORBED TEMP_SENSOR_BED
  #define BED_USES_THERMISTOR
#endif
#if TEMP_SENSOR_0 == -1
  #define HEATER_0_USES_AD595
#endif
#if TEMP_SENSOR_1 == -1
  #define HEATER_1_USES_AD595
#endif
#if TEMP_SENSOR_2 == -1
  #define HEATER_2_USES_AD595
#endif
#if TEMP_SENSOR_BED == -1
  #define BED_USES_AD595
#endif
#if TEMP_SENSOR_0 == -2
  #define HEATER_0_USES_MAX6675
#endif
#if TEMP_SENSOR_0 == 0
  #undef HEATER_0_MINTEMP
  #undef HEATER_0_MAXTEMP
#endif
#if TEMP_SENSOR_1 == 0
  #undef HEATER_1_MINTEMP
  #undef HEATER_1_MAXTEMP
#endif
#if TEMP_SENSOR_2 == 0
  #undef HEATER_2_MINTEMP
  #undef HEATER_2_MAXTEMP
#endif
#if TEMP_SENSOR_BED == 0
  #undef BED_MINTEMP
  #undef BED_MAXTEMP
#endif

#ifdef DUAL_X_DRIVE
  #define X_MAX_POS    gXMaxPos[ACTIVE_EXTRUDER]
  #define X_MIN_POS    gXMinPos[ACTIVE_EXTRUDER]
#endif // DUAL_X_DRIVE
  
#ifdef DUAL_Y_DRIVE
  #define Y_MAX_POS    gYMaxPos[ACTIVE_EXTRUDER]
  #define Y_MIN_POS    gYMinPos[ACTIVE_EXTRUDER]
#endif // DUAL_Y_DRIVE

#if (defined(DUAL_Y_DRIVE) || defined(DUAL_X_DRIVE)) && EXTRUDERS > 2
  #error The dual drive configuration with more than 2 extruders is not supported.
#endif

#ifdef PER_EXTRUDER_FANS
  #ifdef FAN_SOFT_PWM
  #  error The FAN_SOFT_PWM feature is not compatible with PER_EXTRUDER_FANS
  #endif
  #if !(EXTRUDERS > 1)
  #  error The FAN_SOFT_PWM feature requires more than 1 extruder
  #endif
  #define FAN_PIN (fan_pin[ACTIVE_EXTRUDER])
#endif // PER_EXTRUDER_FANS

#endif //__CONFIGURATION_ADV_H
