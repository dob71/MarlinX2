#ifndef CONFIGURATION_H
#define CONFIGURATION_H

// This configurtion file contains the basic settings.
// Advanced settings can be found in Configuration_adv.h 
// BASIC SETTINGS: select your board type, temperature sensor type, axis scaling, and endstop configuration

//User specified version info of this build to display in [Pronterface, etc] terminal window during startup.
//Implementation of an idea by Prof Braino to inform user that any changes made
//to this build by the user have been successfully uploaded into firmware.
#define STRING_VERSION_CONFIG_H __DATE__ " " __TIME__ // build date and time
#define STRING_CONFIG_H_AUTHOR "(none, default config)" //Who made the changes.

// SERIAL_PORT selects which serial port should be used for communication with the host.
// This allows the connection of wireless adapters (for instance) to non-default port pins.
// Serial port 0 is still used by the Arduino bootloader regardless of this setting.
#define SERIAL_PORT 0

// This determines the communication speed of the printer
//#define BAUDRATE 250000
#define BAUDRATE 115200

//// The following define selects which electronics board you have. Please choose the one that matches your setup
// 10 = Gen7 custom (Alfons3 Version) "https://github.com/Alfons3/Generation_7_Electronics"
// 11 = Gen7 v1.1, v1.2 = 11
// 12 = Gen7 v1.3
// 13 = Gen7 v1.4
// 3  = MEGA/RAMPS up to 1.2 = 3
// 33 = RAMPS 1.3 / 1.4 (Power outputs: Extruder, Bed, Fan)
// 34 = RAMPS 1.3 / 1.4 (Power outputs: Extruder0, Extruder1, Bed)
// 4  = Duemilanove w/ ATMega328P pin assignment
// 5  = Gen6
// 51 = Gen6 deluxe
// 6  = Sanguinololu < 1.2
// 62 = Sanguinololu 1.2 and above
// 63 = Melzi
// 64 = STB V1.1
// 7  = Ultimaker
// 71 = Ultimaker (Older electronics. Pre 1.5.4. This is rare)
// 8  = Teensylu
// 80 = Rumba
// 81 = Printrboard (AT90USB1286)
// 82 = Brainwave (AT90USB646)
// 9  = Gen3+
// 70 = Megatronics
// 90 = Alpha OMCA board
// 91 = Final OMCA board
// 301 = Rambo

#ifndef MOTHERBOARD
#define MOTHERBOARD 34
#define REPRAPX2 // SUBFLAVOUR
#endif

//// The following define selects which power supply you have. Please choose the one that matches your setup
// 1 = ATX
// 2 = X-Box 360 203Watts (the blue wire connected to PS_ON and the red wire to VCC)

#define POWER_SUPPLY 1

// This defines the number of extruders
#define EXTRUDERS 2

//===========================================================================
//=============================Thermal Settings  ============================
//===========================================================================
//
//--NORMAL IS 4.7kohm PULLUP!-- 1kohm pullup can be used on hotend sensor, using correct resistor and table
//
//// Temperature sensor settings:
// -2 is thermocouple with MAX6675 (only for sensor 0)
// -1 is thermocouple with AD595
// 0 is not used
// 1 is 100k thermistor - best choice for EPCOS 100k (4.7k pullup)
// 2 is 200k thermistor - ATC Semitec 204GT-2 (4.7k pullup)
// 3 is mendel-parts thermistor (4.7k pullup)
// 4 is 10k thermistor !! do not use it for a hotend. It gives bad resolution at high temp. !!
// 5 is 100K thermistor - ATC Semitec 104GT-2 (Used in ParCan) (4.7k pullup)
// 6 is 100k EPCOS - Not as accurate as table 1 (created using a fluke thermocouple) (4.7k pullup)
// 7 is 100k Honeywell thermistor 135-104LAG-J01 (4.7k pullup)
// 8 is 100k 0603 SMD Vishay NTCS0603E3104FXT (4.7k pullup)
// 9 is 100k GE Sensing AL03006-58.2K-97-G1 (4.7k pullup)
// 10 is 100k RS thermistor 198-961 (4.7k pullup)
//
//    1k ohm pullup tables - This is not normal, you would have to have changed out your 4.7k for 1k 
//                          (but gives greater accuracy and more stable PID)
// 51 is 100k thermistor - EPCOS (1k pullup)
// 52 is 200k thermistor - ATC Semitec 204GT-2 (1k pullup)
// 55 is 100k thermistor - ATC Semitec 104GT-2 (Used in ParCan) (1k pullup)

#define TEMP_SENSOR_0 1
#define TEMP_SENSOR_1 1
#define TEMP_SENSOR_2 0
#define TEMP_SENSOR_BED 1

// Actual temperature must be close to target for this long before M109 returns success
#define TEMP_RESIDENCY_TIME 30  // (seconds)
#define TEMP_WINDOW 2           // (C) range of +/- temperatures considered "close" enough to the target one 
                                // (i.e. M109 will consider the temp reached if withing that range)
#define TEMP_WINDOW_BED 2       // (C) range of +/- temperatures considered "close" enough for bed

// The minimal temperature defines the temperature below which the heater will not be enabled It is used
// to check that the wiring to the thermistor is not broken. 
// Otherwise this would lead to the heater being powered on all the time.
#define HEATER_0_MINTEMP 5
#define HEATER_1_MINTEMP 5
#define HEATER_2_MINTEMP 5
#define BED_MINTEMP 5

// When temperature exceeds max temp, your heater will be switched off.
// This feature exists to protect your hotend from overheating accidentally, but *NOT* from thermistor short/failure!
// You should use MINTEMP for thermistor short/failure protection.
#define HEATER_0_MAXTEMP 275
#define HEATER_1_MAXTEMP 275
#define HEATER_2_MAXTEMP 275
#define BED_MAXTEMP 150

// PID settings:
// Comment the following line to disable PID and enable bang-bang.
#define PIDTEMP
#define PID_MAX 256 // limits current to nozzle while PID is active (see PID_FUNCTIONAL_RANGE below); 256=full current
#ifdef PIDTEMP
  //#define PID_OPENLOOP 1 // Puts PID in open loop. M104/M140 sets the output power from 0 to PID_MAX
  #define PID_INTEGRAL_DRIVE_MAX 255  //limit for the integral term
  #define K1 0.95 //smoothing factor withing the PID
  #define PID_dT 0.1 //sampling period of the PID

  // If PID_FUNCTIONAL_RANGE <deg> is defined the PID is used only within the specified 
  // <deg> range of the target temperature, outside of the range on/off method 
  // is used. The accumulated integral part is not affected.
  #define PID_FUNCTIONAL_RANGE 4.0

  //To develop some PID settings for your machine, you can initiall follow 
  // the Ziegler-Nichols method.
  // set Ki and Kd to zero. 
  // heat with a defined Kp and see if the temperature stabilizes
  // ideally you do this graphically with repg.
  // the PID_CRITIAL_GAIN should be the Kp at which temperature oscillatins are not dampned out/decreas in amplitutde
  // PID_SWING_AT_CRITIAL is the time for a full period of the oscillations at the critical Gain
  // usually further manual tunine is necessary.

  #define PID_CRITIAL_GAIN 50
  #define PID_SWING_AT_CRITIAL 47 //seconds
  
  //#define PID_PI    //no differentail term
  #define PID_PID //normal PID

  #ifdef PID_PID
    //PID according to Ziegler-Nichols method
    //    #define  DEFAULT_Kp  (0.6*PID_CRITIAL_GAIN)
    //    #define  DEFAULT_Ki (2*Kp/PID_SWING_AT_CRITIAL*PID_dT)  
    //    #define  DEFAULT_Kd (PID_SWING_AT_CRITIAL/8./PID_dT)  

    // If you are using a preconfigured hotend then you can use one of the value sets by uncommenting it
    // Ultimaker
    //    #define  DEFAULT_Kp 22.2
    //    #define  DEFAULT_Ki 1.08  
    //    #define  DEFAULT_Kd 114  

    // Makergear
    //    #define  DEFAULT_Kp 7.0
    //    #define  DEFAULT_Ki 0.1  
    //    #define  DEFAULT_Kd 12  
    #define  DEFAULT_Kp 70.0
    #define  DEFAULT_Ki 5.0  
    #define  DEFAULT_Kd 120  

    // Mendel Parts V9 on 12V    
    //    #define  DEFAULT_Kp 63.0
    //    #define  DEFAULT_Ki 2.25
    //    #define  DEFAULT_Kd 440
  #endif
   
  #ifdef PID_PI
    //PI according to Ziegler-Nichols method
    #define  DEFAULT_Kp (PID_CRITIAL_GAIN/2.2) 
    #define  DEFAULT_Ki (1.2*Kp/PID_SWING_AT_CRITIAL*PID_dT)
    #define  DEFAULT_Kd (0)
  #endif

#else // PIDTEMP
  // limits current to nozzle; 255=full current (used even if PID is Off)
  #define PID_MAX 255
#endif // PIDTEMP

// Bed Temperature Control
// Select PID bed temperature control by defining PIDTEMPBED. 
// If not defined traditional on/off method is used and BED_HYSTERESIS in advanced sets the hysteresis loop range.
//
// uncomment this to enable PID on the bed. It uses the same ferquency PWM as the extruder. 
// If your PID_dT above is the default, and correct for your hardware/configuration, that means 7.689Hz,
// which is fine for driving a square wave into a resistive load and does not significantly impact you FET heating.
// This also works fine on a Fotek SSR-10DA Solid State Relay into a 250W heater. 
// If your configuration is significantly different than this and you don't understand the issues involved, you proabaly 
// shouldn't use bed PID until someone else verifies your hardware works.
// If this is enabled, find your own PID constants below.
//#define PIDTEMPBED

// This sets the max power delived to the bed, and replaces the HEATER_BED_DUTY_CYCLE_DIVIDER option.
// all forms of bed control obey this (PID and bang-bang)
// setting this to anything other than 256 enables a form of PWM to the bed just like HEATER_BED_DUTY_CYCLE_DIVIDER did,
// so you shouldn't use it unless you are OK with PWM on your bed.  (see the comment on enabling PIDTEMPBED)
#define MAX_BED_POWER 256 // limits duty cycle to bed; 256=full current

#ifdef PIDTEMPBED
//120v 250W silicone heater into 4mm borosilicate (MendelMax 1.5+)
//from FOPDT model - kp=.39 Tp=405 Tdead=66, Tc set to 79.2, argressive factor of .15 (vs .1, 1, 10)
    #define  DEFAULT_bedKp 10.00
    #define  DEFAULT_bedKi .023
    #define  DEFAULT_bedKd 305.4

//120v 250W silicone heater into 4mm borosilicate (MendelMax 1.5+)
//from pidautotune
//    #define  DEFAULT_bedKp 97.1
//    #define  DEFAULT_bedKi 1.41
//    #define  DEFAULT_bedKd 1675.16

// FIND YOUR OWN: "M303 E-1 C8 S90" to run autotune on the bed at 90 degreesC for 8 cycles.
#endif // PIDTEMPBED



// PREVENT_DANGEROUS_EXTRUDE prevents dangerous Extruder moves.
// If it is defined, then you can define EXTRUDE_MINTEMP to limit the minimum allowed extrusion temperature 
// and/or EXTRUDE_MAXLENGTH to limit the maximum allowed filament extrusion length for any single move. 
#define PREVENT_DANGEROUS_EXTRUDE
#define EXTRUDE_MINTEMP 170 // in deg
#define EXTRUDE_MAXLENGTH 100 //in mm

//===========================================================================
//=============================Mechanical Settings===========================
//===========================================================================

// Uncomment the following line to enable CoreXY kinematics
//#define COREXY

// corse Endstop Settings
// The pullups are needed if you directly connect a mechanical endswitch between the signal and ground pins.
#define ENDSTOPPULLUPS // Comment this out (using // at the start of the line) to disable the endstop pullup resistors

#ifdef ENDSTOPPULLUPS
  // Enstop settings: Individual Pullups. will be ignord if ENDSTOPPULLUPS is defined
  #define ENDSTOPPULLUP_XMAX
  #define ENDSTOPPULLUP_YMAX
  #define ENDSTOPPULLUP_ZMAX
  #define ENDSTOPPULLUP_XMIN
  #define ENDSTOPPULLUP_YMIN
  #define ENDSTOPPULLUP_ZMIN
#endif

const bool X_ENDSTOPS_INVERTING = false; // set to true to invert the logic of the endstops. 
const bool Y_ENDSTOPS_INVERTING = false; // set to true to invert the logic of the endstops. 
const bool Z_ENDSTOPS_INVERTING = false; // set to true to invert the logic of the endstops. 

// If uncommented pins for MAX enstops are undefined
//#define DISABLE_MAX_ENDSTOPS

// If the machine can independently move the second extruder on the X or Y axis
// uncomment the appropriate define and uncomment/use paramters with X0/X1/Y0/Y1 
// instead of X/Y in their name (those are dual drive specific).
#define DUAL_X_DRIVE
//#define DUAL_Y_DRIVE

// For Inverting Stepper Enable Pins (Active Low) use 0, Non Inverting (Active High) use 1
#define X_ENABLE_ON 0 // For all X drives
#define Y_ENABLE_ON 0 // For all Y drives
#define Z_ENABLE_ON 0
#define E_ENABLE_ON 0 // For all extruders

// Disables axis when it's not being used.
#define DISABLE_X false // For all X drives
#define DISABLE_Y false // For all Y drives
#define DISABLE_Z false
#define DISABLE_E false // For all extruders

// Changes the direction of the movement on an axis.
// For dual drive on an axis use numbered variations of the define and comment out the unnumberd.
//#define INVERT_X_DIR false     // X axis motor direction, for Mendel set to false, for Orca set to true
#define INVERT_X0_DIR false  // X axis first motor direction, for Mendel set to false, for Orca set to true
#define INVERT_X1_DIR true   // X axis second motor direction
#define INVERT_Y_DIR false     // Y axis motor direction, for Mendel set to true, for Orca set to false
//#define INVERT_Y0_DIR false  // Y first motor direction, for Mendel set to true, for Orca set to false
//#define INVERT_Y1_DIR true   // Y second motor direction
#define INVERT_Z_DIR true    // for Mendel set to false, for Orca set to true
#define INVERT_E0_DIR false  // for direct drive extruder v9 set to true, for geared extruder set to false
#define INVERT_E1_DIR true   // for direct drive extruder v9 set to true, for geared extruder set to false
#define INVERT_E2_DIR false  // for direct drive extruder v9 set to true, for geared extruder set to false

// ENDSTOP SETTINGS:
// Sets direction of endstops when homing; 1=MAX, -1=MIN
// For dual drive on an axis use numbered variations of the define and comment out the unnumberd.
//#define X_HOME_DIR  -1  // X axis homing direction
#define X0_HOME_DIR -1  // first motor X axis homing direction
#define X1_HOME_DIR  1  // second motor X axis homing direction
#define Y_HOME_DIR -1   // Y axis homing direction
//#define Y0_HOME_DIR -1  // first motor Y axis homing direction
//#define Y1_HOME_DIR  1  // second motor Y axis homing direction
#define Z_HOME_DIR -1

#define min_software_endstops true  // If true, axis won't move to coordinates less than HOME_POS.
#define max_software_endstops true  // If true, axis won't move to coordinates greater than the defined lengths below.

// Travel limits after homing (use defines for dual drives if enabled for axes)
// For dual drive on an axis use numbered variations of the define and comment out the unnumbered.
//#define X_MAX_POS 250
#define X0_MAX_POS 207.45
#define X1_MAX_POS 252.45
//#define X_MIN_POS 0
#define X0_MIN_POS -35
#define X1_MIN_POS 10
#define Y_MAX_POS 198
//#define Y0_MAX_POS 200
//#define Y1_MAX_POS 200
#define Y_MIN_POS 0
//#define Y0_MIN_POS 0
//#define Y1_MIN_POS 0
#define Z_MAX_POS 135
#define Z_MIN_POS 0

// What's the max move distance along an axis? For dual drievs use the greatest.
//#define X_MAX_LENGTH (X_MAX_POS - X_MIN_POS) 
#define X_MAX_LENGTH (X1_MAX_POS - X0_MIN_POS) 
// #define X_MAX_LENGTH (X0_MAX_POS - X1_MIN_POS) 
#define Y_MAX_LENGTH (Y_MAX_POS - Y_MIN_POS)
// #define Y_MAX_LENGTH (Y1_MAX_POS - Y0_MIN_POS)
// #define Y_MAX_LENGTH (Y0_MAX_POS - Y1_MIN_POS)
#define Z_MAX_LENGTH (Z_MAX_POS - Z_MIN_POS)

//The BED_CENTER_AT_0_0 is not supported anymore, use *_MIN_POS and *_MAX_POS instead

// The position of the homing switches
//#define MANUAL_HOME_POSITIONS  // If defined, MANUAL_*_HOME_POS below will be used
//Manual homing switch locations:
//#define MANUAL_X_HOME_POS 0
//#define MANUAL_X0_HOME_POS 0
//#define MANUAL_X1_HOME_POS 290
//#define MANUAL_Y_HOME_POS 0
//#define MANUAL_Y0_HOME_POS 0
//#define MANUAL_Y1_HOME_POS 200
//#define MANUAL_Z_HOME_POS 0

// MOVEMENT SETTINGS
#define NUM_AXIS 4 // The axis order in all axis related arrays is X, Y, Z, E*
#define HOMING_FEEDRATE {30*60, 30*60, 2*60, 0}  // set the homing speeds (mm/min)

// Homing hits the endstop, then retracts by this distance, before it tries to slowly bump again.
// For dual drives on an axis the same value is used for both drives.
#define X_HOME_RETRACT_MM 5 
#define Y_HOME_RETRACT_MM 5 
#define Z_HOME_RETRACT_MM 1 

// Offset of the extruders (uncomment if using more than one and relying on firmware to position when changing).
// The offset has to be X=0, Y=0 for the extruder 0 hotend (default extruder).
// For the other hotends it is their distance from the extruder 0 hotend.
// Note: the extruder offset for multiple extruder machines with dual drive on an axis should be set to 0 
//       for that axis. On that axis the positioning is controlled by the the coordinates of the homing 
//       switches (i.e. *_MAX_POS/*_MIN_POS or MANUAL_*_HOME_POS if MANUAL_HOME_POSITIONS if defined).
#define EXTRUDER_OFFSET_X {0.0, 0.0} // (in mm) per extruder, offset of the extruder on the X axis
#define EXTRUDER_OFFSET_Y {0.0, -0.05} // (in mm) per extruder, offset of the extruder on the Y axis

// default settings 
#define DEFAULT_AXIS_STEPS_PER_UNIT   {80.0000, 80.0000, 2284.7651, 661.78, 661.78} // X,Y,Z,E0... SAE Prusa w/ Wade extruder
#define DEFAULT_MAX_FEEDRATE          {230, 230, 7, 23, 23} // X,Y,Z,E0...(mm/sec)    
#define DEFAULT_MAX_ACCELERATION      {2000, 1500, 100, 5000, 5000} // X,Y,Z,E0... maximum acceleration (mm/s^2). E default values are good for skeinforge 40+, for older versions raise them a lot.
#define DEFAULT_RETRACT_ACCELERATION  {60000, 60000} // E0... (per extruder) acceleration in mm/s^2 for retracts 
#define DEFAULT_ACCELERATION          2500    // X,Y,Z and E* acceleration (one for all) in mm/s^2 for printing moves 

#define DEFAULT_XYJERK                2.0     // (mm/sec)
#define DEFAULT_ZJERK                 0.4     // (mm/sec)
#define DEFAULT_EJERK                 {17, 17}    // E0... (mm/sec) per extruder, max initial speed for retract moves

//===========================================================================
//=============================Additional Features===========================
//===========================================================================

// EEPROM
// the microcontroller can store settings in the EEPROM, e.g. max velocity...
// M500 - stores paramters in EEPROM
// M501 - reads parameters from EEPROM (if you need reset them after you changed them temporarily).  
// M502 - reverts to the default "factory settings".  You still need to store them in EEPROM afterwards if you want to.
//define this to enable eeprom support
#define EEPROM_SETTINGS

//to disable EEPROM Serial responses and decrease program space by ~1700 byte: comment this out:
// please keep turned on if you can.
#define EEPROM_CHITCHAT

//LCD and SD support
//#define ULTRA_LCD  //general lcd support, also 16x2
//#define DOGLCD	   // Support for SPI LCD 128x64 (Controller ST7565R graphic Display Family)
//#define SDSUPPORT  // Enable SD Card Support in Hardware Console

//#define ULTIMAKERCONTROLLER //as available from the ultimaker online store.
//#define ULTIPANEL  //the ultipanel as on thingiverse

// The RepRapDiscount Smart Controller (white PCB)
// http://reprap.org/wiki/RepRapDiscount_Smart_Controller
//#define REPRAP_DISCOUNT_SMART_CONTROLLER

// The GADGETS3D G3D LCD/SD Controller (blue PCB)
// http://reprap.org/wiki/RAMPS_1.3/1.4_GADGETS3D_Shield_with_Panel
//#define G3D_PANEL

// The RepRapDiscount FULL GRAPHIC Smart Controller (quadratic white PCB)
// http://reprap.org/wiki/RepRapDiscount_Full_Graphic_Smart_Controller
//
// ==> REMEMBER TO INSTALL U8glib to your ARDUINO library folder: http://code.google.com/p/u8glib/wiki/u8glib
#define REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER

//automatic expansion
#if defined (REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
 #define DOGLCD
 #define U8GLIB_ST7920
 #define REPRAP_DISCOUNT_SMART_CONTROLLER
#endif

#if defined(ULTIMAKERCONTROLLER) || defined(REPRAP_DISCOUNT_SMART_CONTROLLER) || defined(G3D_PANEL)
 #define ULTIPANEL
 #define NEWPANEL
#endif 

// Preheat Constants
#define PLA_PREHEAT_HOTEND_TEMP 178 
#define PLA_PREHEAT_HPB_TEMP 80
#define PLA_PREHEAT_FAN_SPEED 128		// Insert Value between 0 and 255

#define ABS_PREHEAT_HOTEND_TEMP 220
#define ABS_PREHEAT_HPB_TEMP 110
#define ABS_PREHEAT_FAN_SPEED 70		   // Insert Value between 0 and 255

#ifdef ULTIPANEL
//  #define NEWPANEL  //enable this if you have a click-encoder panel
  #define SDSUPPORT
  #define ULTRA_LCD
	#ifdef DOGLCD	// Change number of lines to match the DOG graphic display
		#define LCD_WIDTH 20
		#define LCD_HEIGHT 5
	#else
		#define LCD_WIDTH 20
		#define LCD_HEIGHT 4
	#endif
#else //no panel but just lcd 
  #ifdef ULTRA_LCD
	#ifdef DOGLCD	// Change number of lines to match the 128x64 graphics display
		#define LCD_WIDTH 20
		#define LCD_HEIGHT 5
	#else
		#define LCD_WIDTH 16
		#define LCD_HEIGHT 2
	#endif    
  #endif
#endif

// Enable filament compression compensation (for bowden drives). If enabled the 
// firmware compensates for a few more mm of the filament compressed in the 
// guiding tube when extruding at high speed vs low. The amount of the 
// compensation is likely to depend on the plastic type and the nozzle 
// temperature. The M340 command can be used to change the default compensation
// table in G-code startup script for specific printing profiles.
// M340 - Set filament compression compensation table paramters. 
//        P<0-N> - table entry position, S<speed> - E speed in mm/sec, 
//        C<compensation> - length (in mm) of the filament compressed 
//        in the guiding tube when extruding at the given speed. 
//        The table entries should be ordered by E speed value. 
// The number of entries in this define determines the max size of the table.
// The compensation for speed 0mm/s is always 0mm and should not be listed.
// For speeds higher than listed in the table firmware uses the last row value.
// Each row: {{E0_speed, E0_compensation}, {E1_speed, E1_compensation}, ...}
//#define C_COMPENSATION  {{0.0, 0.0}, {0.0, 0.0}}, \
//                        {{0.0, 0.0}, {0.0, 0.0}}, \
//                        {{0.0, 0.0}, {0.0, 0.0}}, \
//                        {{0.0, 0.0}, {0.0, 0.0}}, \
//                        {{0.0, 0.0}, {0.0, 0.0}}

// Minimum speed at which to add/remove compensation filament. For each move the 
// firmware tries to compensate filament at the speed that assures that the 
// max-E-jerk or max-E-feedrate limits are not broken. If the move itself 
// requires E-speed at the mentioned limits (normally retract/return moves), it 
// is slowed down to allow room for the minimum compensation speed set here. 
// M340 'L' can be used to adjust the value on the fly.
#define C_COMPENSATION_MIN_SPEED { 3, 3 }

// The max compensation speed is used to limit the compensation rate for the 
// cases when firmware is not smart enough to figure the safe limits.
// M340 'H' can be used to adjust the value on the fly.
#define C_COMPENSATION_MAX_SPEED { 17, 17 }

// Note: compensation is adjusted using "best effort" strategy, i.e. when 
//       compensation is calculated for a move the firmware tries to adjust 
//       it as much as possible while performing that move then moves on 
//       to the next one and so on.

// The compensation skip window. All the short back-to-back printing moves 
// that fit into the window will use compensation value of the first move  
// that was fit into the window. The window is not used untill the planner 
// buffer is half full. The window is closed by any non-printig move even if 
// the accumulated moves distance is still shorter than the window size. 
// The window is also closed if the planner buffer becomes half empty (note: 
// several commands including extruder change wait for the buffer to be empty). 
// The window size is configured in mm. The total distance of the printing 
// moves is used to match against the the window size. The very large window 
// can be used to eliminate compensation changes between the printing moves 
// and do it only after retract, return or travel moves.
// M340 'W' can be used to adjust the value on the fly.
#define C_COMPENSATION_WINDOW { 100, 100 }

// The auto-slowdown lowers the feedrate multiplier in case the printing 
// buffer is emptied faster than the moves are added. If it is enabled the feed 
// rate multiplier (manipulated through M220 command or LCD controller) is 
// decreased (to the specified by the define percentage anmount) as new moves 
// arrive. The slowdown activated only if the printing buffer has ever reached 
// 75% during the print. It slowes the printing down if buffer goes under 25%.
// When it is activated the current feedrate multiplier is saved. It is restored 
// the printing session ends (see PRINTING_SESSION_TIMEOUT). 
//#define AUTO_SLOWDOWN 1
// AUTO_SLOWDOWN_MIN sets the minimum feed rate multiplier (in %) that 
// auto slowdown is allowd to get down to.
#define AUTO_SLOWDOWN_MIN 25
// AUTO_SLOWDOWN_BACKOFF sets the time (in milliseconds) for how long it has
// to wait after changing the multiplier before it can do it again.
#define AUTO_SLOWDOWN_BACKOFF 1000

// PRINTING_SESSION_TIMEOUT sets printing session timeout (in milliseconds). 
// If queue was drained empty and no new moves received during the timeout the 
// ongoing printing job is considered to be done. 
// The state of the printing job is used by AUTO_SLOWDOWN feature to determine 
// when to restore original feedrate multiplier (saved when the session starts).
// It is also used by LCD code to switch back from "printing" to "welocme" 
// status message.
#define PRINTING_SESSION_TIMEOUT 3000

// Uncomment the below define if the machine has individually controlled 
// hotend fans. The pins for those fans have to be defined by 
// FAN0_PIN (extruder 0 fan), FAN1_PIN (extruder 1 fan), ...
// Note: FAN_SOFT_PWM is not supported with PER_EXTRUDER_FANS
// If the fan is shared between hotends, then the fan speed for currently 
// active hotend is used.
#define PER_EXTRUDER_FANS

// Increase the FAN pwm frequency. Removes the PWM noise but increases heating in the FET/Arduino
//#define FAST_PWM_FAN

// M240  Triggers a camera by emulating a Canon RC-1 Remote
// Data from: http://www.doc-diy.net/photo/rc-1_hacked/
//#define PHOTOGRAPH_PIN     23

// SF send wrong arc g-codes when using Arc Point as fillet procedure
//#define SF_ARC_FIX

#include "Configuration_adv.h"
#include "thermistortables.h"

#endif //__CONFIGURATION_H
