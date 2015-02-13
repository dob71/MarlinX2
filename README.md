Quick Information
===================
This is Marlin X2 RepRap firmware. It is a mod of popular Marlin firmware. 
It has some extra features mostly in the area of multiple extruder support.

The default baudrate is 115200. 

The default config file is for dual X drive machines (i.e. RepRap X2V3). 
There are example configuration files for typical single extruder printers 
and RepRap X2 machines on branches "one_e" and "x2".  

See for more info:
http://www.okob.net/projects/reprap_x2v3

The firware is based on Marlin firware that in its turn is based on Sprinter firmware...

Main Features:
*   Interrupt based movement with real linear acceleration
*   High steprate
*   Look ahead (Keep the speed high when possible. High cornering speed)
*   Interrupt based temperature protection
*   Full endstop support
*   SD Card support
*   SD Card folders (works in pronterface)
*   SD Card autostart support
*   LCD support (ideally 20x4 and graphics LCD) 
*   LCD menu system for autonomous SD card printing, controlled by an click-encoder. 
*   EEPROM storage of e.g. max-velocity, max-acceleration, and similar variables
*   many small but handy things originating from bkubicek's fork.
*   Arc support
*   Temperature oversampling
*   Dynamic Temperature setpointing aka "AutoTemp"
*   Support for QTMarlin, a very beta GUI for PID-tuning and velocity-acceleration testing. https://github.com/bkubicek/QTMarlin
*   Endstop trigger reporting to the host software.
*   Updated sdcardlib
*   Heater power reporting. Useful for PID monitoring.
*   PID tuning
*   CoreXY kinematics (www.corexy.com/theory.html)
*   Configurable serial port to support connection of wireless adaptors.
*   Support for up to 3 extruders
*   Support for dual (independently moving) hotend carriages on X or Y axis.
*   Support for filament compression (in bowden filament drive) compensation.
*   Support for the follow-me mode.
*   Auto-adjustment of feed rate multiplier for planner overrun prevention.


Information on some differences and additions:
==============================================
*Temperature control:*

You can configure PID to be active only within specified range of the terget temperature.
Outside of that range simple on/off mode is used.

*Multiple extruder support:*

T_num_ [F_num_] [S_num_] - changes the extruder. 
The feedrate might be set to reposition the extruder and specify at what 
speed. If "F" is not present the coordinates are adjusted, but the move is 
not performed. "S" allows to choose what E position the just
selected extruder should start from (_num_ - the extruder number to 
pick the last position from). If "S" is not specified the last known 
position for the new selected extruder is used. For example, if Skeinforge 
generates support using absolute coordinates and you want it to be printed 
using extruder 1 while the object is printed using extruder 0, use "T1 S0" 
for switching to support printing and "T0 S1" to go back to the object printing. 
Alternatively, If you have gcode for extrider 0 and 1 generated separately 
and then mixed just use "T0" and "T1" to switch between the extruders.

By default, the extruder commands and settings are applied only to the
active extruder. For example, to change max feedrate for extruders 1 
while extruder 0 is active, one has to either use T1 option with M203 command
setting the new feedrate or switch to the extruder 1 by sending T1 command 
first and set the feedrate with M203 after that.

The M503 command (print EEPROM setting) shows info for all extruders. 
It shows each setting for extruder 0 first (without the T option).
Then it shows extruder specific command settings with T option for 
additional extruders. 

The commands affecting different extruders (like M104, M105 and M109) can 
take extruder number as a parameter. For example, in order to change the 
temperature of the extruder 1 heater without switching to it use 
"M104 S180 T1". Some commands can also be used with option A1 for acting on  
all extruders. For example, M105 can be used with option A1 to change 
its output to show temperature of all extruders starting with the active one. 
It also shows the target temperature. For example, if extruder 1 is active on 
a 2 extruder system, target temperatures set to 100deg and bed temperature
is not set: "ok T1:56/100 T0:51/100  B:20/0".

M109 can be used to wait for all extruder heaters that have temperature 
set to non-0 values by specifying non-0 'A' parameter, e.g. "M109 A1".
M109 can also take the W_num_ parameter that can change the default 
dwell time for temperature stabilization (if enabled in the config).

The "follow me" mode command (M322), turns the "follow me" mode on or 
off for extruders (T&lt;extruder&gt;  S&lt;1-on/0-off&gt;). The mode has to be off 
for the active extruder. The extruders that have the mode set repeat 
moves of the active extruder (for simultaneous printing from multiple 
extruders). The command also has H and F options for turning on/off 
follow me heater (H) and follow-me fan (F) modes. When the modes are on the 
temperature and fan setting changes of the active extruder are applied 
to the followers too. The option R allows to turn on or off the "reverse" 
mode for the follwer's extruder on dual X or Y drive machines. That mode 
allows simultaneous printing of mirrored copy of the object. Those are 
often needed when printing printer parts (for example left and right 
extruder parts) or halfs of some single object that have to be glued 
together.

Note that the firmware retract feature (FWRETRACT define) is not compatible 
with multiple extruders.


What changes came with the Marlin X2 v1.1.1:
============================================
- Merged in mainstream code with graphics LCD (DOGLCD) support
- Upgraded all LCD menus to support X2 features
- Fixed a few bugs and LCD status reporting logic
- Various optimizations and simplification of the filament compression compensation feature
- Auto slowdown feature for automatically detecting and slowing down prints where planner cannot keep up with the machine


What changes came with the Marlin X2 v1.1.0:
============================================
- Support for dual X or Y drive machines (i.e. machines that can move 
  multiple hotends independently on X or Y axis)
- M322 the "follw me" mode command
- M331/M332 (position save/restore commands)
- M340 (filament compression compensation) command
- Option to disable chit-chat (human readable "echo") printouts during 
  printing (see NO_ECHO_WHILE_PRINTING in Configuration_adv.h)
- All setup commands take 'T#' paramter for specifying extruder 
  the command applies to. For example:
    M92 T0 E661.78
    M203 T0 E23.00
    M201 T1 E5000
    M204 T0 S3000.00 R60000.00
    M205 T1 E17.00
    M218 T1 X0.00 Y-0.05
    ...
  As of this writing the complete list of comands accepting T# for 
  choosing extruder includes:
    M104, M105, M109, M106, M107, M92, M201, M203, M204, 
    M205, M206, M218, M322, M340
  Note:
  M205 now uses V for min travel feedrate and T for choosing the extruder
  M204 now uses R for retract acceleration and T for choosing the extruder
- Acceleration and retract_aceleration are now saved in EEPROM
- M109, M104 H and L options are now limited to bump temperature up or down 
  only once until opposite option is used. However, if desired, it is 
  possible to invoke the opposite option with zero degrees change and 
  further bump the temperature the same direction.
- M504 command can be used to set firmware debug flags enabling debug 
  output if ENABLE_DEBUG is defined (see Configuration_adv.h).
  Option 'S' sets the flags, option 'P' is for immediate debug actions.
- M301 option R sets the range off of the target temperature where PID 
  algorithm is to be used, outside of that range the temperature control 
  is done with simple ON/OFF logic (see PID_FUNCTIONAL_RANGE define).
- The option A can be used to turn on multiple extruder output format 
  for commands M105, M109 and M190 (can be useful for the host software)
  Example:
    SENDING:M105 A1
    ok T0:19/0 T1:19/0  B:19/0


Non-standard G-Codes, different to an old version of sprinter:
==============================================================
Movement:

*   G2  - CW ARC
*   G3  - CCW ARC

General:

*   M17  - Enable/Power all stepper motors. Compatibility to ReplicatorG.
*   M18  - Disable all stepper motors; same as M84.Compatibility to ReplicatorG.
*   M30  - Print time since last M109 or SD card start to serial
*   M42  - Change pin status via gcode
*   M80  - Turn on Power Supply
*   M81  - Turn off Power Supply
*   M114 - Output current position to serial port 
*   M119 - Output Endstop status to serial port

Movement variables:

*   M202 - Set max acceleration in units/s^2 for travel moves (M202 X1000 Y1000) Unused in Marlin!!
*   M203 - Set maximum feedrate that your machine can sustain (M203 X200 Y200 Z300 E10000) in mm/sec
*   M204 - Set default acceleration: S normal moves T filament only moves (M204 S3000 T7000) im mm/sec^2  also sets minimum segment time in ms (B20000) to prevent buffer underruns and M20 minimum feedrate
*   M206 - set home offsets.  This sets the X,Y,Z coordinates of the endstops (and is added to the {X,Y,Z}_HOME_POS configuration options (and is also added to the coordinates, if any, provided to G82, as with earlier firmware)
*   M220 - set build speed mulitplying S:factor in percent ; aka "realtime tuneing in the gcode". If you have AUTO_SLOWDOWN enabled the L (minimum) and B (backoff time) allow to control it. See Configuration.h for details.
*   M221 - set the extrude multiplying S:factor in percent
*   M400 - Finish all buffered moves.

Temperature variables:

*   M301 - Set PID parameters P I and D
*   M302 - Allow cold extrudes
*   M303 - PID relay autotune S&lt;temperature&gt; sets the target temperature. (default target temperature = 150C)

EEPROM:

*   M500 - stores paramters in EEPROM. 
*   M501 - reads parameters from EEPROM (if you need reset them after you changed them temporarily).  
*   M502 - reverts to the default "factory settings".  You still need to store them in EEPROM afterwards if you want to.
*   M503 - print the current settings (from memory not from eeprom)

MISC:

*   M240 - Trigger a camera to take a photograph
*   M999 - Restart after being stopped by error
*   M331 - Save current position coordinates (all axes, for active extruder).
           S&lt;SLOT&gt; - specifies memory slot # (0-based) to save into (default 0)
*   M332 - Apply/restore saved coordinates to the active extruder. X&lt;0|1&gt;,
           Y&lt;0|1&gt;,Z&lt;0|1&gt;,E&lt;0|1&gt; - use 1 to filter the axis in (default), 0 to 
           filter it out. F&lt;speed&gt; - make move to the restored position, if 
           'F' is not used the restored coordinates set as current position. 
           S&lt;SLOT&gt; - specifies memory slot # (0-based) to restore from 
           (default 0)
*   M504 - Set/clear debug flags (see ENABLE_DEBUG in Marlin.h). For example 
           M504 S1 enables 'echo' output during printing even if firmware
           is compiled with NO_ECHO_WHILE_PRINTING.


MULTIPLE EXTRUDERS:

*   T - changes active extruder (details in the section above) 


FILAMENT COMPRESSION COMPENSATION:

*   M340 - (compression compensation) command. That command allows to specify 
    a table of values telling the firmware how much the filament being pushed 
    to the hotend gets compressed depending on the extrusion speed. The firmware 
    tries to compensate accordingly. This is still an experimental feature. 
    It covers the functionality similar to "advance algorithm" that was not 
    working in the original Marlin firmware.


Configuring and compilation:
============================
Install the arduino software IDE/toolset v1.0.6
   http://www.arduino.cc/en/Main/Software

Copy the Marlin firmware
   https://github.com/dob71/MarlinX2
   (Clone or use zip download)

Start the arduino IDE.

Select Tools -&gt; Board -&gt; Arduino Mega 2560 or your microcontroller.

Select the correct serial port in Tools -&gt;Serial Port.

Add libraries (override if exist) from
"&lt;MarlinX2 folder&gt;/Marlin/ArduinoAddons/Arduino_1.x.x/libraries/" to 
"&lt;Arduino folder&gt;/libraries/".

Open Marlin.pde.

Edit the &lt;MarlinX2 folder&gt;/Marlin/Configuration.h to match you printer.

Click the Verify/Compile button.

Click the Upload button. If all goes well the firmware is uploading.

Enjoy Silky Smooth Printing.
