WARNING: 
--------
THIS IS MARLIN X2 1.1.0 Beta 1
Merged Marlin RC2 with MARLIN X2 1.0.0
Several new features added

The configuration is now split in two files
Configuration.h for the normal settings
Configuration_adv.h for the advanced settings

Gen7T is not supported.

Quick Information
===================
This RepRap firmware is a mashup between <a href="https://github.com/kliment/Sprinter">Sprinter</a>, <a href="https://github.com/simen/grbl/tree">grbl</a> and many original parts.
On top of that it has multiple extruder support.

Derived from Sprinter and Grbl by Erik van der Zalm.
Sprinters lead developers are Kliment and caru.
Grbls lead developer is Simen Svale Skogsrud. Sonney Jeon (Chamnit) improved some parts of grbl
A fork by bkubicek for the Ultimaker was merged, and further development was aided by him.
Some features have been added by:
Lampmaker, Bradley Feldman, and others...

Features:

*   Interrupt based movement with real linear acceleration
*   High steprate
*   Look ahead (Keep the speed high when possible. High cornering speed)
*   Interrupt based temperature protection
*   preliminary support for Matthew Roberts advance algorithm 
    For more info see: http://reprap.org/pipermail/reprap-dev/2011-May/003323.html
*   Full endstop support
*   SD Card support
*   SD Card folders (works in pronterface)
*   SD Card autostart support
*   LCD support (ideally 20x4) 
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
*   Support for filament compression (bowden filament drive) compensation.
*   Support for the follow-me mode.

The default baudrate is 115200. You can try 250000 it is less supported by drivers and host-environments, but might work better for you.


Differences and additions to the already good Sprinter firmware:
================================================================

*Look-ahead:*

Marlin has look-ahead. While sprinter has to break and re-accelerate at each corner, 
lookahead will only decelerate and accelerate to a velocity, 
so that the change in vectorial velocity magnitude is less than the xy_jerk_velocity.
This is only possible, if some future moves are already processed, hence the name. 
It leads to less over-deposition at corners, especially at flat angles.

*Arc support:*

Slic3r can find curves that, although broken into segments, were ment to describe an arc.
Marlin is able to print those arcs. The advantage is the firmware can choose the resolution,
and can perform the arc with nearly constant velocity, resulting in a nice finish. 
Also, less serial communication is needed.

*Temperature Oversampling:*

To reduce noise and make the PID-differential term more useful, 16 ADC conversion results are averaged.

*AutoTemp:*

If your gcode contains a wide spread of extruder velocities, or you realtime change the building speed, the temperature should be changed accordingly.
Usually, higher speed requires higher temperature.
This can now be performed by the AutoTemp function
By calling M109 S<mintemp> T<maxtemp> F<factor> you enter the autotemp mode.

You can leave it by calling M109 without any F.
If active, the maximal extruder stepper rate of all buffered moves will be calculated, and named "maxerate" [steps/sec].
The wanted temperature then will be set to t=tempmin+factor*maxerate, while being limited between tempmin and tempmax.
If the target temperature is set manually or by gcode to a value less then tempmin, it will be kept without change.
Ideally, your gcode can be completely free of temperature controls, apart from a M109 S T F in the start.gcode, and a M109 S0 in the end.gcode.

*EEPROM:*

If you know your PID values, the acceleration and max-velocities of your unique machine, you can set them, and finally store them in the EEPROM.
After each reboot, it will magically load them from EEPROM, independent what your Configuration.h says.

*LCD Menu:*

If your hardware supports it, you can build yourself a LCD-CardReader+Click+encoder combination. It will enable you to realtime tune temperatures,
accelerations, velocities, flow rates, select and print files from the SD card, preheat, disable the steppers, and do other fancy stuff.
One working hardware is documented here: http://www.thingiverse.com/thing:12663 
Also, with just a 20x4 or 16x2 display, useful data is shown.

*SD card folders:*

If you have an SD card reader attached to your controller, also folders work now. Listing the files in pronterface will show "/path/subpath/file.g".
You can write to file in a subfolder by specifying a similar text using small letters in the path.
Also, backup copies of various operating systems are hidden, as well as files not ending with ".g".

*SD card folders:*

If you place a file auto[0-9].g into the root of the sd card, it will be automatically executed if you boot the printer. The same file will be executed by selecting "Autostart" from the menu.
First *0 will be performed, than *1 and so on. That way, you can heat up or even print automatically without user interaction.

*Endstop trigger reporting:*

If an endstop is hit while moving towards the endstop, the location at which the firmware thinks that the endstop was triggered is outputed on the serial port.
This is useful, because the user gets a warning message.
However, also tools like QTMarlin can use this for finding acceptable combinations of velocity+acceleration.

*Coding paradigm:*

Not relevant from a user side, but Marlin was split into thematic junks, and has tried to partially enforced private variables.
This is intended to make it clearer, what interacts which what, and leads to a higher level of modularization.
We think that this is a useful prestep for porting this firmware to e.g. an ARM platform in the future.
A lot of RAM (with enabled LCD ~2200 bytes) was saved by storing char []="some message" in Program memory.
In the serial communication, a #define based level of abstraction was enforced, so that it is clear that
some transfer is information (usually beginning with "echo:"), an error "error:", or just normal protocol,
necessary for backwards compatibility.

*Interrupt based temperature measurements:*

An interrupt is used to manage ADC conversions, and enforce checking for critical temperatures.
This leads to less blocking in the heater management routine.

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
to start support printing and "T0 S1" to go go back to the object printing. 
Alternatively, If you have gcode for extrider 0 and 1 generated separately 
and then mixed just use T0 and T1 to switch between the extruders.

By default, the extruder commands and settings are applied only to the
active extruder. For example, to change max feedrate for extruders 1 
while extruder 0 is active, one has to either use T1 option with M203 command
setting the new feedrate or switch to the extruder 1 by sending T1 command 
and set the feedrate with M203 after that.

The M503 command (print EEPROM setting) shows info for all extruders. 
It shows the setting for extruder 0 first (without the T option).
Then it shows extruder specific command settings with T option for 
additional extruders. 

The commands affecting different extruders (like M104, M105 and M109) can 
take extruder number as a parameter. For example, in order to change the 
temperature of the extruder 1 heater without switching to it use 
"M104 S180 T1". Some commands can also be used with option A1 for affecting 
all extruders. For example, M105 can be used with option A1 that will change 
its output to show temperature of all extruders starting with the active one 
as well as the target temperature (for example, if extruder 0 is active on 
a 2 extruder system, target temperature set to 100deg and bed temperature
is not set: "ok T0:56/100 T1:51/100  B:20/0").

M109 can be used to wait for all extruder heaters that have temperature 
set to non-0 values by specifying non-0 'A' parameter, e.g. "M109 A1".
M109 can also take the W_num_ parameter that can change the default 
dwell time for temperature stabilization (if enabled in the config).

The "follw me" mode command (M322), turns the "follow me" mode on or 
off for extruders (T<extruder>  S<1-on/0-off>). The mode has to be off 
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

Note that the firmware retract feature (FWRETRACT define) is not 
multiple extruder compatible.


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
*   M220 - set build speed mulitplying S:factor in percent ; aka "realtime tuneing in the gcode". So you can slow down if you have islands in one height-range, and speed up otherwise.
*   M221 - set the extrude multiplying S:factor in percent
*   M400 - Finish all buffered moves.

Temperature variables:

*   M301 - Set PID parameters P I and D
*   M302 - Allow cold extrudes
*   M303 - PID relay autotune S<temperature> sets the target temperature. (default target temperature = 150C)

EEPROM:

*   M500 - stores paramters in EEPROM. This parameters are stored:  axis_steps_per_unit,  max_feedrate, max_acceleration  ,acceleration,retract_acceleration,
  minimumfeedrate,mintravelfeedrate,minsegmenttime,  jerk velocities, PID
*   M501 - reads parameters from EEPROM (if you need reset them after you changed them temporarily).  
*   M502 - reverts to the default "factory settings".  You still need to store them in EEPROM afterwards if you want to.
*   M503 - print the current settings (from memory not from eeprom)

MISC:

*   M240 - Trigger a camera to take a photograph
*   M999 - Restart after being stopped by error
*   M331 - Save current position coordinates (all axes, for active extruder).
           S<SLOT> - specifies memory slot # (0-based) to save into (default 0)
*   M332 - Apply/restore saved coordinates to the active extruder. X<0|1>,
           Y<0|1>,Z<0|1>,E<0|1> - use 1 to filter the axis in (default), 0 to 
           filter it out. F<speed> - make move to the restored position, if 
           'F' is not used the restored coordinates set as current position. 
           S<SLOT> - specifies memory slot # (0-based) to restore from 
           (default 0)


MULTIPLE EXTRUDERS:

*   T - changes active extruder (details in the section above) 


FILAMENT COMPRESSION COMPENSATION:

*   M340 - (compression compensation) command. That command allows to specify 
    a table of values telling the firmware how much the filament being 
    pushed to the hotend compresses depending on how fast it has to be extruded 
    and compensate. That is still experimental. It covers the functionality 
    called "ADVANCE" (the ADVANCE code was broken in Marlin).


Configuring and compilation:
============================

Install the arduino software IDE/toolset v23
   http://www.arduino.cc/en/Main/Software

Copy the Marlin firmware
   https://github.com/dob71/Marlin/tree/m
   (Use the download button)

Start the arduino IDE.
Select Tools -> Board -> Arduino Mega 2560 or your microcontroller
Select the correct serial port in Tools ->Serial Port
Open Marlin.pde

Click the Verify/Compile button

Click the Upload button
If all goes well the firmware is uploading

Enjoy Silky Smooth Printing.
