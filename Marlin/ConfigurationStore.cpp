#include "Marlin.h"
#include "planner.h"
#include "temperature.h"
#include "ultralcd.h"
#include "ConfigurationStore.h"

void _EEPROM_writeData(int &pos, uint8_t* value, uint8_t size)
{
    do {
        eeprom_write_byte((unsigned char*)pos, *value);
        pos++;
        value++;
    } while(--size);
}
#define EEPROM_WRITE_VAR(pos, value) _EEPROM_writeData(pos, (uint8_t*)&value, sizeof(value))

void _EEPROM_readData(int &pos, uint8_t* value, uint8_t size)
{
    do {
        *value = eeprom_read_byte((unsigned char*)pos);
        pos++;
        value++;
    } while(--size);
}
#define EEPROM_READ_VAR(pos, value) _EEPROM_readData(pos, (uint8_t*)&value, sizeof(value))

//======================================================================================




#define EEPROM_OFFSET 100


// IMPORTANT:  Whenever there are changes made to the variables stored in EEPROM
// in the functions below, also increment the version number. This makes sure that
// the default values are used whenever there is a change to the data, to prevent
// wrong data being written to the variables.
// ALSO:  always make sure the variables in the Store and retrieve sections are in 
// the same order.
#define EEPROM_VERSION "X08"


#ifdef EEPROM_SETTINGS
void Config_StoreSettings() 
{
  char ver[4]= "000";
  int i=EEPROM_OFFSET;
  int extruders = EXTRUDERS;
  EEPROM_WRITE_VAR(i,ver); // invalidate data first 
  EEPROM_WRITE_VAR(i,extruders); 
  EEPROM_WRITE_VAR(i,axis_steps_per_unit);  
  EEPROM_WRITE_VAR(i,max_feedrate);  
  EEPROM_WRITE_VAR(i,max_acceleration_units_per_sq_second);
  EEPROM_WRITE_VAR(i,acceleration);
  EEPROM_WRITE_VAR(i,retract_acceleration);
  EEPROM_WRITE_VAR(i,minimumfeedrate);
  EEPROM_WRITE_VAR(i,mintravelfeedrate);
  EEPROM_WRITE_VAR(i,minsegmenttime);
  EEPROM_WRITE_VAR(i,max_xy_jerk);
  EEPROM_WRITE_VAR(i,max_z_jerk);
  EEPROM_WRITE_VAR(i,max_e_jerk);
  #ifdef ENABLE_ADD_HOMEING
  EEPROM_WRITE_VAR(i,add_homeing);
  #else  // ENABLE_ADD_HOMEING
  for(int ii = 0; ii < EXTRUDERS * 3; ii++) {
    float z = 0.0;
    EEPROM_WRITE_VAR(i,z);
  }
  #endif // ENABLE_ADD_HOMEING
  #if EXTRUDERS > 1
  EEPROM_WRITE_VAR(i,extruder_offset);
  #endif // EXTRUDERS > 1
  #ifndef ULTIPANEL
  int plaPreheatHotendTemp = PLA_PREHEAT_HOTEND_TEMP;
  int plaPreheatHPBTemp = PLA_PREHEAT_HPB_TEMP;
  int plaPreheatFanSpeed = PLA_PREHEAT_FAN_SPEED;
  int absPreheatHotendTemp = ABS_PREHEAT_HOTEND_TEMP;
  int absPreheatHPBTemp = ABS_PREHEAT_HPB_TEMP;
  int absPreheatFanSpeed = ABS_PREHEAT_FAN_SPEED;
  #endif
  EEPROM_WRITE_VAR(i,plaPreheatHotendTemp);
  EEPROM_WRITE_VAR(i,plaPreheatHPBTemp);
  EEPROM_WRITE_VAR(i,plaPreheatFanSpeed);
  EEPROM_WRITE_VAR(i,absPreheatHotendTemp);
  EEPROM_WRITE_VAR(i,absPreheatHPBTemp);
  EEPROM_WRITE_VAR(i,absPreheatFanSpeed);
  #if !defined(PIDTEMP)
    float Kp = 3000, Ki = 0, Kd = 0, Kr = 0;
  #elif !defined(PID_FUNCTIONAL_RANGE)
    float Kr = 0;
  #endif // PIDTEMP
  EEPROM_WRITE_VAR(i,Kp);
  EEPROM_WRITE_VAR(i,Ki);
  EEPROM_WRITE_VAR(i,Kd);
  EEPROM_WRITE_VAR(i,Kr);
  char ver2[4]=EEPROM_VERSION;
  i=EEPROM_OFFSET;
  EEPROM_WRITE_VAR(i,ver2); // validate data
  SERIAL_ECHO_START;
  SERIAL_ECHOLNPGM("Settings Stored");
}
#endif //EEPROM_SETTINGS


#ifdef EEPROM_CHITCHAT
void Config_PrintSettings()
{  // Always have this function, even with EEPROM_SETTINGS disabled, the current values will be shown
    int i;
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Steps per unit:");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M92 X",axis_steps_per_unit[0]);
    SERIAL_ECHOPAIR(" Y",axis_steps_per_unit[1]);
    SERIAL_ECHOPAIR(" Z",axis_steps_per_unit[2]);
    SERIAL_ECHOPAIR(" E",axis_steps_per_unit[3]);
    SERIAL_ECHOLN("");
    #if (EXTRUDERS > 1)
    for(i = 1; i < EXTRUDERS; i++)
    {
      SERIAL_ECHO_START;
      SERIAL_ECHOPAIR("  M92 T", i);
      SERIAL_ECHOPAIR(" E",axis_steps_per_unit[3 + i]);
      SERIAL_ECHOLN("");
    }
    #endif
      
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Maximum feedrates (mm/s):");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M203 X",max_feedrate[0]);
    SERIAL_ECHOPAIR(" Y",max_feedrate[1] ); 
    SERIAL_ECHOPAIR(" Z", max_feedrate[2] ); 
    SERIAL_ECHOPAIR(" E", max_feedrate[3]);
    SERIAL_ECHOLN("");
    #if (EXTRUDERS > 1)
    for(i = 1; i < EXTRUDERS; i++)
    {
      SERIAL_ECHO_START;
      SERIAL_ECHOPAIR("  M203 T", i);
      SERIAL_ECHOPAIR(" E", max_feedrate[3 + i]);
      SERIAL_ECHOLN("");
    }
    #endif

    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Maximum Acceleration (mm/s2):");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M201 X" ,max_acceleration_units_per_sq_second[0] ); 
    SERIAL_ECHOPAIR(" Y" , max_acceleration_units_per_sq_second[1] ); 
    SERIAL_ECHOPAIR(" Z" ,max_acceleration_units_per_sq_second[2] );
    SERIAL_ECHOPAIR(" E" ,max_acceleration_units_per_sq_second[3]);
    SERIAL_ECHOLN("");
    #if (EXTRUDERS > 1)
    for(i = 1; i < EXTRUDERS; i++)
    {
      SERIAL_ECHO_START;
      SERIAL_ECHOPAIR("  M201 T", i);
      SERIAL_ECHOPAIR(" E" ,max_acceleration_units_per_sq_second[3 + i]);
      SERIAL_ECHOLN("");
    }
    #endif

    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Acceleration: S=acceleration, R=retract acceleration:");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M204 S",acceleration); 
    SERIAL_ECHOPAIR(" R" ,retract_acceleration[0]);
    SERIAL_ECHOLN("");
    #if (EXTRUDERS > 1)
    for(i = 1; i < EXTRUDERS; i++)
    {
      SERIAL_ECHO_START;
      SERIAL_ECHOPAIR("  M204 T", i);
      SERIAL_ECHOPAIR(" R" ,retract_acceleration[i]);
      SERIAL_ECHOLN("");
    }
    #endif

    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Advanced variables: S=Min feedrate (mm/s), M=Min travel feedrate (mm/s), B=minimum segment time (us), X=max XY jerk (mm/s), Z=max Z jerk (mm/s), E=max E jerk (mm/s)");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M205 S",minimumfeedrate ); 
    SERIAL_ECHOPAIR(" V" ,mintravelfeedrate ); 
    SERIAL_ECHOPAIR(" B" ,minsegmenttime ); 
    SERIAL_ECHOPAIR(" X" ,max_xy_jerk ); 
    SERIAL_ECHOPAIR(" Z" ,max_z_jerk);
    SERIAL_ECHOPAIR(" E" ,max_e_jerk[0]);
    SERIAL_ECHOLN(""); 
    #if (EXTRUDERS > 1)
    for(i = 1; i < EXTRUDERS; i++)
    {
      SERIAL_ECHO_START;
      SERIAL_ECHOPAIR("  M205 T", i);
      SERIAL_ECHOPAIR(" E" ,max_e_jerk[i]);
      SERIAL_ECHOLN("");
    }
    #endif

    #ifdef ENABLE_ADD_HOMEING
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Home offset (mm):");
    for(i = 0; i < EXTRUDERS; i++)
    {
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M206 X",add_homeing[i][0] );
    SERIAL_ECHOPAIR(" Y" ,add_homeing[i][1] );
    SERIAL_ECHOPAIR(" Z" ,add_homeing[i][2] );
    #if (EXTRUDERS > 1)
    SERIAL_ECHOPAIR(" T", i);
    #endif
    SERIAL_ECHOLN("");
    }
    #endif  // ENABLE_ADD_HOMEING
    
#ifdef PIDTEMP
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("PID settings:");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("   M301 P",Kp); 
    SERIAL_ECHOPAIR(" I" ,Ki/PID_dT); 
    SERIAL_ECHOPAIR(" D" ,Kd*PID_dT);
    #ifdef PID_FUNCTIONAL_RANGE
    SERIAL_ECHOPAIR(" R" ,Kr);
    #endif
    SERIAL_ECHOLN(""); 
#endif
} 
#endif


#ifdef EEPROM_SETTINGS
void Config_RetrieveSettings()
{
    int i=EEPROM_OFFSET;
    char stored_ver[4];
    char ver[4]=EEPROM_VERSION;
    int extruders = 0;
    EEPROM_READ_VAR(i, stored_ver); //read stored version
    if (strncmp(ver, stored_ver, 3) == 0) 
    {   // version number match, now get the number of extruders
       EEPROM_READ_VAR(i, extruders); //read number of extruders
    }
    //  SERIAL_ECHOLN("Version: [" << ver << "] Stored version: [" << stored_ver << "]");
    if ((strncmp(ver,stored_ver,3) == 0) && (extruders == EXTRUDERS))
    {   // version and extruders numbers match
      EEPROM_READ_VAR(i,axis_steps_per_unit);  
      EEPROM_READ_VAR(i,max_feedrate);  
      EEPROM_READ_VAR(i,max_acceleration_units_per_sq_second);
      EEPROM_READ_VAR(i,acceleration);
      EEPROM_READ_VAR(i,retract_acceleration);
      EEPROM_READ_VAR(i,minimumfeedrate);
      EEPROM_READ_VAR(i,mintravelfeedrate);
      EEPROM_READ_VAR(i,minsegmenttime);
      EEPROM_READ_VAR(i,max_xy_jerk);
      EEPROM_READ_VAR(i,max_z_jerk);
      EEPROM_READ_VAR(i,max_e_jerk);
      #ifdef ENABLE_ADD_HOMEING
      EEPROM_READ_VAR(i,add_homeing);
      #else // ENABLE_ADD_HOMEING
      for(int ii = 0; ii < EXTRUDERS * 3; ii++) {
         float z;
         EEPROM_READ_VAR(i,z);
      }
      #endif // ENABLE_ADD_HOMEING
      #if EXTRUDERS > 1
      EEPROM_READ_VAR(i,extruder_offset);
      #endif // EXTRUDERS > 1
      #ifndef ULTIPANEL
      int plaPreheatHotendTemp;
      int plaPreheatHPBTemp;
      int plaPreheatFanSpeed;
      int absPreheatHotendTemp;
      int absPreheatHPBTemp;
      int absPreheatFanSpeed;
      #endif
      EEPROM_READ_VAR(i,plaPreheatHotendTemp);
      EEPROM_READ_VAR(i,plaPreheatHPBTemp);
      EEPROM_READ_VAR(i,plaPreheatFanSpeed);
      EEPROM_READ_VAR(i,absPreheatHotendTemp);
      EEPROM_READ_VAR(i,absPreheatHPBTemp);
      EEPROM_READ_VAR(i,absPreheatFanSpeed);
      #ifndef PIDTEMP
        float Kp,Ki,Kd,Kr;
      #elif !defined(PID_FUNCTIONAL_RANGE)
        float Kr;
      #endif
      EEPROM_READ_VAR(i,Kp);
      EEPROM_READ_VAR(i,Ki);
      EEPROM_READ_VAR(i,Kd);
      EEPROM_READ_VAR(i,Kr);
      #ifdef PIDTEMP
      if(Kp <= 0) Kp = DEFAULT_Kp;
      if(Ki <= 0) Ki = (DEFAULT_Ki*PID_dT);
      if(Kd <= 0) Kd = (DEFAULT_Kd/PID_dT);
      #ifdef PID_FUNCTIONAL_RANGE
      if(Kr <= 0) Kr = PID_FUNCTIONAL_RANGE;
      #endif
      updatePID();
      #endif
      SERIAL_ECHO_START;
      SERIAL_ECHOLNPGM("Stored settings retreived:");
      Config_PrintSettings();
    }
    else
    {
      Config_ResetDefault();
    }
}
#endif


void Config_ResetDefault()
{
    float tmp1[] = DEFAULT_AXIS_STEPS_PER_UNIT;
    float tmp2[] = DEFAULT_MAX_FEEDRATE;
    long  tmp3[] = DEFAULT_MAX_ACCELERATION;
    long  tmp4[] = DEFAULT_RETRACT_ACCELERATION; 
    long  tmp5[] = DEFAULT_EJERK;
    #if EXTRUDERS > 1
    float tmp6[] = EXTRUDER_OFFSET_X;
    float tmp7[] = EXTRUDER_OFFSET_Y;
    #else // EXTRUDERS > 1
    float tmp6[] = { 0.0 };
    float tmp7[] = { 0.0 };
    #endif // EXTRUDERS > 1
    
    // Populate missing values for extruders from the last value in arrays
    for (short i = 0; i < (3 + EXTRUDERS); i++) 
    {
      short max_i;
      max_i = sizeof(tmp1)/sizeof(*tmp1);
      if(i < max_i)
        axis_steps_per_unit[i]=tmp1[i];
      else
        axis_steps_per_unit[i]=tmp1[max_i - 1];
      max_i = sizeof(tmp2)/sizeof(*tmp2);
      if(i < max_i)
        max_feedrate[i]=tmp2[i];
      else
        max_feedrate[i]=tmp2[max_i - 1];
      max_i = sizeof(tmp3)/sizeof(*tmp3);
      if(i < max_i)
        max_acceleration_units_per_sq_second[i]=tmp3[i];
      else
        max_acceleration_units_per_sq_second[i]=tmp3[max_i - 1];
      if(i < EXTRUDERS)
      {
        #ifdef ENABLE_ADD_HOMEING
        add_homeing[i][0] = add_homeing[i][1] = add_homeing[i][2] = 0;
        #endif // ENABLE_ADD_HOMEING
        max_i = sizeof(tmp4)/sizeof(*tmp4);
        if(i < max_i)
          retract_acceleration[i]=tmp4[i];
        else
          retract_acceleration[i]=tmp4[max_i - 1];
        max_i = sizeof(tmp5)/sizeof(*tmp5);
        if(i < max_i)
          max_e_jerk[i]=tmp5[i];
        else
          max_e_jerk[i]=tmp5[max_i - 1];
        #if EXTRUDERS > 1
        max_i = sizeof(tmp6)/sizeof(*tmp6);
        if(i < max_i)
          extruder_offset[X_AXIS][i]=tmp6[i];
        else
          extruder_offset[X_AXIS][i]=0;
        max_i = sizeof(tmp7)/sizeof(*tmp7);
        if(i < max_i)
          extruder_offset[Y_AXIS][i]=tmp7[i];
        else
          extruder_offset[Y_AXIS][i]=0;
        #endif // EXTRUDERS > 1
      }
    }
    acceleration = DEFAULT_ACCELERATION;
    minimumfeedrate = DEFAULT_MINIMUMFEEDRATE;
    minsegmenttime = DEFAULT_MINSEGMENTTIME;       
    mintravelfeedrate = DEFAULT_MINTRAVELFEEDRATE;
    max_xy_jerk = DEFAULT_XYJERK;
    max_z_jerk=DEFAULT_ZJERK;
#ifdef ULTIPANEL
    plaPreheatHotendTemp = PLA_PREHEAT_HOTEND_TEMP;
    plaPreheatHPBTemp = PLA_PREHEAT_HPB_TEMP;
    plaPreheatFanSpeed = PLA_PREHEAT_FAN_SPEED;
    absPreheatHotendTemp = ABS_PREHEAT_HOTEND_TEMP;
    absPreheatHPBTemp = ABS_PREHEAT_HPB_TEMP;
    absPreheatFanSpeed = ABS_PREHEAT_FAN_SPEED;
#endif
#ifdef PIDTEMP
    Kp = DEFAULT_Kp;
    Ki = (DEFAULT_Ki*PID_dT);
    Kd = (DEFAULT_Kd/PID_dT);
#ifdef PID_ADD_EXTRUSION_RATE
    Kc = DEFAULT_Kc;
#endif//PID_ADD_EXTRUSION_RATE
#endif//PIDTEMP
    SERIAL_ECHO_START;
    SERIAL_ECHOLN("Using Default settings:");
    Config_PrintSettings();
}
