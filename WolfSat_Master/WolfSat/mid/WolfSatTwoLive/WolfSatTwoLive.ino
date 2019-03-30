/*
 *  In Memory of Randy
 */
/*
 * This is the WolfSat RT'OS' for the FRCC-BCC Engineering Club.
 * It queries several sensors for data, logs them into a template
 * object (DataSet<>), and then sends them to an SD card for data
 * logging.
 * 
 * By James Craft, and Tyler Dow
 */

 // WolfSat_lib inclusion
#include <DataSet.h>


// External sensor libraries
#include "sps30.h"
#include <unistd.h>
#include <SparkFunTMP102.h>
#include <LSM9DS1_Registers.h>
#include <LSM9DS1_Types.h>
#include <SparkFunLSM9DS1.h>
//#include <SparkFun_Qwiic_OpenLog_Arduino_Library.h>
#include "SparkFun_SCD30_Arduino_Library.h" 
#include <SparkFun_HIH4030.h>
#include <Wire.h>

// Static consts for external sensors
#define LSM9DS1_M   0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW
#define HIGH_TEMP 50
#define LOW_TEMP 49
#define SPS30_DEBUG 0 // Can be changed to get debug info, 0 is none
#define HIH4030_VCC 5 // Operating at optimal 5v
//#define SCD30_RDY 12

// Pins
#define PIN_HEATER0 12
#define PIN_HEATER1 11
#define PIN_HEATER2 10
#define PIN_RESET 38
#define PIN_RADNOISE 3
#define PIN_RADSIG 2
#define PIN_PRESSURE A5
#define PIN_HUMIDITY A4
#define PIN_EXTTMP A3
#define INT_COULOMB A15
#define INT_IMUM PJ4
#define INT_IMU2 PJ3
#define INT_IMU1 PJ2
#define PIN_DBG 9
#define PIN_CO2RDY 8
//#define HIH4030_PIN A4


// Static consts for DataSets
#define LOG_CMD "LOG_CMD" // Coulomb counter included here...
#define LOG_TMP "LOG_TMP" // all TMP102, maybe tmp36
#define LOG_IMU "LOG_IMU" // IMU
#define LOG_PAR "LOG_PAR" // Particulates
#define LOG_RAD "LOG_RAD" // Geiger
#define LOG_ATM "LOG_ATM" // Pressure, humidity, maybe TMP36, CO2, CH4

#define TIME_POINTS 4
#define DATA_POINT 1
#define LIM_IMU 9
#define LIM_TMP 4
#define LIM_SPS30 10
#define LIM_ATM 4
#define LIM_RAD 4


// Static consts for others
#define DEBUG_SPEED 115200
#define OPLOG_SPEED 9600
#define PARTI_SPEED 115200
#define NULLSTR " "
#define TYPE_TXT ".txt"
#define TYPE_CSV ".csv"

// Preprocesor directives
#if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define LOGG1 Serial1
#define LOGG2 Serial2
#define PARTI Serial
#define DEBUG Serial3
#else
// This code was designed to run on an Arduino Mega, 
// but needs to be accomodating to UNO debugs 
#define DEBUG Serial
#define LOGG1 Serial
#define LOGG2 Serial
#define PARTI Serial
#endif

// Global vars
bool debugging;
bool verboseDebug;
int cmdCount;
String activeLog;
int hour;
int minute;
int second;
long mil;
long lastMil;
DataSet<double> imuDat;
DataSet<double> tmpDat;
DataSet<double> sps30Dat;
DataSet<double> atmDat;
LSM9DS1 imu;
TMP102 innerTemp1(0x48);
SPS30 sps30;
SCD30 scd30;
HIH4030 hih4030(PIN_HUMIDITY, HIH4030_VCC); // No other setup required for this sensor
double oldp,oldD;
double lastPIDrun = millis();

// Global Vars for datapoint tracking
int dp_CMD;
int dp_TMP;
int dp_IMU;
int dp_PAR;
int dp_RAD;
int dp_ATM;

// Special debug
//#define PIN_DBG 7

void setup() 
{
  delay(1000);
  setup_PINS();
  setup_VARS();
  setup_DATASETS();
  setup_SERIAL();
  //delay(100);
  setup_DEBUG();
  setup_LOGG(LOGG1);
  //setup_LOGG(LOGG2);
  
  if(debugging)
  {
    dubLog(1);
  }    
  else
    dubLog(2); 

  if (debugging)
    DEBUG.println("SETUP :: SETTING UP IMU...");
  setup_IMU();
  if(debugging)
  {
    dubLog(4);
  }

  if(debugging)
    DEBUG.println("SETUP :: SETTING UP TMP102...");
  setup_TMP102(innerTemp1);
  if(debugging)
  {
    dubLog(3);
  }

  if(debugging)
    DEBUG.println("SETUP :: SETTING UP SPS30...");
  setup_SPS30();
  if (debugging)
  {
    dubLog(6);
  }

  if(debugging)
    DEBUG.println("SETUP :: SETTING UP SCD30...");
  setup_SCD30();
  if (debugging)
  {
    dubLog(7);  
  }
}


void setup_PINS()
{
  pinMode(PIN_DBG, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_CO2RDY, INPUT);
}


void setup_DEBUG()
{
  if(digitalRead(PIN_DBG) == HIGH)
  {
    debugging = true;
    //DEBUG.println("Hello!");
    int cnt = millis();
    int lim = cnt + 5000;
    while(true)
    {      
      DEBUG.println("Listen up LittleWolf");
      //DEBUG.flush();
      //delay(20);
      if(DEBUG.available() > 0)
      {
        if(DEBUG.readStringUntil('\r') == "I'm listening...")
        {
          DEBUG.println("Hello LittleWolf...");
          break;
        }
      }
      else
      {
        cnt = millis();
      }
  }
  }
}


void setup_DATASETS()
{
  if(debugging)
    DEBUG.println("SETTING UP DATASETS...");
  sps30Dat = DataSet<double>(DATA_POINT + TIME_POINTS + LIM_SPS30);
  imuDat = DataSet<double>(DATA_POINT + TIME_POINTS + LIM_IMU);
  tmpDat = DataSet<double>(DATA_POINT + TIME_POINTS + LIM_TMP); // Also initializes dataSet used for TMP36
  atmDat = DataSet<double>(DATA_POINT + TIME_POINTS + LIM_ATM);
}


void setup_VARS()
{
  if(debugging)
    DEBUG.println("SETTING UP VARS...");
  debugging = true;  // <<<<<< Could be pin controlled...<<<<<<<<<
  verboseDebug = true;  // Same here...
  cmdCount = 0;
  activeLog = "setup";
  hour = 0;
  minute = 0;
  second = 0;
  mil = millis();
  lastMil = mil;
  dp_CMD = 0;
  dp_TMP = 0;
  dp_IMU = 0;
  dp_PAR = 0;
  dp_RAD = 0;
  dp_ATM = 0;
}


void setup_SCD30()
{
  if(debugging)
    DEBUG.println("SETTING UP CO2 SENSOR...");
  if(!scd30.begin())
  {
    if(debugging)
    {
      DEBUG.println("SCD30 :: INIT FAILURE");  
    }    
  }
  else
  {
    if(debugging)
      DEBUG.println("SCD30 :: INIT SUCCESS");
    //scd30.beginMeasuring();

  }
}


void setup_SPS30()
{
  if(debugging)
    DEBUG.println("SETTING UP PARTICULATE SENSOR...");
  sps30.EnableDebugging(SPS30_DEBUG);
  if ((sps30.begin(SERIALPORT) == false)&&(debugging))
    DEBUG.println("SPS  :: COMM INIT FAILED");
  if ((sps30.probe() == false)&&(debugging))
    DEBUG.println("SPS  :: PROBE INIT FAILED");
  else if (debugging)
    DEBUG.println("SPS  :: PROBE INIT SUCCESS");
  if ((sps30.reset() == false)&&(debugging))
    DEBUG.println("SPS  :: RESET FAIL");
  if ((sps30.start() == true)&&(debugging))
    DEBUG.println("SPS  :: BEGINNING MEASUREMENT");
  else if(debugging)
    DEBUG.println("SPS  :: MEASUREMENT FAILURE");
}


void setup_SERIAL()
{
  if(debugging)
  {
    DEBUG.begin(DEBUG_SPEED);
    DEBUG.println("WOLFSAT RTOS");
    DEBUG.println("SETTING UP SERIAL...");
  }
  LOGG1.begin(OPLOG_SPEED);
  //LOGG2.begin(OPLOG_SPEED);
  PARTI.begin(PARTI_SPEED);
}


void setup_LOGG(HardwareSerial& in_serial)
{
  if(debugging)
    DEBUG.println("SETTING UP OPENLOG");
  oLog_append(in_serial, LOG_CMD, TYPE_CSV, LOG_CMD);
  oLog_append(in_serial, LOG_TMP, TYPE_CSV, LOG_TMP);
  oLog_append(in_serial, LOG_IMU, TYPE_CSV, LOG_IMU);
  oLog_append(in_serial, LOG_PAR, TYPE_CSV, LOG_PAR);
  oLog_append(in_serial, LOG_RAD, TYPE_CSV, LOG_RAD);
  oLog_append(in_serial, LOG_ATM, TYPE_CSV, LOG_ATM);
  oLog_logCMD(in_serial, 0);
}


void setup_IMU()
{
  if(debugging)
    DEBUG.println("SETTING UP IMU");
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;

  if (!imu.begin())
  {
    if(debugging)
    {
      DEBUG.println("Failed to communicate with LSM9DS1.");
      DEBUG.println("Looping to infinity.");
      while (1);
    }
  }
  imu.begin();
}


void setup_TMP102(TMP102& in_TMP)
{
  if(debugging)
    DEBUG.println("SETTING UP TMP102");
  in_TMP.begin();
  in_TMP.setFault(0);
  in_TMP.setAlertPolarity(1);
  in_TMP.setAlertMode(0);
  in_TMP.setConversionRate(0);
  in_TMP.setExtendedMode(0);
  in_TMP.setHighTempC(HIGH_TEMP);
  in_TMP.setLowTempC(LOW_TEMP);
}


void loop() 
{
  incTime();
  
  int pidValue = PID(25, innerTemp1.readTempC());
  analogWrite(PIN_HEATER0,pidValue);

  
  sps30Dat.reset();
  run_SPS30();
  dubLog(LOG_PAR, sps30Dat);

  tmpDat.reset();
  run_TMP36();
  run_TMP102(innerTemp1);
  dubLog(LOG_TMP, tmpDat);
    
  atmDat.reset();
  run_SCD30();
  run_HIH4030();
  run_PRESSURE();

  imuDat.reset();
  run_IMU();
  dubLog(LOG_IMU, imuDat);

  if(debugging)
    bigOuts();
    
  DEBUG.println("Done...");
  
  delay(2010);
  heartBeat();
}


void heartBeat()
{
  digitalWrite(LED_BUILTIN, HIGH);
  delay(45);
  digitalWrite(LED_BUILTIN, LOW);
  delay(45);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(45);
  digitalWrite(LED_BUILTIN, LOW);
  delay(360);
}


void incTime()
{
  mil = millis();
  if((mil >= (lastMil + 1000)) || (mil <= (lastMil - 1000)))
  {
    if(mil >= (lastMil + 2000))
    {
      while (mil >= (lastMil + 1000))
      {
        second++;
        mil -= 1000;
      }
    }
    else
      second++;
    lastMil = millis();
    if(second >= 60)
    {
      while(second >= 60)
      {
        minute++;
        second -= 60;
      }
      if(minute >= 60)
      while(minute >= 60)
      {
        hour++;
        minute -= 60;
      }
    }
  }
}


void dubLog(int in_CMD)
{
  oLog_logCMD(LOGG1, in_CMD);
  //oLog_logCMD(LOGG2, in_CMD);
}


void dubLog(String in_file, String in_type, String in_string)
{
  oLog_append(LOGG1, in_file, in_type, in_string);
  //oLog_append(LOGG2, in_file, in_type, in_string);
}


template<typename type> void dubLog(String in_file, DataSet<type> in_set)
{
  oLog_append(LOGG1, in_file, in_set);
  //oLog_append(LOGG2, in_file, in_set);
  //in_set.reset();
}


void bigOuts()
{
  DEBUG.println("WOLFSAT RTOS");
  DEBUG.println();
  DEBUG.println(timeOuts());
  DEBUG.println(particulatesOuts());
  DEBUG.println(tmpOuts());
  DEBUG.println(imuOuts());
  DEBUG.println(atmOuts());

  int pos = 0;
  int lim = 8;
  while (pos < lim)
  {
    DEBUG.println();
    pos++;
  }
}


String timeOuts()
{
  String toRet = "Time : ";
  toRet.concat(hour);
  toRet.concat(":");
  toRet.concat(minute);
  toRet.concat(":");
  toRet.concat(second);
  toRet.concat(":");
  toRet.concat(mil);
  toRet.concat(" actual millis : ");
  toRet.concat(mil);
  toRet.concat(" actual lastMil : ");
  toRet.concat(lastMil);
  return toRet;
}


String particulatesOuts()
{
  String toRet = "Particulates : m1: ";
  toRet.concat(sps30Dat.get_data(0));
  toRet.concat(" m2: ");
  toRet.concat(sps30Dat.get_data(1));
  toRet.concat(" m4: ");
  toRet.concat(sps30Dat.get_data(2));
  toRet.concat(" m10: ");
  toRet.concat(sps30Dat.get_data(3));
  toRet.concat(" n0: ");
  toRet.concat(sps30Dat.get_data(4));
  toRet.concat(" n1: ");
  toRet.concat(sps30Dat.get_data(5));
  toRet.concat(" n2: ");
  toRet.concat(sps30Dat.get_data(6));
  toRet.concat(" n4: ");
  toRet.concat(sps30Dat.get_data(7));
  toRet.concat(" n10: ");
  toRet.concat(sps30Dat.get_data(8));
  toRet.concat(" size: ");
  toRet.concat(sps30Dat.get_data(9));
  return toRet;
}


String tmpOuts()
{
  String toRet = "Temperatures : Ext: ";
  toRet.concat(tmpDat.get_data(0));
  toRet.concat(" In1: ");
  toRet.concat(tmpDat.get_data(1));
  toRet.concat(" In2: ");
  toRet.concat(tmpDat.get_data(2));
  toRet.concat(" In3: ");
  toRet.concat(tmpDat.get_data(3));
  return toRet;
}


String imuOuts()
{
  String toRet = "Motions : Ax: ";
  toRet.concat(imuDat.get_data(0));
  toRet.concat(" Ay: ");
  toRet.concat(imuDat.get_data(1));
  toRet.concat(" Az: ");
  toRet.concat(imuDat.get_data(2));
  toRet.concat(" Rx: ");
  toRet.concat(imuDat.get_data(3));
  toRet.concat(" Ry: ");
  toRet.concat(imuDat.get_data(4));
  toRet.concat(" Rz: ");
  toRet.concat(imuDat.get_data(5));
  toRet.concat(" Mx: ");
  toRet.concat(imuDat.get_data(6));
  toRet.concat(" My: ");
  toRet.concat(imuDat.get_data(7));
  toRet.concat(" Mz: ");
  toRet.concat(imuDat.get_data(8));
  return toRet;
}


String atmOuts()
{
  String toRet = "Atmosphere : CO2: ";
  toRet.concat(atmDat.get_data(0));
  toRet.concat(" CO2 rh: ");
  toRet.concat(atmDat.get_data(1));
  toRet.concat(" CO2 Tmp: ");
  toRet.concat(atmDat.get_data(2));
  toRet.concat(" RH : ");
  toRet.concat(atmDat.get_data(3));
  toRet.concat(" Pressure: ");
  toRet.concat(atmDat.get_data(4));
  return toRet;
}


template<typename type> void outSet(DataSet<type>& in_set)
{
  DEBUG.println("DEBUG :: OUT SET");
  int lim = in_set.get_size();
  int pos = 0;
  if (lim <= pos)
  {
    DEBUG.print("DEBUG :: OUT SET FAILURE POS:");
    DEBUG.print(pos);
    DEBUG.print(" LIM: ");
    DEBUG.println(lim);
  }
  while(pos < lim)
  {
    delay(100);
    String toWrite = (String)in_set.get_data(pos);
    DEBUG.print((String)pos);
    DEBUG.print(" of ");
    DEBUG.print((String)lim);
    DEBUG.print(" ");    
    delay(20);
      
    DEBUG.println(toWrite);
    pos++;      
    DEBUG.flush();
  } 
  delay(3000);
}


void run_HIH4030()
{
  double val = hih4030.getSensorRH();
  atmDat.set_data(val);
  if(debugging)
  {
    DEBUG.println("HIH4030 :: DATA LOGGED");
  }
}


void run_SCD30()
{
  double co2 = 0;
  double locTemp = 0;
  double rh = 0;
  if (digitalRead(PIN_CO2RDY) == HIGH)
  {
    if(scd30.dataAvailable())
    {
      co2 = scd30.getCO2();
      locTemp = scd30.getTemperature();
      rh = scd30.getHumidity();
    }
    else if (debugging)
    {
      DEBUG.println("SCD30 :: DATA UNAVAILABLE");
    }    
  }
  else
    if(debugging)
      DEBUG.println("SCD30 :: DATA NOT READY");

  atmDat.set_data(co2);
  atmDat.set_data(locTemp);
  atmDat.set_data(rh);
}



void run_PRESSURE()
{
  double voltIn = analogRead(PIN_PRESSURE);
  double voltSup = 5;
  double voltActual = voltIn * 0.0049;
  double magicFactor = 1.2002924558587479935794542536116;
  double pressure = magicFactor *(((voltActual - (0.1 * voltSup)) * 15)
                    / (0.8 * voltSup));
  atmDat.set_data(pressure);
}


void run_TMP36()
{
  double reading = analogRead(PIN_EXTTMP);
  double voltage = reading * 5.0;
  voltage /= 1024.0;
  double tempC = (voltage - 0.5) * 100;
  fill_tmpDat(tempC);
}


void fill_tmpDat(double in_tempC)
{
  //tmpDat.set_data(in_voltage);
  tmpDat.set_data(in_tempC);
}


void run_TMP102(TMP102& in_TMP)
{
  in_TMP.wakeup();
  fill_tmpDat(in_TMP);
  in_TMP.sleep();
}

void fill_tmpDat(TMP102& in_TMP)
{
  tmpDat.set_data(in_TMP.readTempC());
}


void run_IMU()
{
  imu.readAccel();
  imu.readGyro();
  imu.readMag();
  fill_imuDat();
}


void fill_imuDat()
{
  imuDat.set_data(imu.calcAccel(imu.ax));
  imuDat.set_data(imu.calcAccel(imu.ay));
  imuDat.set_data(imu.calcAccel(imu.az));
  imuDat.set_data(imu.calcGyro(imu.gx));
  imuDat.set_data(imu.calcGyro(imu.gy));
  imuDat.set_data(imu.calcGyro(imu.gz));
  imuDat.set_data(imu.calcMag(imu.mx));
  imuDat.set_data(imu.calcMag(imu.my));
  imuDat.set_data(imu.calcMag(imu.mz));
}


bool run_SPS30()
{
  struct sps_values loc;
  uint8_t check, errCnt = 0;
  sps30.GetValues(&loc);
  fill_sps30Dat(loc);
}


void fill_sps30Dat(sps_values& in_data)
{
  sps30Dat.set_data(in_data.MassPM1);
  sps30Dat.set_data(in_data.MassPM2);
  sps30Dat.set_data(in_data.MassPM4);
  sps30Dat.set_data(in_data.MassPM10);
  sps30Dat.set_data(in_data.NumPM0);
  sps30Dat.set_data(in_data.NumPM1);
  sps30Dat.set_data(in_data.NumPM2);
  sps30Dat.set_data(in_data.NumPM4);
  sps30Dat.set_data(in_data.NumPM10);
  sps30Dat.set_data(in_data.PartSize);
  
}


// Open Log functions. Considers only one serial channel at a time.
void oLog_enterCMD(HardwareSerial& in_serial)
{
  in_serial.write(26);
  in_serial.write(26);
  in_serial.write(26);
  while(true)
    if(in_serial.available())
      if(in_serial.read() == '>')
        break;
}


void oLog_exitCMD(HardwareSerial& in_serial)
{
  while(true)
    if(in_serial.available())
      if(in_serial.read() == '<')
        break;
}


/*void oLog_append(HardwareSerial& in_serial, String in_file, String in_string)
{
  oLog_changeFile(in_serial, in_file);
  in_serial.println(in_string);
  delay(20);
  oLog_changeFile(in_serial, LOG_CMD);
//  if(debugging)
//  {
//    DEBUG.print("OLOG :: FILE APPENDED ");
//    DEBUG.print(in_file);
//    DEBUG.print(" ");
//    DEBUG.println(in_string);
//  }
}
*/


void oLog_append(HardwareSerial& in_serial, String in_file, String in_type, String in_header)
{
//  String theFile = in_file;
//  theFile.concat(in_type);
  oLog_changeFile(in_serial, in_file, in_type);
  in_serial.println(in_header);
  oLog_changeFile(in_serial, LOG_CMD, TYPE_TXT);
}

template<typename type> void oLog_append(HardwareSerial& in_serial, String in_file, DataSet<type> in_set)
{
  oLog_changeFile(in_serial, in_file, TYPE_CSV);
  String toLog = oLog_formatCSV(in_set);
  in_serial.println(toLog);
  oLog_changeFile(in_serial, LOG_CMD, TYPE_TXT);
}


template <typename type> String oLog_formatCSV(DataSet<type> in_set)
{
  int pos = 0;
  int lim = in_set.get_size();
  String toRet = "";
  while (pos < lim)
  {
    String loc = (String)in_set.get_data(pos);
    loc.concat(", ");
    toRet.concat(loc);
    pos++;
  }
  return toRet;
}

/*
template <typename type> void oLog_append(HardwareSerial& in_serial, String in_file, DataSet<type> in_set)
{
  oLog_changeFile(in_serial, in_file);
  int pos = 0;
  int lim = in_set.get_size();
  while (pos < lim)
  {
    in_serial.println((String)in_set.get_data(pos));
    delay(20);
    pos++;
  }
  oLog_changeFile(in_serial, LOG_CMD);
  
//  if(debugging)
//  {
//    DEBUG.print("OLOG :: FILE APPENDED ");
//    DEBUG.print(in_file);
//    DEBUG.println(" WITH DATASET");
//  }
}

*/


void oLog_changeFile(HardwareSerial& in_serial, String in_name, String in_type)
{
  String root = "append ";
  String fileType = in_type;
  String changeTo = root;
  changeTo.concat(in_name);
  changeTo.concat(fileType);
  oLog_enterCMD(in_serial);
  in_serial.println(changeTo);
  delay(10);
  oLog_exitCMD(in_serial);
  activeLog = in_name;
//  if (debugging)
//  {
//    DEBUG.print("OLOG :: ACTIVE LOG ");
//    DEBUG.println(activeLog);
//    delay(20);
//  }
}


void oLog_logCMD(HardwareSerial& in_serial, int in_CMD)
{
  String toLog = "CMD :: ";
  toLog.concat((String)cmdCount);
  toLog.concat(" - ");
  if(verboseDebug)
    toLog.concat(oLog_verboseSwitch(in_CMD));
  else
    toLog.concat((String)oLog_nonVerboseSwitch(in_CMD));
  if(activeLog != LOG_CMD)
  {
    oLog_changeFile(in_serial, LOG_CMD, TYPE_TXT);
    if (debugging)
    {
      DEBUG.println("OLOG :: NOT COMMAND LOG");
      delay(20);      
    }
      
  }
  in_serial.println(toLog);
  delay(20);
//  if (debugging)
//  {
//    DEBUG.print("OLOG :: ");
//    DEBUG.println(toLog);
//    delay(20);
//  }
}


String oLog_verboseSwitch(int in_CMD)
{
  String toRet = "UNDEFINED";
  switch(in_CMD)
  {
    case 0:
      toRet = "WOLFSAT BOOT";
      break;
    case 1:
      toRet = "DEBUG ACTIVE";
      break;
    case 2:
      toRet = "DEBUG INACTIVE";
      break;      
    case 3:
      toRet = "TMP SETUP";
      break;
    case 4:
      toRet = "IMU SETUP";
      break;
    case 5:
      toRet = "TMP36 SETUP";
      break;
    case 6:
      toRet = "SPS30 SETUP";
      break;
    case 7:
      toRet = "SCD30 SETUP";
      break;
    default:
      toRet = "UNKNOWN CMD";
      break;
  }
  return toRet;
}


byte oLog_nonVerboseSwitch(int in_CMD)
{
  byte toRet = (byte)in_CMD;
  return toRet;
}


// PID controller by Tyler Dow

// Copied and pasted to global vars section for consistency
//double oldp,oldD;
//double lastPIDrun=millis(); 

int PID(double target,double current){
  double p,i,d;
  long deltaTime=millis()-lastPIDrun;
  lastPIDrun=millis();
  
  p=(target-current);
  
  i+=(p/1000)*5;
  i=constrain(i,-25,25);
  d=(p-oldp)*deltaTime;
  d=d*0.1+oldD*0.9;
  

  return constrain(p+i+d,9,15);
}


