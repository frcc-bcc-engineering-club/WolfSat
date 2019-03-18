#include <DataSet.h>
#include <sensirion_arch_config.h>
#include <sensirion_shdlc.h>
#include <sensirion_uart.h>



#include <sps30.h>
#include <unistd.h>
/*
 *  In Memory of Randy
 */


#include <SparkFunTMP102.h>

#include <LSM9DS1_Registers.h>
#include <LSM9DS1_Types.h>
#include <SparkFunLSM9DS1.h>

#include <SparkFun_Qwiic_OpenLog_Arduino_Library.h>


#define LSM9DS1_M   0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW

#define LIM_IMU 9
#define LIM_TMP 1

#define HIGH_TEMP 50
#define LOW_TEMP 49

DataSet<double> imuDat;
DataSet<double> tmp1Dat;

LSM9DS1 imu;
TMP102 innerTemp1(0x48);

void setup() 
{
  Serial.begin(9600);
  Serial.println("WolfSat RTOS");
  setup_IMU();
  setup_TMP(innerTemp1, tmp1Dat);
}


void setup_IMU()
{
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;

  if (!imu.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Looping to infinity.");
    while (1);
  }
  
  imuDat = DataSet<double>(LIM_IMU);
  
  imu.begin();
}


template <typename type> void setup_TMP(TMP102& in_TMP, DataSet<type>& in_set)
{
  in_TMP.begin();
  in_TMP.setFault(0);
  in_TMP.setAlertPolarity(1);
  in_TMP.setAlertMode(0);
  in_TMP.setConversionRate(0);
  in_TMP.setExtendedMode(0);
  in_TMP.setHighTempC(HIGH_TEMP);
  in_TMP.setLowTempC(LOW_TEMP);
  in_set = DataSet<double>(LIM_TMP);
}


void loop() 
{
  run_imu();
  outSet(imuDat);
  run_TMP(innerTemp1, tmp1Dat);
  outSet(tmp1Dat);
  delay(1000);
  
  Serial.println("Done...");
}

template<typename type> void outSet(DataSet<type>& in_set)
{
  int lim = in_set.get_size();
  int pos = 0;
  while(pos < lim)
  {
    String toWrite = (String)in_set.get_data(pos);
    pos++;
    #if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    Serial3.println(toWrite);
    #else
    Serial.println(toWrite);
    #endif
  }
}


template <typename type> void run_TMP(TMP102& in_TMP, DataSet<type>& in_set)
{
  in_TMP.wakeup();
  fill_tmpDat(in_TMP, in_set);
  in_TMP.sleep();
}

template<typename type> void fill_tmpDat(TMP102& in_TMP, DataSet<type>& in_set)
{
  in_set.set_data(in_TMP.readTempC());
}


void run_imu()
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



