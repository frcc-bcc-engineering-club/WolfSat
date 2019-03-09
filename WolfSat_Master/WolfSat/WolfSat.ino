#include <LSM9DS1_Registers.h>
#include <LSM9DS1_Types.h>
#include <SparkFunLSM9DS1.h>

#include <SparkFun_Qwiic_OpenLog_Arduino_Library.h>

#include <DataSet.h>

#define LSM9DS1_M   0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW

#define LIM_IMU 9

DataSet<double> imuDat;

LSM9DS1 imu;

void setup() 
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
  imuDat = DataSet<double>(9);
  Serial.begin(9600);
  Serial.println("Hello");
  imu.begin();
}


void loop() 
{
  run_imu();
  outSet(imuDat);
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
    Serial.println(toWrite);
  }
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


void run_imu()
{
  imu.readAccel();
  imu.readGyro();
  imu.readMag();
  fill_imuDat();
}

