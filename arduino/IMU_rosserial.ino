/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>

#include <ros.h>
#include <imu_read/imu_read.h>

ros::NodeHandle  nh;

imu_read::imu_read imu_msg;

LSM9DS1 imu;

ros::Publisher p("imu_read", &imu_msg);

char hello[13] = "hello world!";

// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M  0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW

#define PRINT_SPEED 250 // 250 ms between prints
static unsigned long lastPrint = 0; // Keep track of print time

#define DECLINATION -8.58 // Declination (degrees) in Boulder, CO.

void setup()
{
  Serial.begin(57600);
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;

  if (!imu.begin())
  {
    Serial.println("Failed to communicate with IMU. Check wiring.");
    while(1);
  }
  nh.initNode();
  nh.advertise(p);
}

void loop()
{
  imu_msg.header.stamp = nh.now();
  if ( imu.gyroAvailable() )
  {
    imu.readGyro();
  }
  if ( imu.magAvailable() )
  {
    imu.readMag();
  }
    if ( imu.accelAvailable() )
  {
    imu.readAccel();
  }
  if ((lastPrint + PRINT_SPEED) < millis())
  {    
  imu_msg.gx = imu.calcGyro(imu.gx);
  imu_msg.gy = imu.calcGyro(imu.gy);
  imu_msg.gz = imu.calcGyro(imu.gz);
  imu_msg.mx = imu.calcMag(imu.mx);
  imu_msg.my = imu.calcMag(imu.my);
  imu_msg.mz = imu.calcMag(imu.mz);
  imu_msg.ax = imu.calcAccel(imu.ax);
  imu_msg.ay = imu.calcAccel(imu.ay);
  imu_msg.az = imu.calcAccel(imu.az);
  
  p.publish( &imu_msg );
  lastPrint = millis(); // Update lastPrint time
  }
  nh.spinOnce();
}
