#include <Wire.h>
#include <ros.h>
#include <math.h>
#include <ros/time.h>
#include <sesa/calib.h>

#include <std_msgs/Bool.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#include "TCA9548A.h"
#include "LiquidCrystal.h"
#include "button.h"


#define BAUD_RATE 115200



#define N_IMU_MAX 8     // CAN BE CHANGED
#define MUX_0_ADDR 0x70 // Using TCA9548A
#define DELAY_BETWEEN_SIGNALS_MS 500.0

// BNO*** RELATED
int n_imu;                                // # of bno*** sensors to measure
TCA9548A I2CMux(MUX_0_ADDR);              // Address can be passed into the constructor
Adafruit_BNO055 bno[N_IMU_MAX];           // Handlers of the bno*** sensors

// ROS RELATED
ros::NodeHandle_<ArduinoHardware, 3, 3, 2048, 2048> nh; // Node handler for ROS
sesa::calib c;                                 // Variable to publish calibration
ros::Publisher calib_pub("/calib_meas", &c);            // ROS Publisher

// LCD or button related
const int rs = 53, en = 49, d4 = 47, d5 = 45, d6 = 43, d7 = 41;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
bool flag_was_disconnected = 0;


//#TODO #6 see param /calib/id

void setup_sensors()
// Initiate the connection with the bno*** sensors and configurates the axis map
{
  // Show what you do
  lcd.clear();
  lcd.print("Begin IMU: 0/  ");
  lcd.setCursor(13, 0);
  lcd.print(n_imu);
  // Read all IMU, check
  for (uint8_t i = 0; i < n_imu; i++)
  {
    lcd.setCursor(11, 0);
    lcd.print(String(i + 1)); // print the text to the lcd
    bno[i] = Adafruit_BNO055(55, 0x28);
    I2CMux.openChannel(i);
    bno[i].begin();
    bno[i].setAxisRemap(bno[i].REMAP_CONFIG_P1);
    bno[i].setAxisSign(bno[i].REMAP_SIGN_P1);
    I2CMux.closeChannel(i);
  }
  lcd.clear();
}

bool getAndStoreSensorOffsets(int id)
{
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno[id].getCalibration(&system, &gyro, &accel, &mag);
    c.ID = id+1;
    c.sys = system;
    c.acc = accel;
    c.gyro= gyro;
    c.mag = mag;

    adafruit_bno055_offsets_t calibData;
    bool can_read;
    can_read = bno[id].getSensorOffsets(calibData);
    c.off_accX = calibData.accel_offset_x;
    c.off_accY = calibData.accel_offset_y;
    c.off_accZ = calibData.accel_offset_z;
    c.off_magX = calibData.mag_offset_x;
    c.off_magY = calibData.mag_offset_y;
    c.off_magZ = calibData.mag_offset_z;
    c.off_gyroX = calibData.gyro_offset_x;
    c.off_gyroY = calibData.gyro_offset_y;
    c.off_gyroZ = calibData.gyro_offset_z;
    c.rad_acc = calibData.accel_radius;
    c.rad_mag = calibData.mag_radius;
    return can_read;
}

void stream_calib()
{
  bool flag_disconnected = 0;

  // The reading and publishing are made individually for each IMU
  for (uint8_t i = 0; i < n_imu; i++)
  {
    // Reads and publishes the state for each IMU seperately
    I2CMux.openChannel(i);

    if (!getAndStoreSensorOffsets(i))
    {
      if (!flag_was_disconnected)
      {
        flag_disconnected = 1;
        flag_was_disconnected = 1;
        lcd.clear();
        lcd.print("IMU ");
        lcd.print(i+1);
        lcd.print(" disconnect");
      }
      bno[i] = Adafruit_BNO055(55, 0x28);
      if (bno[i].begin())
      {
        lcd.clear();
        lcd.print("IMU ");
        lcd.print(String(i+1));
        lcd.print(" is back!");
      }
    }
    I2CMux.closeChannel(i);

    // Publish to ROS
    calib_pub.publish(&c);
    // This is recommended by ROS tutorials to keep a good serial connection
    nh.spinOnce();
  }
  if ((!flag_disconnected) && flag_was_disconnected)
  {
    flag_was_disconnected = 0;
    lcd.setCursor(0, 1);
    lcd.print("IMUs reading    calibration ok!");
  }
}

void setup()
{
  // LCD Setup
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);     // set cursor to top left corner
  lcd.print("Booting..."); // print the text to the lcd

  // ROS Setup
  nh.getHardware()->setBaud(BAUD_RATE); // Fastest baud for MEGA2560
  nh.initNode();
  while (!nh.connected())
  {
    nh.spinOnce();
    lcd.setCursor(0, 1);           // set cursor to top left corner
    lcd.print("Connects to ROS."); // print the text to the lcd
  }
  nh.advertise(calib_pub);
  nh.spinOnce();

  // Sensors Setup
  nh.getParam("/imus/amount", &n_imu);

  I2CMux.begin(Wire);
  I2CMux.closeAll(); // Set a base state which we know (also the default state on power on)
  setup_sensors();
  lcd.print("IMUs reading ok!");
}

void loop()
{
  unsigned long tStart = micros();
  nh.spinOnce();
  stream_calib();

  if (!nh.connected())
  {
    while (!nh.connected())
    { 
      flag_was_disconnected = 1;
      lcd.clear();
      lcd.print("Reconnect to ROS");
      nh.spinOnce();
      delay(1000);
    }
  }

  while ((micros() - tStart) < (DELAY_BETWEEN_SIGNALS_MS * 1000.0))
  {
    nh.spinOnce();
    delay(5);
    // poll until the next sample is ready
  }
}
