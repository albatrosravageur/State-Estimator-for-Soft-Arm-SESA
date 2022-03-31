#include <Wire.h>
#include <ros.h>
#include <math.h>
#include <ros/time.h>
#include <sesa/quat.h>
#include <sesa/acc.h>

#include <std_msgs/Bool.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#include "TCA9548A.h"
#include "LiquidCrystal.h"
#include "button.h"

#define N_BNO_MAX 8     // FOR THE PROJECT 2
#define MUX_0_ADDR 0x70 // Using TCA9548A

#define BUTTON_CALIB_INPUT A1
#define BUTTON_CALIB_OUTPUT A2
#define DELAY_BETWEEN_SIGNALS_MS 50.0

// BNO*** RELATED
int n_bno;      // # of bno*** sensors to measure
int stream_acc; // Stream only orientation or orientation + acc
int fast_mag_calib;
TCA9548A I2CMux(MUX_0_ADDR);    // Address can be passed into the constructor
Adafruit_BNO055 bno[N_BNO_MAX]; // Handlers of the bno*** sensors

// ROS RELATED
sesa::quat q;                                         // Variable to publish quaternions
sesa::acc a;                                          // Variable to publish accelerations
ros::NodeHandle_<ArduinoHardware, 3, 3, 512, 512> nh; // Node handler for ROS
ros::Publisher quat_pub("/quat_meas", &q);            // ROS Publisher
ros::Publisher acc_pub("/acc_meas", &a);              // ROS Publisher
bool use_saved_calib;

// LCD or button related
const int rs = 53, en = 49, d4 = 47, d5 = 45, d6 = 43, d7 = 41;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
Button but_cal;
bool flag_was_disconnected = 0;

void setup_sensors()
// Initiate the connection with the bno*** sensors and configurates the axis map
{
  // Show what you do
  lcd.clear();
  lcd.print("Begin IMU: 0/  ");
  lcd.setCursor(13, 0);
  lcd.print(n_bno);
  int count = 0;
  // Read all IMU, check
  for (uint8_t i = 0; i < n_bno; i++)
  {
    lcd.setCursor(11, 0);
    lcd.print(String(i + 1)); // print the text to the lcd
    bno[i] = Adafruit_BNO055(55, 0x28);
    I2CMux.openChannel(i);
    count += bno[i].begin();
    bno[i].setAxisRemap(bno[i].REMAP_CONFIG_P1);
    bno[i].setAxisSign(bno[i].REMAP_SIGN_P1);
    if (fast_mag_calib)
    {
      bno[i].setMode(bno[i].OPERATION_MODE_NDOF);
    }
    else
    {
      bno[i].setMode(bno[i].OPERATION_MODE_NDOF_FMC_OFF);
    }
    I2CMux.closeChannel(i);
  }
  lcd.clear();
}

void stream_acc_bno()
{
  sensors_event_t accelerometerData;
  for (uint8_t i = 0; i < n_bno; i++)
  {
    I2CMux.openChannel(i);
    bno[i].getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    I2CMux.closeChannel(i);
    a.ID = i + 1;
    // BASIS CHANGE HERE
    a.x = accelerometerData.acceleration.x;
    a.y = accelerometerData.acceleration.z;
    a.z = accelerometerData.acceleration.y;

    // Publish to ROS
    acc_pub.publish(&a);
    // This is recommended by ROS tutorials to keep a good serial connection
    nh.spinOnce();
  }
}

void stream_quat_bno()
{
  bool flag_disconnected = 0;

  // The reading and publishing ar emade individually for each IMU
  for (uint8_t i = 0; i < n_bno; i++)
  {
    // This default state won't be read correctly by ROS since the norm isn't 1
    q.w = 0.0;
    q.x = 0.0;
    q.y = 0.0;
    q.z = 0.0;

    // Reads and publishes the state for each IMU seperately
    I2CMux.openChannel(i);
    imu::Quaternion quat = bno[i].getQuat();
    q.ID = i + 1; // Identifies which sensor we are talking about
    q.w = float(quat.w());
    q.x = float(quat.x());
    q.y = float(quat.y());
    q.z = float(quat.z());

    if ((abs(q.w) + abs(q.x) + abs(q.y) + abs(q.z)) < 0.0001)
    {
      if (!flag_was_disconnected)
      {
        flag_disconnected = 1;
        flag_was_disconnected = 1;
        lcd.clear();
        lcd.print("IMU ");
        lcd.print(i + 1);
        lcd.print(" disconnect");
      }
      bno[i] = Adafruit_BNO055(55, 0x28);
      if (bno[i].begin())
      {
        lcd.clear();
        lcd.print("IMU ");
        lcd.print(String(i + 1));
        lcd.print(" is back!");
      }
    }
    I2CMux.closeChannel(i);

    // Publish to ROS
    quat_pub.publish(&q);
    // This is recommended by ROS tutorials to keep a good serial connection
    nh.spinOnce();
  }
  if ((!flag_disconnected) && flag_was_disconnected)
  {
    flag_was_disconnected = 0;
    lcd.setCursor(0, 1);
    lcd.print("IMUs reading ok!");
  }
}

void load_calib_to_sensors()
{
  lcd.clear();
  lcd.print("Calibration IMU ");
  lcd.setCursor(0, 1);
  lcd.print("Loading:0/");
  lcd.print(n_bno);

  for (uint8_t i = 0; i < n_bno; i++)
  {
    // Read calibration
    adafruit_bno055_offsets_t calibData;
    int nb_params = 11;
    String path = String("/calib_saved/imu_") + String(i + 1);
    String spots[nb_params] = {"/acc/x", "/acc/y", "/acc/z", "/mag/x", "/mag/y",
                               "/mag/z", "/gyro/x", "/gyro/y", "/gyro/z", "/rad/acc", "/rad/mag"};
    int params[nb_params];
    for (int p = 0; p < nb_params; p++)
    {
      String this_param_path = path + spots[p];
      char charBuf[50];
      this_param_path.toCharArray(charBuf, 50);
      nh.getParam(charBuf, &params[p]);
    }

    calibData.accel_offset_x = int16_t(params[0]);
    calibData.accel_offset_y = int16_t(params[1]);
    calibData.accel_offset_z = int16_t(params[2]);
    calibData.mag_offset_x = int16_t(params[3]);
    calibData.mag_offset_y = int16_t(params[4]);
    calibData.mag_offset_z = int16_t(params[5]);
    calibData.gyro_offset_x = int16_t(params[6]);
    calibData.gyro_offset_y = int16_t(params[7]);
    calibData.gyro_offset_z = int16_t(params[8]);
    calibData.accel_radius = int16_t(params[9]);
    calibData.mag_radius = int16_t(params[10]);

    // Push it to the sensor
    I2CMux.openChannel(i);
    bno[i].setSensorOffsets(calibData);
    I2CMux.closeChannel(i);

    lcd.setCursor(8, 1);
    lcd.print(i+1);
    delay(100);
  }
}

void disp_calib()
{
  lcd.clear();
  lcd.print("Calibration IMU");
  lcd.setCursor(0, 1);

  for (uint8_t i = 0; i < n_bno; i++)
  {
    I2CMux.openChannel(i);
    uint8_t syst = 0, gyro = 0, accel = 0, mag = 0;
    bno[i].getCalibration(&syst, &gyro, &accel, &mag);
    lcd.print(syst);
    lcd.print(" ");
    I2CMux.closeChannel(i);
  }
}

void setup()
{
  // LCD Setup
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);     // set cursor to top left corner
  lcd.print("Booting..."); // print the text to the lcd

  // Button setup
  but_cal.begin(BUTTON_CALIB_INPUT, BUTTON_CALIB_OUTPUT);

  // ROS Setup
  nh.getHardware()->setBaud(115200); // Fastest baud for MEGA2560
  nh.initNode();
  while (!nh.connected())
  {
    nh.spinOnce();
    lcd.setCursor(0, 1);           // set cursor to top left corner
    lcd.print("Connects to ROS."); // print the text to the lcd
  }
  nh.advertise(quat_pub);
  nh.advertise(acc_pub);

  nh.spinOnce();

  // Sensors Setup
  nh.getParam("/imus/amount", &n_bno);
  nh.getParam("/my_rosbag/to_record/accelerometers", &stream_acc);
  nh.getParam("/calib/use_saved", &use_saved_calib);
  nh.getParam("/calib/use_fast_mag", &fast_mag_calib);

  I2CMux.begin(Wire);
  I2CMux.closeAll(); // Set a base state which we know (also the default state on power on)
  setup_sensors();
  if (use_saved_calib)
  {
    load_calib_to_sensors();
  }
  lcd.print("IMUs reading ok!");
}

void loop()
{
  unsigned long tStart = micros();

  nh.spinOnce();

  stream_quat_bno();

  if (stream_acc)
  {
    stream_acc_bno();
  }

  // Display calibration on LCD?
  but_cal.read();
  if (but_cal._state)
  {
    disp_calib();
  }

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
    delay(50);
    // poll until the next sample is ready
  }
}