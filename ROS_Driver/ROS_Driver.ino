#include <ArduinoJson.h>
StaticJsonDocument<256> jsonCmdReceive;
StaticJsonDocument<256> jsonInfoSend;
StaticJsonDocument<1024> jsonInfoHttp;

#include <SCServo.h>
#include <nvs_flash.h>
#include <esp_system.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <WebServer.h>
#include <esp_now.h>
#include <nvs_flash.h>
#include <Adafruit_SSD1306.h>
#include <INA219_WE.h>
#include <ESP32Encoder.h>
#include <PID_v2.h>
#include <SimpleKalmanFilter.h>
#include <math.h>
#include "ICM_20948.h"

// functions for barrery info.
#include "battery_ctrl.h"

// config for ugv.
#include "ugv_config.h"

// functions for oled.
#include "oled_ctrl.h"

// functions for the leds of UGV.
#include "ugv_led_ctrl.h"

// functions for RoArm-M2 ctrl.
#include "RoArm-M2_module.h"

// functions for gimbal ctrl.
#include "gimbal_module.h"

// define json cmd.
#include "json_cmd.h"

// functions for IMU ctrl.
#include "IMU_ctrl.h"

// functions for movtion ctrl. 
#include "movtion_module.h"

// functions for editing the files in flash.
#include "files_ctrl.h"

// advance functions for ugv ctrl.
#include "ugv_advance.h"

// functions for wifi ctrl.
#include "wifi_ctrl.h"

// functions for esp-now.
#include "esp_now_ctrl.h"

// functions for uart json ctrl.
#include "uart_ctrl.h"

// functions for http & web server.
#include "http_server.h"


void moduleType_RoArmM2() {
  unsigned long curr_time = millis();
  if (curr_time - prev_time >= 10){
    constantHandle();
    prev_time = curr_time;
  }

  RoArmM2_getPosByServoFeedback();
  
  // esp-now flow ctrl as a flow-leader.
  switch(espNowMode) {
  case 1: espNowGroupDevsFlowCtrl();break;
  case 2: espNowSingleDevFlowCtrl();break;
  }

  if (InfoPrint == 2) {
    RoArmM2_infoFeedback();
  }
}


void moduleType_Gimbal() {
  getGimbalFeedback();
  gimbalSteady(steadyGoalY);
}


void setup() {
  Serial.begin(115200);
  while(!Serial) {}
  Wire.begin(S_SDA, S_SCL, 400000);

  bool initialized = false;
  while (!initialized) {
    myICM.begin(Wire, AD0_VAL);
    Serial.print(F("Initialization of the sensor returned: "));
    Serial.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok) {
      Serial.println(F("Trying again..."));
      delay(500);
    }
    else {
      initialized = true;
    }
  }

  Serial.println(F("Device connected!"));

  // --- Direct register read (getAGMT), NOT the DMP FIFO ---
  // The DMP FIFO overflows on this board because the main loop is heavy (WiFi
  // server.handleClient(), servos, OLED, PID). An overflowed FIFO returns
  // misaligned packets, so accel bytes leak into the gyro fields -> the gyro reads
  // tens of rad/s of garbage while the robot is still (~40-97% of frames bad).
  // Reading the sensor registers directly can't misalign. We set the full-scale
  // ranges so the raw counts still match the ROS driver's scaling:
  //   accel: /8192 in the driver  => +/-4g  (8192 LSB/g)
  //   gyro:  /16.4 in the driver  => +/-2000 dps (16.384 LSB/dps)
  bool success = true;
  myICM.lowPower(false);
  success &= (myICM.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous) == ICM_20948_Stat_Ok);
  ICM_20948_fss_t myFSS;
  myFSS.a = gpm4;      // +/-4g   -> matches driver /8192
  myFSS.g = dps2000;   // +/-2000 dps -> matches driver /16.4
  success &= (myICM.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS) == ICM_20948_Stat_Ok);
  success &= (myICM.startupMagnetometer() == ICM_20948_Stat_Ok);  // AK09916 for mx/my/mz

  if (success)
  {
    Serial.println(F("IMU direct-read configured (DMP FIFO bypassed)."));
  }
  else
  {
    Serial.println(F("IMU config failed!"));
  }

  ina219_init();
  inaDataUpdate();

  // set mainType & moduleType.
  // mainType: 1.RaspRover, 2.UGV Rover, 3.UGV Beast
  // moduleType: 0.Null, 1.RoArm, 2.PT
  mm_settings(mainType, moduleType);

  init_oled();
  screenLine_0 = "ugv_base_ros.git";
  screenLine_1 = "version: 0.96";
  screenLine_2 = "starting...";
  screenLine_3 = "";
  oled_update();

  delay(1200);

  // functions for IMU.
  // imu_init();

  // functions for the leds on ugv.
  led_pin_init();

  // init the littleFS funcs in files_ctrl.h
  screenLine_2 = screenLine_3;
  screenLine_3 = "Initialize LittleFS";
  oled_update();
  if(InfoPrint == 1){Serial.println("Initialize LittleFS for Flash files ctrl.");}
  initFS();

  // init the funcs in switch_module.h
  screenLine_2 = screenLine_3;
  screenLine_3 = "Initialize 12V-switch ctrl";
  oled_update();
  if(InfoPrint == 1){Serial.println("Initialize the pins used for 12V-switch ctrl.");}
  movtionPinInit();

  // servos power up
  screenLine_2 = screenLine_3;
  screenLine_3 = "Power up the servos";
  oled_update();
  if(InfoPrint == 1){Serial.println("Power up the servos.");}
  delay(500);
  
  // init servo ctrl functions.
  screenLine_2 = screenLine_3;
  screenLine_3 = "ServoCtrl init UART2TTL...";
  oled_update();
  if(InfoPrint == 1){Serial.println("ServoCtrl init UART2TTL...");}
  RoArmM2_servoInit();

  // check the status of the servos.
  screenLine_2 = screenLine_3;
  screenLine_3 = "Bus servos status check...";
  oled_update();
  if(InfoPrint == 1){Serial.println("Bus servos status check...");}
  RoArmM2_initCheck(false);

  if(InfoPrint == 1 && RoArmM2_initCheckSucceed){
    Serial.println("All bus servos status checked.");
  }
  if(RoArmM2_initCheckSucceed) {
    screenLine_2 = "Bus servos: succeed";
  } else {
    screenLine_2 = "Bus servos: " + 
    servoFeedback[BASE_SERVO_ID - 11].status +
    servoFeedback[SHOULDER_DRIVING_SERVO_ID - 11].status +
    servoFeedback[SHOULDER_DRIVEN_SERVO_ID - 11].status +
    servoFeedback[ELBOW_SERVO_ID - 11].status +
    servoFeedback[GRIPPER_SERVO_ID - 11].status;
  }
  screenLine_3 = ">>> Moving to init pos...";
  oled_update();
  RoArmM2_resetPID();
  RoArmM2_moveInit();

  screenLine_3 = "Reset joint torque to ST_TORQUE_MAX";
  oled_update();
  if(InfoPrint == 1){Serial.println("Reset joint torque to ST_TORQUE_MAX.");}
  RoArmM2_dynamicAdaptation(0, ST_TORQUE_MAX, ST_TORQUE_MAX, ST_TORQUE_MAX, ST_TORQUE_MAX);

  screenLine_3 = "WiFi init";
  oled_update();
  if(InfoPrint == 1){Serial.println("WiFi init.");}
  initWifi();

  screenLine_3 = "http & web init";
  oled_update();
  if(InfoPrint == 1){Serial.println("http & web init.");}
  initHttpWebServer();

  screenLine_3 = "ESP-NOW init";
  oled_update();
  if(InfoPrint == 1){Serial.println("ESP-NOW init.");}
  initEspNow();

  screenLine_3 = "IMU Calibrating";
  oled_update();
  if(InfoPrint == 1){Serial.println("IMU Calibrating");}
  imuCalibration();

  screenLine_3 = "UGV started";
  oled_update();
  if(InfoPrint == 1){Serial.println("UGV started.");}

  getThisDevMacAddress();

  updateOledWifiInfo();

  initEncoders();

  pidControllerInit();

  screenLine_2 = String("MAC:") + macToString(thisDevMac);
  oled_update();

  led_pwm_ctrl(0, 0);

  if(InfoPrint == 1){Serial.println("Application initialization settings.");}
  createMission("boot", "these cmds run automatically at boot.");
  missionPlay("boot", 1);
}


void loop() {
  // Read accel/gyro/mag straight from the sensor registers (no DMP FIFO, so no
  // overflow/misalignment). getAGMT() returns raw counts at the FSR set in setup()
  // (+/-4g, +/-2000 dps), so the ROS driver's /8192 and /16.4 scaling still holds.
  if (myICM.dataReady()) {
    myICM.getAGMT();
    ax = myICM.agmt.acc.axes.x;
    ay = myICM.agmt.acc.axes.y;
    az = myICM.agmt.acc.axes.z;
    gx = myICM.agmt.gyr.axes.x;
    gy = myICM.agmt.gyr.axes.y;
    gz = myICM.agmt.gyr.axes.z;
    mx = myICM.agmt.mag.axes.x;
    my = myICM.agmt.mag.axes.y;
    mz = myICM.agmt.mag.axes.z;
  }

  serialCtrl();
  server.handleClient();

  // read and compute the info of joints.
  switch (moduleType) {
  case 1: moduleType_RoArmM2();break;
  case 2: moduleType_Gimbal();break;
  }

  // recv esp-now json cmd.
  if(runNewJsonCmd) {
    jsonCmdReceiveHandler();
    jsonCmdReceive.clear();
    runNewJsonCmd = false;
  }

  getLeftSpeed();

  LeftPidControllerCompute();
  
  getRightSpeed();
  
  RightPidControllerCompute();
  
  oledInfoUpdate();

  if (baseFeedbackFlow) {
    baseInfoFeedback();
  }

  heartBeatCtrl();
}
