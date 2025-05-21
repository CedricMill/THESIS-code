#include <Ramp.h>
#include <Arduino.h>
#include "ODriveCAN.h"
#include "ODriveFlexCAN.hpp"
#include <math.h>
#include "HX711.h"

#define CAN_BAUDRATE 250000
#define ODRV2_NODE_ID 2
#define ODRV3_NODE_ID 3

struct ODriveStatus; // hack to prevent teensy compile error

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_intf;

bool setupCan() {
  can_intf.begin();
  can_intf.setBaudRate(CAN_BAUDRATE);
  can_intf.setMaxMB(16);
  can_intf.enableFIFO();
  can_intf.enableFIFOInterrupt();
  can_intf.onReceive(onCanMessage);
  return true;
}

// Instantiate ODrive objects
ODriveCAN odrv2(wrap_can_intf(can_intf), ODRV2_NODE_ID); // Standard CAN message ID for ODrive 2
ODriveCAN odrv3(wrap_can_intf(can_intf), ODRV3_NODE_ID); // Standard CAN message ID for ODrive 3
ODriveCAN* odrives[] = {&odrv2, &odrv3}; // Add both ODriveCAN instances

// User data for both ODrives
struct ODriveUserData {
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
};

// Keep application-specific user data for every ODrive
ODriveUserData odrv2_user_data; // Add for ODrive 2
ODriveUserData odrv3_user_data; // Add for ODrive 3

//**************************************MOTOR STATE VARIABLES*******************************************//
float position_motor1 = 0.0; // turns
float velocity_motor1 = 0.0; // turns/s

float position_motor2 = 0.0; // turns
float velocity_motor2 = 0.0; // turns/s

float current_motor1 = 0.0;
float current_motor2 = 0.0;

float electrical_power_motor1 = 0.0;
float electrical_power_motor2 = 0.0;

//**************************************JOINT STATE VARIABLES*******************************************//
float current_theta1 = 0;
float current_theta2 = 0;

float current_theta1_speed = 0;
float current_theta2_speed = 0;

float current_x = 0;
float current_z = 0;

float current_x_speed = 0;
float current_z_speed = 0;

//************************************MOTOR LIMIT VARIABLES*********************************************//
float max_motor_torque = 1.0; //Nm
float max_motor_current = 40.0;
float max_motor_rpm = 9999.0;

//*******************************FORCE IMPEDANCE CONTROLLER VARIABLES***********************************//
constexpr float L1 = 0.17f;     // heup‑knie‑lengte [m]
constexpr float L2 = 0.17f;     // knie‑voet‑lengte [m]

constexpr float gear1 = 10.0f;  // tandwielverhouding joint 1
constexpr float gear2 = 10.0f;  // tandwielverhouding joint 2

float TWO_Pi = 6.28318;
float TAU_MAX = 1.0;       // veiligheidslimiet [N m]

float stiffness = 600.0;
float dampening = 25;

float x_error = 0.0;
float z_error = 0.0;

float x_dot_error = 0.0;
float z_dot_error = 0.0;

int impedance_pos_reached = 0;


//**************************************HOMING VARIABLES************************************************//
float x_home = 0;   //30.03; //mm
float z_home = 340; //96.40; //mm

float motor1_home_rot;
float motor2_home_rot; 

//***************************************LOAD CELL VARIABLES********************************************//
const int LOADCELL1_DOUT_PIN = 0;
const int LOADCELL1_SCK_PIN = 1;

const int LOADCELL2_DOUT_PIN = 3;
const int LOADCELL2_SCK_PIN = 8;

float total_weight;

HX711 scale1;
HX711 scale2;

//****************************************BUTTON VARIABLES*********************************************//
const int GRN_Button = 6;
const int RED_Button = 5;
const int YELL_Button = 4;
const int E_stop = 7;

int GRN_state = 0;
int RED_state = 0;
int YELL_state = 0;
int E_stop_state = 0;

int GRN_pressed = 0;
int RED_pressed = 0;
int YELL_pressed = 0;

//***************************************KINEMATICS******************************************//
float path_x[10000];
float path_z[10000];

float distance_between_points = 5.0; //mm
int n_points;

int path_completed = 0;

int path_index = 0;

unsigned long last_increment_time = 0;

//**************************************MOVE ALONG PATH**************************************//
int pos_index = 0;

//****************************************GENERAL VARIABLES*********************************************//
float pi = 3.14159265;

int homed_Check;

unsigned long lastMeasureMillis = 0;

const unsigned long measureInterval = 200;

unsigned long lastSwitch = 0;
const unsigned long interval = 1000;  // 5 seconden in milliseconden
int mode = 0; 

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 30 && !Serial; ++i) {// Wait for up to 3 seconds for the serial port to be opened on the PC side.
    delay(100);
  }
  delay(1000);

  pinMode(GRN_Button, INPUT);
  pinMode(RED_Button, INPUT);
  pinMode(YELL_Button, INPUT);
  pinMode(E_stop, INPUT);

  odrv2.onFeedback(onFeedback, &odrv2_user_data); // Register callbacks for ODrive 2
  odrv2.onStatus(onHeartbeat, &odrv2_user_data);
  
  odrv3.onFeedback(onFeedback, &odrv3_user_data); // Register callbacks for ODrive 3
  odrv3.onStatus(onHeartbeat, &odrv3_user_data);

  if (!setupCan()) {// Configure and initialize the CAN bus interface.
    Serial.println("CAN failed to initialize: reset required");
    while (true);
  }

  odrv2.setLimits(max_motor_rpm, max_motor_current); //Set max rpm and max current
  odrv3.setLimits(max_motor_rpm, max_motor_current); //Set max rpm and max current

  homing();
  homed_Check = 1;

  delay(100);
  odrv2.setControllerMode(ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
  odrv3.setControllerMode(ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
  delay(100);
  odrv2.setControllerMode(ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
  odrv3.setControllerMode(ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
  delay(100);
  odrv2.setControllerMode(ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
  odrv3.setControllerMode(ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
  delay(800);
  //scales_calibration();
  // Serial.print("dampening, ");
  // Serial.print("stiffness, ");
  // Serial.print("z displacement, ");
  //Serial.println("force");
  Serial.print("time [ms],");
  Serial.print("torque motor 1 [Nm],");
  Serial.print("torque motor 2 [Nm],");
  Serial.print("z pos [m],");
  Serial.print("power motor 1 [W],");
  Serial.print("power motor 2 [W]");
  Serial.print("current motor 1 [A]");
  Serial.println("current motor 2 [A]");
  lastSwitch = millis();

}

void loop() {
  pumpEvents(can_intf);  // Handle incoming feedback CAN messages
  move_up_down();
  //medical_devices_demo_2(100, 10, 400, 20.0, 0.0, 0.0, 0.250);
  //medical_devices_demo_0(100, 10, 400, 22.5);
  //medical_devices_demo_3(600, 25);
  //buttons();
  
}