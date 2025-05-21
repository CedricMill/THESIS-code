#include "TrajectorySpec.h"
#include <Arduino.h>
#include "ODriveCAN.h"
#include "ODriveFlexCAN.hpp"
#include <math.h>
#include <AutoLQR.h>

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


//***************************************EDIT VARIABLES BELOW***********************************************//
float x_base = 60;
float z_base = 250;

float max_motor_torque = 0.2;
float max_motor_current = 40.0;
float max_motor_rpm = 9999.0;

//**************************************JOINT STATE VARIABLES*******************************************//
float current_f1 = 0;
float current_f2 = 0;

float current_f1_speed = 0;
float current_f2_speed = 0;

float current_x = 0;
float current_z = 0;

float current_x_speed = 0;
float current_z_speed = 0;

//***************************************************PID**************************************************//
int PID_pos_reached = 0;
int f_index = 0;


//**************************************TORQUE CONTROL VARIABLES: EDIT********************************************//
float Kp2 = 0.3; //Periode = 0.4 (=pcr) als kp2 = 0.5 (=kcr) -> kp2 = 0.6*kcr = 0.3
float Ki2 = 0.005; //ki = 2*kp/pcr = 2*0.3/0.4 = 1.5
float Kd2 = 1;

float Kp3 = 0.2;
float Ki3 = 0.0005;
float Kd3 = 0.0005;

float dt = 0.001;

const int t_step = 100;    // Aantal stappen


//*************************************PATH PLANNING*****************************************//
const float dist_between_points = 0.001; //m

const float l1_val = 0.170;

const float x_0 = 0.0;
const float z_0 = 0.340;

const float x_1 = 0.010;
const float z_1 = 0.180;

const float x_2 = 0.050;
const float z_2 = 0.250;

const float dx = x_1 - x_0;
const float dz = z_1 - z_0;

const float diameter = sqrt(dx * dx + dz * dz);

const float radius = diameter / 2.0;

const float halve_omtrek = PI * radius;

const int n_points_0 = (int)((fabs(halve_omtrek)) / dist_between_points + 0.5);
const int n_points_1 = (int)((fabs(sqrt( (x_2 - x_1) * (x_2 - x_1) + (z_2 - z_1) * (z_2 - z_1)))) / dist_between_points + 0.5);
const int n_points_2 = (int)((fabs(sqrt( (x_0 - x_2) * (x_0 - x_2) + (z_0 - z_2) * (z_0 - z_2)))) / dist_between_points + 0.5);

//int traj2_npoints = 15;
//int traj3_npoints = 30;

// Definieer segmenten
SegmentSpec specs[] = {
                      {x_0, z_0, x_1, z_1,  n_points_0, CIRCULAR}, //{x_start, z_start, x_end, z_end, number of points, trajectory type}
                      {x_1, z_1, x_2, z_2, n_points_1, LINEAR}, //{x_start, z_start, x_end, z_end, number of points, trajectory type}
                      {x_2, z_2, x_0, z_0, n_points_2, LINEAR} //{x_start, z_start, x_end, z_end, number of points, trajectory type}
                      };
const int N_SEGS = sizeof(specs) / sizeof(specs[0]);

const int list_length = n_points_0 + n_points_1 + n_points_2;

const int NUM_SEGMENTS = 3;

float full_px[list_length];
float full_pz[list_length];

float f1_list[list_length];
float f2_list[list_length];

float joint1_angles[list_length];
float joint2_angles[list_length];

int total_pts;

//***************************************LQR VARIABLES***********************************************//
const int STATE_SIZE = 4;
const int CONTROL_SIZE = 2;

float K11 = 0.002188;
float K12 = 0.000171;
float K13 = 1.554798;
float K14 = -0.071439;
float K21 = 0.000168;
float K22 = 0.001541;
float K23 = -0.064951;
float K24 = 1.708565;




float K_LQR[CONTROL_SIZE][STATE_SIZE] = {
                                        { K11, K12,  K13, K14 },
                                        { K21,  K22, K23, K24 }
                                        };                                 
//**************************************HOME VARIABLES***************************************************//
float x_home = 30.03; //mm
float z_home = 96.40; //mm

float f1_home_angle = 0; //(rad)
float f2_home_angle = 0; //(rad)

float pi = 3.14159265;

int pos_reached = 0;
int homingCheck = 0;
int impedance_position_reached = 0; 

float f1_home_rot;
float f2_home_rot;

float offsetZ1 = 0;
float offsetZ2 = 0;

float theta_rot;
float beta_rot;

int IKcounter = 0;

int prgrm_Start = 0;
int prgrm_Stop = 1;

int homed_Check = 0;
int run_Program = 0;

float position_motor1 = 0.0; // turns
float velocity_motor1 = 0.0; // turns/s
float position_motor2 = 0.0; // turns
float velocity_motor2 = 0.0; // turns/s

int test_check = 0;

void setup() {
  Serial.begin(115200);

  // Wait for up to 3 seconds for the serial port to be opened on the PC side.
  for (int i = 0; i < 30 && !Serial; ++i) {
    delay(100);
  }
  delay(1000);

  // Register callbacks for the heartbeat and encoder feedback messages
  odrv2.onFeedback(onFeedback, &odrv2_user_data); // Register for ODrive 2
  odrv2.onStatus(onHeartbeat, &odrv2_user_data);
  
  odrv3.onFeedback(onFeedback, &odrv3_user_data); // Register for ODrive 3
  odrv3.onStatus(onHeartbeat, &odrv3_user_data);

  // Configure and initialize the CAN bus interface.
  if (!setupCan()) {
    Serial.println("CAN failed to initialize: reset required");
    while (true); // spin indefinitely
  }

  //***************************************ODRIVE VARIABLES: EDIT ABOVE IN VARIABLES SECTION***********************************************//
  odrv2.setLimits(max_motor_rpm, max_motor_current);
  odrv3.setLimits(max_motor_rpm, max_motor_current);

  homing(2);

  odrv2.setControllerMode(ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
  odrv3.setControllerMode(ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
  delay(500);
  odrv2.setControllerMode(ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
  odrv3.setControllerMode(ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
  //**************************************TEST****************************************//

  combineTrajectories(specs, N_SEGS, full_px, full_pz);
  //computeDesiredJointAngles(full_px, full_pz, l1_val);

  //*****************************************LQR SETUP******************************************//
  
  //********************************PRINT HEADER***************************************//

  Serial.print("Torque motor 1 [Nm],");
  Serial.print("Torque motor 2 [Nm],");
  Serial.print("pos joint 1 [rad],");
  Serial.print("pos joint 2 [rad],");
  Serial.print("vel joint 1 [rad/s],");
  Serial.println("vel joint 2 [rad/s]");


}

void loop() {
  pumpEvents(can_intf); // Handle incoming feedback CAN messages

  PID(2.5, 3, 0.08);
  //LQR();

  
}