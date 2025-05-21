void LQR(){
  //Persistent state (blijft bewaard tussen aanroepen)
  static unsigned long last_time            = 0;
  static int           f_index              = 0;

  //Lokale variabelen voor deze iteratie
  float error_joint1     = 0.0f;
  float error_joint2     = 0.0f;
  float setpoint_joint1  = 0.0f;
  float setpoint_joint2  = 0.0f;
  float torque_joint1    = 0.0f;
  float torque_joint2    = 0.0f;
  const float dt         = 0.01f;       // 10 ms
  const float threshold  = 0.010f;

  // Init-oproep de eerste keer
  if (last_time == 0) {
    last_time = millis();
    computeDesiredJointAngles(full_px, full_pz, l1_val);
    return;
  }

  // Tijd check: doe elke ~10 ms één stap
  unsigned long now = millis();
  if (now - last_time < dt * 1000) {
    return;
  }
  last_time = now;

  // Lees sensoren
  update_joint_parameters();  // vult current_f1, current_f2

  // Bepaal gewenste hoeken (setpoints)
  setpoint_joint1 = f1_list[f_index];
  setpoint_joint2 = f2_list[f_index];

  // 1) Error
  error_joint1 = setpoint_joint1 - current_f1;
  error_joint2 = setpoint_joint2 - current_f2;

  // 4) LQR‐output w1 w2 f1 f2
  torque_joint1 = (K11 * current_f1_speed + K12 * current_f2_speed + K13 * error_joint1 + K14 * error_joint2);
  torque_joint2 = (K21 * current_f1_speed + K22 * current_f2_speed + K23 * error_joint1 + K24 * error_joint2);

  //torque_joint1 = (K11 * current_f1_speed + K13 * error_joint1);
  //torque_joint2 = (K22 * current_f2_speed + K24 * error_joint2);


  set_motor_torque(1, torque_joint1);
  set_motor_torque(2, torque_joint2);

  float x_desired[] = {0.0f, 0.0f, setpoint_joint1, setpoint_joint2};

  float position_error = computePositionError(x_desired, l1_val);

  if (position_error <= threshold) {
    f_index++;
    const int listSize = sizeof(f1_list) / sizeof(f1_list[0]);

    if (f_index >= listSize) {
      f_index = listSize - 1;  // of zet terug op 0 als je wilt herhalen
    }
  }

  Serial.print(torque_joint1, 3);
  Serial.print(",");
  Serial.print(torque_joint2, 3);
  Serial.print(",");
  Serial.print(current_f1, 3);
  Serial.print(",");
  Serial.println(current_f2, 3);


}
