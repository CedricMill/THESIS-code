void PID(float Kp, float Ki, float Kd) {
  //Persistent state (blijft bewaard tussen aanroepen)
  static unsigned long last_time            = 0;
  static int           f_index              = 0;
  static float         prev_error_joint1    = 0.0f;
  static float         prev_error_joint2    = 0.0f;
  static float         integral_joint1      = 0.0f;
  static float         integral_joint2      = 0.0f;

  //Lokale variabelen voor deze iteratie
  float error_joint1     = 0.0f;
  float error_joint2     = 0.0f;
  float derivative_joint1= 0.0f;
  float derivative_joint2= 0.0f;
  float setpoint_joint1  = 0.0f;
  float setpoint_joint2  = 0.0f;
  float torque_joint1    = 0.0f;
  float torque_joint2    = 0.0f;
  const float dt         = 0.01f;       // 10 ms
  const float threshold  = 0.005f;

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

  // 2) Integral
  integral_joint1 += error_joint1 * dt;
  integral_joint2 += error_joint2 * dt;

  // 3) Derivative
  derivative_joint1 = (error_joint1 - prev_error_joint1) / dt;
  derivative_joint2 = (error_joint2 - prev_error_joint2) / dt;

  // Onthoud voor de volgende iteratie
  prev_error_joint1 = error_joint1;
  prev_error_joint2 = error_joint2;

  // 4) PID‐output
  torque_joint1 = -(Kp * error_joint1 + Ki * integral_joint1 + Kd * derivative_joint1);
  torque_joint2 = -(Kp * error_joint2 + Ki * integral_joint2 + Kd * derivative_joint2);


  set_motor_torque(1, -torque_joint1);
  set_motor_torque(2, -torque_joint2);

  float x_desired[] = {0.0f, 0.0f, setpoint_joint1, setpoint_joint2};

  float position_error = computePositionError(x_desired, l1_val);

  if (position_error <= threshold) {
    f_index++;
    const int listSize = sizeof(f1_list) / sizeof(f1_list[0]);

    if (f_index >= listSize) {
      f_index = listSize - 1;  // of zet terug op 0 als je wilt herhalen
    }
  }

  Serial.print(torque_joint1);
  Serial.print(",");
  Serial.print(torque_joint2);
  Serial.print(",");
  Serial.print(current_f1);
  Serial.print(",");
  Serial.print(current_f2);
  Serial.print(",");
  Serial.print(current_f1_speed);
  Serial.print(",");
  Serial.println(current_f2_speed);

}
