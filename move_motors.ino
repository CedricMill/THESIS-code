void impedanceForceControl(float pos_x, float pos_z, float speed_x, float speed_z, float stiffness, float dampening){
  float pos_tolerance = 0.010; //tolerance on position [m]
  stiffness = 0.94*stiffness + 51.91;

  update_joint_parameters();

  pos_x = pos_x/1000;
  pos_z = pos_z/1000;

  speed_x = speed_x/1000;
  speed_z = speed_z/1000;

  /* 2. Jointhoeken‑ en snelheden → rad, rad/s */
  float theta1    = current_theta1;
  float theta2    = current_theta2;
  float theta1dot = current_theta1_speed;
  float theta2dot = current_theta2_speed;

  /* 3. Forward kinematics  (x naar links, z naar beneden) */
  float s1  = sinf(theta1);
  float c1  = cosf(theta1);
  float s12 = sinf(theta1+theta2);
  float c12 = cosf(theta1+theta2);

  float x =  L1 * s1  + L2 * s12;   // ← left +
  float z =  L1 * c1  + L2 * c12;   // ↓ under +

  /* 4. Jacobian     J = ∂(x,z)/∂(q1,q2) */
  float j11 =  L1 * c1  + L2 * c12;   // ∂x/∂q1
  float j12 =  L2 * c12;              // ∂x/∂q2
  float j21 = -L1 * s1  - L2 * s12;   // ∂z/∂q1
  float j22 = -L2 * s12;              // ∂z/∂q2

  /* 5. Cartesian snelheden   ẋ = J q̇ */
  float x_dot = j11*theta1dot + j12*theta2dot;
  float z_dot = j21*theta1dot + j22*theta2dot;

  /*6. positie error berekenen*/
  x_error = pos_x - x;
  z_error = pos_z - z;

  x_dot_error = speed_x - x_dot;
  z_dot_error = speed_z - z_dot;

  /* 7. Virtuele veer‑demper‑krachten */
  float Fx = stiffness * (x_error) + dampening * x_dot_error;
  float Fz = stiffness * (z_error) + dampening * z_dot_error;

  /* 8. Joint­koppels  τ = Jᵀ F */
  float tau1 = j11*Fx + j21*Fz;
  float tau2 = j12*Fx + j22*Fz;

  /* 9. Koppels naar de motoren */
  tau1 = tau1 * 0.1;
  tau2 = tau2 * 0.1;

  if((abs(x_error) <= pos_tolerance) && (abs(z_error) <= pos_tolerance)){
    impedance_pos_reached = 1;
  }
  
  set_motor_torque(1, tau1);
  set_motor_torque(2, tau2);

  update_power(1);
  update_power(2);

  update_current(1);
  update_current(2);

  Serial.print(millis());
  Serial.print(",");
  Serial.print(tau1,3);
  Serial.print(",");
  Serial.print(tau2,3);
  Serial.print(",");
  Serial.print(z,3);
  Serial.print(",");
  Serial.print(electrical_power_motor1,3);
  Serial.print(",");
  Serial.print(electrical_power_motor2,3);
  Serial.print(",");
  Serial.print(current_motor1,3);
  Serial.print(",");
  Serial.println(current_motor2,3);

}

void homing(){
  float motor1_homing_current = 3.0;
  float motor2_homing_current = 2.0;

  pumpEvents(can_intf);
  float motor2_calib_rot;
  float motor1_position_rot;

  update_current(2);
  while(abs(current_motor2) < motor2_homing_current){
    motor2_home_rot -= 0.001;
    moveMotors(2, motor2_home_rot);
    update_current(2);
  }
  motor2_calib_rot = motor2_home_rot;

  update_position_and_velocity();
  while(position_motor2 <= (2 + motor2_calib_rot)){
    motor2_home_rot += 0.001;
    update_position_and_velocity();
    moveMotors(2, (motor2_home_rot));
    delay(1);
  }
  motor2_home_rot = 4.03 + motor2_calib_rot;
  //Serial.println("motor 2 homing done");

  update_current(1);
  while(abs(current_motor1) < motor1_homing_current){
    motor1_home_rot -= 0.001;
    moveMotors(1, motor1_home_rot);
    update_current(1);
    
  }
  //Serial.println("motor 1 homing done");

  while(position_motor1 <= (1.1 + motor1_home_rot)){
    motor1_position_rot += 0.001;
    update_position_and_velocity();
    moveMotors(1, (motor1_position_rot + motor1_home_rot));
    delay(1);
  }
  //Serial.println("motor 1 positioning done");
}

void forceControl(float force_x, float force_z){
  float pos_tolerance = 0.010; //tolerance on position [m]
  //force = 0.94*stiffness + 51.91;

  update_joint_parameters();

  /* 2. Jointhoeken‑ en snelheden → rad, rad/s */
  float theta1    = current_theta1;
  float theta2    = current_theta2;
  float theta1dot = current_theta1_speed;
  float theta2dot = current_theta2_speed;

  /* 3. Forward kinematics  (x naar links, z naar beneden) */
  float s1  = sinf(theta1);
  float c1  = cosf(theta1);
  float s12 = sinf(theta1+theta2);
  float c12 = cosf(theta1+theta2);

  float x =  L1 * s1  + L2 * s12;   // ← left +
  float z =  L1 * c1  + L2 * c12;   // ↓ under +

  /* 4. Jacobian     J = ∂(x,z)/∂(q1,q2) */
  float j11 =  L1 * c1  + L2 * c12;   // ∂x/∂q1
  float j12 =  L2 * c12;              // ∂x/∂q2
  float j21 = -L1 * s1  - L2 * s12;   // ∂z/∂q1
  float j22 = -L2 * s12;              // ∂z/∂q2


  /* 7. Virtuele krachten */
  float Fx = force_x;
  float Fz = force_z;

  /* 8. Joint­koppels  τ = Jᵀ F */
  float tau1 = j11*Fx + j21*Fz;
  float tau2 = j12*Fx + j22*Fz;

  /* 9. Koppels naar de motoren */
  tau1 = tau1 * 0.1;
  tau2 = tau2 * 0.1;
  
  static float prev1 = 0, prev2 = 0;
  const float alpha = 0.1f;
  tau1 = alpha*tau1 + (1-alpha)*prev1;
  tau2 = alpha*tau2 + (1-alpha)*prev2;
  prev1 = tau1; prev2 = tau2;

  set_motor_torque(1, tau1);
  set_motor_torque(2, tau2);

  Serial.print(tau1);
  Serial.print(",");
  Serial.println(tau2);

}