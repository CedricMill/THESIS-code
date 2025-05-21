void set_motor_torque(int motor, float torque){
  E_stop_state = digitalRead(E_stop);
  torque = 1.2186*torque + 0.0268;

  if(torque > max_motor_torque){
    torque = max_motor_torque;
  }

  else if(torque < ((-1)*max_motor_torque)){
    torque = (-1)*max_motor_torque;
  }

  if(E_stop_state == 1){
    torque = 0;
    Serial.println("EMERGENCY");
  }
  
  if (motor == 1) {
    //odrv2.setControllerMode(ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
    odrv2.setTorque(-torque);
  } 
  else if (motor == 2) {
    //odrv3.setControllerMode(ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
    odrv3.setTorque(torque);
  }
}

void update_position_and_velocity(){
  pumpEvents(can_intf);
  //if (odrv2_user_data.received_feedback && odrv3_user_data.received_feedback) {
  Get_Encoder_Estimates_msg_t feedback2 = odrv2_user_data.last_feedback;
  odrv2_user_data.received_feedback = false; //Reset de flag na uitlezen
  position_motor1 = -(feedback2.Pos_Estimate);
  velocity_motor1 = -(feedback2.Vel_Estimate);
      
  Get_Encoder_Estimates_msg_t feedback3 = odrv3_user_data.last_feedback;
  odrv3_user_data.received_feedback = false; //Reset de flag na uitlezen
  position_motor2 = feedback3.Pos_Estimate;
  velocity_motor2 = feedback3.Vel_Estimate;
  //}
}

void update_current(int motor){
  pumpEvents(can_intf);
  Get_Iq_msg_t current;
  
  if(motor == 1){
    if (!odrv2.request(current, 2)) {
      Serial.println("motor1 current request failed!");
      while (true); // spin indefinitely
    }
  current_motor1 = current.Iq_Measured;
  current_motor1 = -current_motor1;
  }
  
  else if(motor == 2){
    if (!odrv3.request(current, 2)) {
      Serial.println("motor2 current request failed!");
      while (true); // spin indefinitely
    }
  current_motor2 = current.Iq_Measured;
  }
}

void update_power(int motor){
  pumpEvents(can_intf);
  Get_Powers_msg_t power;

  if(motor == 1){
    if (!odrv2.request(power, 2)) {
      Serial.println("motor 1 power request failed!");
      while (true); // spin indefinitely
    }
    electrical_power_motor1 = power.Electrical_Power;
  }

  if(motor == 2){
    if (!odrv3.request(power, 2)) {
      Serial.println("motor2 power request failed!");
      while (true); // spin indefinitely
    }
    electrical_power_motor2 = power.Electrical_Power;
  }

}

void moveMotors(int joint, float pos){
  if (joint == 1) {
    //odrv2.setControllerMode(ODriveControlMode::CONTROL_MODE_POSITION_CONTROL, ODriveInputMode::INPUT_MODE_POS_FILTER);
    odrv2.setPosition(-pos);
  }
  else if (joint == 2) {
    //odrv3.setControllerMode(ODriveControlMode::CONTROL_MODE_POSITION_CONTROL, ODriveInputMode::INPUT_MODE_POS_FILTER);
    odrv3.setPosition(pos);
  }
}