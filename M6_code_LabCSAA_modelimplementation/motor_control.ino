void set_motor_torque(int motor, float torque) {
  //motor 1 is de hip motor, motor 2 is de kniemotor
  if(torque > max_motor_torque){
    torque = max_motor_torque;
    //torque = 0;
  }

  else if(torque < (-1 * max_motor_torque)){
    torque = -1 * max_motor_torque;
    //torque = 0;
  }
  
  if (motor == 1) {
      odrv2.setTorque(-torque);
  } 
  else if (motor == 2) {
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

void moveMotors(int joint, float pos){
  // Used to make a certain motor (joint) go to a certain rotary position (pos) relative to its starting position.
  if (joint == 1) {
    odrv2.setPosition(-pos);
  }
  else if (joint == 2) {
    odrv3.setPosition(pos);
  }  

}
