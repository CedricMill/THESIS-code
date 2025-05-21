void homing(int leg){
  float current_ax1;
  float current_ax2;
  //float f1_go_home = 0;
  float f2_go_home = 0;

  Get_Iq_msg_t current;
  if (!odrv3.request(current, 1)) {
    Serial.println("current request failed!");
    while (true); // spin indefinitely
  }
  current_ax2 = current.Iq_Measured;
  while(abs(current_ax2) < 2){
    f2_home_rot -= 0.001;
    moveMotors(2, f2_home_rot);
    odrv3.request(current, 1);
    current_ax2 = current.Iq_Measured;
    //Serial.println(current_ax2);
  }

  update_position_and_velocity();
  while(position_motor2 <= (4.03 + f2_home_rot)){
    f2_go_home += 0.001;
    update_position_and_velocity();
    moveMotors(2, (f2_go_home + f2_home_rot));
    delay(1);
  }
  f2_home_rot = f2_home_rot + 4.03;
  f2_home_angle = f2_home_rot*2*pi;

  if (!odrv2.request(current, 1)) {
    Serial.println("current request failed!");
    while (true); // spin indefinitely
  }
  current_ax1 = current.Iq_Measured;
  while(abs(current_ax1) < 2.5){
    f1_home_rot -= 0.001;
    moveMotors(1, f1_home_rot);
    odrv2.request(current, 1);
    current_ax1 = current.Iq_Measured;
  }
  f1_home_angle = f1_home_rot*2*pi;

}