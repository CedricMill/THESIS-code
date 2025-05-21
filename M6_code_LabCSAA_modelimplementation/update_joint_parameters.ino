void update_joint_parameters(){

  update_position_and_velocity();
  current_f1 = 0.1 * (position_motor1 - f1_home_rot) * 2 * pi;
  current_f2 = 0.1 * (position_motor2 - f2_home_rot) * 2 * pi;

  current_f1_speed = 0.1 * velocity_motor1 * 2 * pi;
  current_f2_speed = 0.1 * velocity_motor2 * 2 * pi;

  current_x = l1_val*sin(current_f1) + l1_val*sin(current_f1 + current_f2);
  current_z = l1_val*cos(current_f1) + l1_val*cos(current_f1 + current_f2);

  current_x_speed = l1_val * cos(current_f1) * current_f1_speed + l1_val * cos(current_f1 + current_f2)* (current_f1_speed + current_f2_speed);
  current_z_speed = -l1_val * sin(current_f1) * current_f1_speed -l1_val * sin(current_f1 + current_f2)* (current_f1_speed + current_f2_speed);
}