void update_joint_parameters(){

  update_position_and_velocity();
  current_theta1 = 0.1 * (position_motor1 - motor1_home_rot) * 2 * pi;
  current_theta2 = 0.1 * (position_motor2 - motor2_home_rot) * 2 * pi;

  current_theta1_speed = 0.1 * velocity_motor1 * 2 * pi;
  current_theta2_speed = 0.1 * velocity_motor2 * 2 * pi;

  current_x = L1*sin(current_theta1) + L2*sin(current_theta1 + current_theta2);
  current_z = L1*cos(current_theta1) + L2*cos(current_theta1 + current_theta2);

  current_x_speed = L1 * cos(current_theta1) * current_theta1_speed + L2 * cos(current_theta1 + current_theta2)* (current_theta1_speed + current_theta2_speed);
  current_z_speed = -L1 * sin(current_theta1) * current_theta1_speed -L2 * sin(current_theta1 + current_theta2)* (current_theta1_speed + current_theta2_speed);
}