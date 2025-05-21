void calculate_path(float x_start, float z_start, float x_end, float z_end, bool linear, bool circular, float dist_between_points){
  if (dist_between_points <= 0.0f) {
    n_points = 0;
    return;
  }

  float dx = x_end - x_start;
  float dz = z_end - z_start;

  if (linear) {
    float total_dist = sqrtf(dx*dx + dz*dz);
    int segments = (int)floorf(total_dist / dist_between_points);
    if (segments < 1) segments = 1;

    n_points = segments + 1;

    for (int i = 0; i <= segments; i++) {
      float t = (i < segments) ? (dist_between_points * i) / total_dist : 1.0f;  // laatste punt exact op eind
      path_x[i] = x_start + t * dx;
      path_z[i] = z_start + t * dz;
    }
  }

  else if (circular) {
    float center_x = 0.5f * (x_start + x_end);
    float center_z = 0.5f * (z_start + z_end);
    float radius   = 0.5f * sqrtf(dx*dx + dz*dz);

    float arc_length = M_PI * radius;

    int segments = (int)floorf(arc_length / dist_between_points);
    if (segments < 1) segments = 1;

    n_points = segments + 1;

    float start_angle = atan2f(z_start - center_z, x_start - center_x);
    float end_angle   = start_angle + M_PI;
    float angle_span  = end_angle - start_angle;

    for (int i = 0; i <= segments; i++) {
      float t = (i < segments) ? (dist_between_points * i) / arc_length : 1.0f;
      float theta = start_angle + t * angle_span;
      path_x[i] = center_x + radius * cosf(theta);
      path_z[i] = center_z + radius * sinf(theta);
    }
  }
  else {
    n_points = 0;
  }
}

void move_along_path(float pos_x_start, float pos_z_start, float pos_x_end, float pos_z_end, float x_speed, float z_speed, float stiffness_setpoint, float dampening_setpoint, unsigned long move_time){
  //Give values in mm

  calculate_path(pos_x_start, pos_z_start, pos_x_end, pos_z_end, true, false, distance_between_points);
  
  unsigned long interval_ms = move_time/((sqrt((pos_x_end-pos_x_start)*(pos_x_end-pos_x_start) + (pos_z_end-pos_z_start)*(pos_z_end-pos_z_start)))/distance_between_points);

  unsigned long now = millis();

  if(last_increment_time == 0) {
    last_increment_time = now;
  }

  if(now - last_increment_time >= interval_ms) {
    last_increment_time = now;          // reset de timer

    if (pos_index < n_points - 1) {
      pos_index++;
      if (pos_index == n_points - 1) {
        path_completed = 1;
      }
    }
  }

  float x_setpoint = path_x[pos_index];
  float z_setpoint = path_z[pos_index];

  impedanceForceControl(x_setpoint, z_setpoint, x_speed, z_speed, stiffness_setpoint, dampening_setpoint);

}
