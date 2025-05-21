void test_z_stiffness(float x_starting_pos, float z_starting_pos, float min_stiffness, float max_stiffness, float dampening){
  float z_goal_pos = z_starting_pos;
  float x_goal_pos = x_starting_pos;
  float set_stiffness = min_stiffness;
  float set_dampening = dampening;

  unsigned long lastMeasureMillis1 = 0;
  unsigned long lastMeasureMillis2 = 0;
  const unsigned long measureInterval1 = 1000;
  const unsigned long measureInterval2 = 40000;

  float stiffness_increase = 20.0;

  if(set_stiffness > max_stiffness){
    set_stiffness = max_stiffness;
  }
  impedanceForceControl(x_goal_pos, z_goal_pos, 0, 0, set_stiffness, set_dampening);

  unsigned long currentMillis = millis();
  
  if(currentMillis - lastMeasureMillis1 >= measureInterval1) {
    lastMeasureMillis1 = currentMillis; 
    measure_scales();
    Serial.print(set_dampening, 3);
    Serial.print(", ");
    Serial.print(set_stiffness, 3);
    Serial.print(", ");
    Serial.print(z_error, 3); 
    Serial.print(", ");
    Serial.println(total_weight, 3);   
    z_goal_pos += 2.5;             
  }

  if((currentMillis - lastMeasureMillis2) >= measureInterval2){
    lastMeasureMillis2 = currentMillis;
    set_stiffness += stiffness_increase;
    z_goal_pos = z_starting_pos;
  }

  impedanceForceControl(x_goal_pos, z_goal_pos, 0, 0, set_stiffness, set_dampening);
}

void test_x_stiffness(){
  impedanceForceControl(50, 250, 0, 0, stiffness, dampening);
  //Serial.println(z_displacement,3);

  //test_stiffness(100.0, 200.0, 300.0, 300.0, 0.0);
  unsigned long currentMillis = millis();
  if(currentMillis - lastMeasureMillis >= measureInterval) {
      lastMeasureMillis = currentMillis; 
      Serial.print(dampening, 3);
      Serial.print(", ");
      Serial.print(stiffness, 3);
      Serial.print(", ");
      Serial.println(x_error, 3); 
              
    }

}

void move_up_down(){
  if(path_completed == 1 && path_index == 0){
    path_completed = 0;
    path_index = 1;
    pos_index = 0;
  }

  if(path_completed == 1 && path_index == 1){
    path_completed = 0;
    path_index = 0;
    pos_index = 0;
  }

  if(path_index == 0){
    move_along_path(75, 200, 75, 300, 0, 0, 800, 30, 1000); //positional values in mm
  }

  if(path_index == 1){
    move_along_path(75, 300, 75, 200, 0, 0, 800, 30, 1000); //positional values in mm
  }
}

void medical_devices_demo_0(float stiffness1, float damp1, float stiffness2, float damp2){
  //puncture trough surface
  float stiffness_setpoint = stiffness1;
  float dampening_setpoint = damp1;

  if(current_z <= 0.250){
    stiffness_setpoint = stiffness2;
    dampening_setpoint = damp2;
  }

  if(current_z <= 0.240){
    stiffness_setpoint = stiffness1;
    dampening_setpoint = damp1;
  }

  Serial.println(current_z);
  impedanceForceControl(75, 300, 0, 0, stiffness_setpoint, dampening_setpoint);
}

void medical_devices_demo_1(float stiffness1, float damp1, float stiffness2, float damp2) {

  int step = int(current_z / 0.01f);
  
  bool useA = (step % 2 == 0);

  float stiffness_setpoint = useA ? stiffness1 : stiffness2;
  float damping_setpoint = useA ? damp1      : damp2;

  impedanceForceControl(75, 300, 0, 0, stiffness_setpoint, damping_setpoint);
}


void medical_devices_demo_2(float stiffness1, float damp1, float stiffness2, float damp2, float center_x,  float center_z, float radius) {
  float dx = current_x - center_x;
  float dz = current_z - center_z;
  float dist2     = dx*dx + dz*dz;
  float radius2   = radius * radius;

  // kies setpoint op basis van binnen/ buiten de cirkel
  float stiffness = (dist2 <= radius2) ? stiffness2 : stiffness1;
  float damping   = (dist2 <= radius2) ? damp2      : damp1;

  impedanceForceControl(75, 300, 0, 0, stiffness, damping);
}

void medical_devices_demo_3(float stiffness_setpoint, float dampening_setpoint){
  update_joint_parameters();

  // 1. Bereken penetratie t.o.v. vlak bij z=0.25 en x=0
  const float z_surface = 0.20;
  const float x_surface = 0.0;
  float dz = z_surface - current_z;   // >0 → in wand
  float dx = x_surface - current_x;   // >0 → in wand

  // 2. Impedance (veer-demper)
  const float K = stiffness_setpoint;   // stijfheid [N/m], afstemmen
  const float B = dampening_setpoint;    // demping [N·s/m], afstemmen
  float Fx = 0, Fz = 0;

  if(dz > 0){
    float vz = -current_z_speed;            // snelheid richting wand
    Fz = K * dz + B * vz;
  }
  if(dx > 0){
    float vx = -current_x_speed;
    Fx = K * dx + B * vx;
  }

  // 3. Doorgeven aan forceControl (geeft 0 torque als Fx=Fz=0)
  forceControl(Fx, Fz);
}

