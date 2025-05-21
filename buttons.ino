void buttons(){
  GRN_state = digitalRead(GRN_Button);
  RED_state = digitalRead(RED_Button);
  YELL_state = digitalRead(YELL_Button);

  if(GRN_state == HIGH && GRN_pressed == 0 && stiffness > 100.0){
    GRN_pressed = 1;
    stiffness -= 100;
    dampening -= 2.5;
  }

  else if(GRN_state == LOW && GRN_pressed == 1){
    GRN_pressed = 0;
  }

  if(RED_state == HIGH){
    stiffness = 200;
    dampening = 12.5;
  }

  if(YELL_state == HIGH && YELL_pressed == 0 && stiffness < 600.0){
    YELL_pressed = 1;
    stiffness += 100;
    dampening += 2.5;
  }

  else if(YELL_state == LOW && YELL_pressed == 1){
    YELL_pressed = 0;
  }

  impedanceForceControl(50, 250, 0, 0, stiffness, dampening);
}