void scales_calibration(){
  scale1.begin(LOADCELL1_DOUT_PIN, LOADCELL1_SCK_PIN);
  scale2.begin(LOADCELL2_DOUT_PIN, LOADCELL2_SCK_PIN);

  scale1.set_scale(2280.f);    // this value is obtained by calibrating the scale with known weights; see the README for details
  scale1.tare();				        // reset the scale to 0

  scale2.set_scale(2280.f);    // this value is obtained by calibrating the scale with known weights; see the README for details
  scale2.tare();				        // reset the scale to 0

}

void measure_scales(){
  float mass_scale1 = 0.023 * scale1.get_units() + 0.0014;
  float mass_scale2 = 0.020 * scale2.get_units() + 0.0014;

  total_weight  = -(mass_scale1 * 9.81 + mass_scale2 * 9.81);

  // unsigned long t_end = micros();
  // unsigned long dt_us = t_end - t_start;
  // float dt_ms = dt_us / 1000.0;

  // Serial.print("Elapsed time (ms):\t");
  // Serial.println(dt_ms, 3);
}
