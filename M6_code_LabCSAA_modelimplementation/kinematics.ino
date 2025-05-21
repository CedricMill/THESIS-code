#include "TrajectorySpec.h"

void generateTrajectory(float x_start, float z_start, float x_end, float z_end, int n_points, float px_desired[], float pz_desired[], int type){
  // Voorbereiden lineaire interpolatiefactor
  for (int i = 0; i < n_points; i++) {
    float t = (n_points > 1) ? (float)i / (n_points - 1) : 0.0;
    px_desired[i] = (x_start + (x_end - x_start) * t);
  }

  if (type == LINEAR) {
    for (int i = 0; i < n_points; i++) {
      float t = (n_points > 1) ? (float)i / (n_points - 1) : 0.0;
      pz_desired[i] = (z_start + (z_end - z_start) * t);
    }
  }
  else if (type == PARABOLIC) {
    // Kies een hoogtepunt (mid_z) zoals in Python: 0.2 van de totale afstand
    float mid_z = (z_start + z_end) / 2.0 + 0.2 * fabs((z_end - z_start) + (x_end - x_start));
    for (int i = 0; i < n_points; i++) {
      float t = (n_points > 1) ? (float)i / (n_points - 1) : 0.0;
      float u = 1.0 - t;
      pz_desired[i] = (u*u * z_start + 2.0 * u * t * mid_z + t*t * z_end);
    }
  }
  else if (type == CIRCULAR) {
    // Bepaal middelpunt en straal
    float center_x = (x_start + x_end) / 2.0;
    float center_z = (z_start + z_end) / 2.0;
    float dx = x_end - center_x;
    float dz = z_end - center_z;
    float radius = sqrt(dx*dx + dz*dz);

    // Hoeken vanaf het middelpunt
    float angle_start = atan2(z_start - center_z, x_start - center_x);
    float angle_end   = atan2(z_end   - center_z, x_end   - center_x);

    // Zorg voor een A→B‐beweging in klokrichting
    if (angle_start < angle_end) {
      angle_start += 2.0 * PI;
    }

    for (int i = 0; i < n_points; i++) {
      float t = (n_points > 1) ? (float)i / (n_points - 1) : 0.0;
      float theta = angle_start + (angle_end - angle_start) * t;
      px_desired[i] = (center_x + radius * cos(theta));
      pz_desired[i] = (center_z + radius * sin(theta));
    }
  }
}

void combineTrajectories(const SegmentSpec segments[], int n_segments, float full_px[], float full_pz[])
{
  total_pts = 0;

  const int MAX_SEG = list_length*10;
  float px_seg[MAX_SEG];
  float pz_seg[MAX_SEG];

  for (int i = 0; i < n_segments; i++) {
    int np = segments[i].n_points;
   
    generateTrajectory(segments[i].x_start, segments[i].z_start, segments[i].x_end, segments[i].z_end, np, px_seg, pz_seg, segments[i].type);

    int start_j = (i > 0) ? 1 : 0;

    for (int j = start_j; j < np; j++) {
      full_px[total_pts] = px_seg[j];
      full_pz[total_pts] = pz_seg[j];
      total_pts++;
    }
  }
}

void plotTrajectory(float px_desired[], float pz_desired[], int n_points, unsigned long delay_ms){
  // Zorg dat Serial al is gestart in setup(): Serial.begin(...)
  for (int i = 0; i < n_points; i++) {
    Serial.print(px_desired[i], 3);
    Serial.print(',');               // separator tussen x en z
    Serial.println(pz_desired[i], 3);
    delay(delay_ms);
  }
}

void computeDesiredJointAngles(const float px[], const float pz[], float l1_val) {
  // l1_val moet > 0 zijn!
  for (int i = 0; i < total_pts; i++) {
    float xi = px[i];
    float zi = pz[i];

    // hoek t.o.v. Z-as, robuust bij zi==0
    float alpha = atan2(xi, zi);

    // afstand tot oorsprong
    float dist = sqrt(xi*xi + zi*zi);

    // argument voor acos clampen tussen -1 en +1
    float arg = dist / (2.0 * l1_val);
    arg = constrain(arg, -1.0, 1.0);

    float beta = acos(arg);

    // gewenste gewrichtshoeken
    float f1 = alpha + beta;
    float f2 = -2.0 * beta;

    f1_list[i] = f1;
    f2_list[i] = f2;
  }
}

float computePositionError(const float x_desired[4], float l1_val) {
  // 1) hoekwaarden extraheren
  //x4 = {snelheid joint 1 (rad/s), snelh joint 2 (rad/s), hoek f1 (rad), hoek f2 (rad)}
  update_position_and_velocity();
  float f1 = 0.1*(position_motor1*2*pi - f1_home_angle);
  float f2 = 0.1*(position_motor2*2*pi - f2_home_angle);

  float f1_d = x_desired[2];
  float f2_d = x_desired[3];

  // 2) forward kinematics huidige positie
  float px  = l1_val * sin(f1) + l1_val * sin(f1 + f2);
  float pz  = -(l1_val * cos(f1) + l1_val * cos(f1 + f2));

  // 3) forward kinematics gewenste positie
  float px_d = l1_val * sin(f1_d) + l1_val * sin(f1_d + f2_d);
  float pz_d = -(l1_val * cos(f1_d) + l1_val * cos(f1_d + f2_d));

  // 4) afstandsberekening
  float dx  = px  - px_d;
  float dz  = pz  - pz_d;
  float err = sqrt(dx*dx + dz*dz);

  // 5) omzetting naar mm indien gewenst
  return err;
}
