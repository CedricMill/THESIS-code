#ifndef TRAJECTORY_SPEC_H
#define TRAJECTORY_SPEC_H

// ---------- Traject-type constanten ----------
#define LINEAR    0
#define PARABOLIC 1
#define CIRCULAR  2

// ---------- Struct voor segment-specificaties ----------
struct SegmentSpec {
  float x_start;
  float z_start;
  float x_end;
  float z_end;
  int   n_points;
  int   type;      // LINEAR, PARABOLIC of CIRCULAR
};
void plotTrajectory(float px_desired[], float pz_desired[], int n_points, unsigned long delay_ms = 1);
#endif
