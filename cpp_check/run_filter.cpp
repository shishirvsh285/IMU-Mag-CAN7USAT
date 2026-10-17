// reads "dt gx gy gz ax ay az mx my mz awx awy awz" lines, prints q after each update
#include <cstdio>
#include "../AdaptiveMahony/AdaptiveMahony.h"
int main(int argc, char **argv)
{
  AdaptiveMahony f;
  if (argc > 2) { f.magField = atof(argv[1]); f.dipDeg = atof(argv[2]); f.useDip = true; }
  float v[13];
  while (scanf("%f %f %f %f %f %f %f %f %f %f %f %f %f", v, v+1, v+2, v+3, v+4, v+5, v+6, v+7, v+8, v+9, v+10, v+11, v+12) == 13)
  {
    f.update(v[1], v[2], v[3], v[4], v[5], v[6], v[0], v + 7, v + 10);
    printf("%.7f %.7f %.7f %.7f %.6f\n", f.q[0], f.q[1], f.q[2], f.q[3], f.kp);
  }
}
