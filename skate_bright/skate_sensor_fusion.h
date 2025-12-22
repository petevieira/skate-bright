#ifndef SKATE_PRESSURE_H
#define SKATE_PRESSURE_H

enum RearLedsState {
  REAR_BRAKING = 0,
  REAR_SLOWING,
  REAR_CRUISING,
  REAR_ACCELERATING,
  REAR_IN_THE_AIR
};

enum FrontLedsState {
  FRONT_CRUSING = 0,
  FRONT_IN_THE_AIR
};

enum SideLedsState {
  SIDE_VERTICAL = 0,
  SIDE_LEANING_INSIDE,
  SIDE_LEANING_OUTSIDE,
  SIDE_HEEL_UP,
  SIDE_TOE_UP,
  SIDE_IN_THE_AIR
};

class SkateSensorFusion {

  void fuseSensorData();
};

#endif