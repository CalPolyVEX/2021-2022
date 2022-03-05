#ifndef ROBOT_SPECIFICS_H
#define ROBOT_SPECIFICS_H

// Header-based robot specifics configuration
// Change configuration by uncommenting lines or changing the value
// of defines.
// ---

// Change the lines here based on what robot we're compiling for.
//#define SMALL_ROBOT_ATREIDES
#define LARGE_ROBOT_HARKONNEN

// Average radius of the two different wheel types.
// The small and big robot both use the same values, for now.
#define WHEEL_DIAMETER 4.1_in

// just something for skills so we get like 20 points every time
//#define AUTONOMOUS_ONLY_DRIVE_FORWARD

#ifdef LARGE_ROBOT_HARKONNEN
  #define DRIVE_MODE TANK
  #define MIDDLE_IDLE_WHEEL_DIAMETER WHEEL_DIAMETER
  #define LENGTH_TO_MIDDLE_WHEEL 0_m
  // See this for what "wheel track" means:
  // https://okapilib.github.io/OkapiLib/classokapi_1_1ChassisScales.html#a7e35f22518d4f74105ffa741b40dcb17
  #define WHEEL_TRACK 16.2_in

  // our electrical setup is messed up on the large robot
  // so just use the integrated encoders :(
  #define USE_INTEGRATED_ENCODERS
#endif

#ifdef SMALL_ROBOT_ATREIDES
  #define HAS_MIDDLE_ENCODER

  // measured
  #define MIDDLE_IDLE_WHEEL_DIAMETER 3.25_in

  // TODO: This is a guess and will need to be confirmed experimentally based
  // on the actual center of rotation.
  #define LENGTH_TO_MIDDLE_WHEEL 1_in

  #define DRIVE_MODE ARCADE
  #define WHEEL_TRACK 12.75_in

  #define USE_INTEGRATED_ENCODERS
#endif

#endif
