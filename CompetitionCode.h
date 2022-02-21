#define SPIN_90_TIME 1300
#define FIND_LINE_90_DELAY 1000
#define FIND_LINE_180_DELAY 2400
#define TURN_SPEED 0.7
#define CUBE_2_CHECK_MID_DELAY 5000
#define CUBE_3_CHECK_MID_DELAY 8000
#define FINISHING_CHECK_MID_DELAY 600
#define CUBE_TEST_DISTANCE 100
#define CUBE_TEST_THRESHOLD 14

#define MOTOR_R_SPEED 210  // Was 235 before line sensor failure
#define MOTOR_L_SPEED 255

#define PINCER_OPEN_ANGLE 130
#define PINCER_CLOSED_ANGLE 45

enum
{
  cubeFine,
  cubeCoarse
};

enum // States determining which part of the routing the robot is in
{
  idling = 0,
  leavingStart,
  findingCube1or2,
  //findingCube2,
  findingCube3,
  returningCube,
  finishing
};

enum  // States determining how many line junctions passed when looking for final cube
{
  leavingJunc = 0,
  lookingForCubeJunc,
  leavingCubeJunc,
  lookingForSquare,
  drivingIntoSquare
};
