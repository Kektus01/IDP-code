#include "Main_code.h"

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Servo.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *MotorL = AFMS.getMotor(1);
Adafruit_DCMotor *MotorR = AFMS.getMotor(2);

Servo PincerServo;

// Pin definitions
const int linePinFL = 6;
const int linePinFR = 7;
const int coarseLed = 8;
const int fineLed = 9;
const int linePinMid = 5;
const int rangeFinderPin = A0;
const int startButtonPin = 13;
const int flashingLedPin = 3;
const int trigPin = 12;    // Trigger
const int echoPin = 11;    // Echo


long duration, cm;  // For ultrasonic sensor
bool seenCube;
int cubesReturned, currentCubeType;
//int coarseCubesReturned, fineCubesReturned;
//bool lineFL, lineFR;
int currentStep;  // State variable for switch case
int distanceSensorStatus;
unsigned long actionStartTime;  // Time marker to be used for delays in something happening
int cube3Step;  // current stage of third cube collection
//int juncEdgesPassed;  //Number of edges of line junctions passed
bool checkingLineMid;
int checkLineMidDelay;  //Delay before rear line sensor starts being checked


bool CheckLine(int sensorPin)
{
  return(digitalRead(sensorPin)); 
}


// Driving functions

void MotorMaxSpeed()
{
  MotorL->setSpeed(MOTOR_L_SPEED);
  MotorR->setSpeed(MOTOR_R_SPEED);
  digitalWrite(flashingLedPin, HIGH);
}

void DriveForward()
{
  MotorL->run(FORWARD);
  MotorR->run(FORWARD);

  MotorMaxSpeed();
}

void DriveBackwards()
{
  MotorL->run(BACKWARD);
  MotorR->run(BACKWARD);

  MotorMaxSpeed();
}

void SpinLeft()
{
  MotorL->run(BACKWARD);
  MotorR->run(FORWARD);

  MotorMaxSpeed();
}

void SpinRight()
{
  MotorL->run(FORWARD);
  MotorR->run(BACKWARD);

  MotorMaxSpeed();
}

void Spin90Left()
{
  SpinLeft();
  delay(SPIN_90_TIME);
}

void Spin90Right()
{
  SpinRight();
  delay(SPIN_90_TIME);
}

/*
void Spin180()
{
  SpinRight();
  delay(2 * SPIN_90_DELAY);
}
*/

void StopMoving()
{
  MotorL->run(RELEASE);
  MotorR->run(RELEASE);
  digitalWrite(flashingLedPin, LOW);  // Turn off flashing LED

  
  //MotorL->setSpeed(0);
  //MotorR->setSpeed(0);
  
}

void FollowLine()
{
  MotorL->run(FORWARD);
  MotorR->run(FORWARD);

  if(!(CheckLine(linePinFR) && CheckLine(linePinFL)))  //Check for case of both line sensors seeing a line
  {
    if(CheckLine(linePinFR)) MotorR->setSpeed(TURN_SPEED * MOTOR_R_SPEED);
    else MotorR->setSpeed(MOTOR_R_SPEED);

    if(CheckLine(linePinFL)) MotorL->setSpeed(TURN_SPEED * MOTOR_L_SPEED);
    else MotorL->setSpeed(MOTOR_L_SPEED);
  }
  else
  {
    MotorMaxSpeed();
  }
}

void TurnAroundOnLine()
{
  SpinLeft();

  /*
  while(1) //Loop until front sensors have left the line
  {
    line_Status1 = analogRead(linePinFL);   //read the status of the line sensor value
  
    line_Status2 = analogRead(linePinFR);
    if(!(line_Status1 >= 100)) break;
  }
  */

  delay(FIND_LINE_180_DELAY);
  
  while(1)  //Loop until line has been found again
  {
    if(CheckLine(linePinFL)) break;
  }

  while(1)  //Loop until line has been left
  {
    if(!CheckLine(linePinFL)) break;
  }
}

void FindLine90L()  Spin 90 degrees left and find line
{
  SpinLeft();
  delay(FIND_LINE_90_DELAY);
  
  while(1) //Loop until line has been found again
  {
    if(CheckLine(linePinFL)) break;
  }

  while(1)
  {
    if(!CheckLine(linePinFL)) break;
  }
}

void FindLine90R()  Spin 90 degrees right and find line
{
  SpinRight();
  delay(FIND_LINE_90_DELAY);
  
  while(1) //Loop until line has been found again
  {
    if(CheckLine(linePinFR)) break;
  }

  while(1)
  {
    if(!CheckLine(linePinFR)) break;
  }
}

void ClosePincers(bool waitForClose) //waitForClose determines whether the robot waits for the pincers to close before proceeding
{
  int i;
  
  if(waitForClose)
  {
    for(i = PINCER_OPEN_ANGLE; i >= PINCER_CLOSED_ANGLE; i -= 1)
    {
      PincerServo.write(i);
      delay(15);
    }
  }
  else
  {
    PincerServo.write(PINCER_CLOSED_ANGLE);
  }
  //delay(200);
}

void OpenPincers()
{
  PincerServo.write(PINCER_OPEN_ANGLE);
}


int TestCubeType()
{
  int tests;
  int highCount = 0;
  
  //currentCubeType = cubeCoarse;

  // Turn left a bit at the start
  SpinLeft();
  delay(130);
  StopMoving();
  delay(50);


  for(tests = 0; tests < 8; tests++)
  {
    {  //Turn right a little bit on each test
      SpinRight();
      delay(30);
      StopMoving();
      delay(50);
    }
    
    digitalWrite(trigPin, LOW);
    delayMicroseconds(5);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
   
    // Read the signal from the sensor: a HIGH pulse whose
    // duration is the time (in microseconds) from the sending
    // of the ping to the reception of its echo off of an object.
    pinMode(echoPin, INPUT);
    duration = pulseIn(echoPin, HIGH);
   
    // Convert the time into a distance
    cm = (duration/2) / 29.1;
    Serial.print(cm);
    Serial.print("cm");
    Serial.println();
    
    if(cm > CUBE_TEST_THRESHOLD)
    {
      highCount++;
      
    }
    delay(250);
  }
  
  //  Realign with cube
  SpinLeft();
  delay(110);
  StopMoving();
  delay(50);
  if(highCount > 7)
  {
    return cubeFine;
  }
  return cubeCoarse;
}


void CollectCube()  //Grabs cube, checks type and turns around
{
  StopMoving();
  ClosePincers(true);
  delay(200);
  OpenPincers();
  delay(CUBE_TEST_DISTANCE);
  DriveBackwards();
  delay(CUBE_TEST_DISTANCE);
  StopMoving();
  delay(200);
  currentCubeType = TestCubeType();
  switch(currentCubeType)
  {  // Turn on correct LED
    case cubeCoarse:
    digitalWrite(coarseLed, HIGH);
    break;

    case cubeFine:
    digitalWrite(fineLed, HIGH);
    break;
  }
  DriveForward();
  delay(300);
  StopMoving();
  ClosePincers(true);
  TurnAroundOnLine();
}

void PutCubeInBox()
{
  OpenPincers();
  DriveForward();
  delay(1000);
  StopMoving();
  delay(200);
  switch(currentCubeType)
  {  //Turn off cube LED
    case cubeCoarse:
    digitalWrite(coarseLed, LOW);
    break;

    case cubeFine:
    digitalWrite(fineLed, LOW);
    break;
  }
  DriveBackwards();
  delay(1000);
  cubesReturned++;
}

void DeliverCube()
{
  switch(currentCubeType)
  {
    case cubeFine:
    Spin90Left();
    PutCubeInBox();
    if(cubesReturned == 3) FindLine90R();
    else FindLine90L();
    break;

    case cubeCoarse:
    Spin90Right();
    PutCubeInBox();
    if(cubesReturned == 3) FindLine90L();
    else FindLine90R();
    break;
  }
}


//////////////////////////////////////////////////////////////////////

void setup()
{
  Serial.begin(9600);

  pinMode(linePinFL, INPUT);
  pinMode(linePinFR, INPUT);
  pinMode(linePinMid, INPUT);
  pinMode(rangeFinderPin, INPUT);
  pinMode(startButtonPin, INPUT);
  pinMode(flashingLedPin, OUTPUT);
  pinMode(coarseLed, OUTPUT);
  pinMode(fineLed, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  PincerServo.attach(10); 
  AFMS.begin();
  //currentStep = idling;
  digitalWrite(flashingLedPin, LOW);
  digitalWrite(coarseLed, LOW);
  digitalWrite(trigPin, LOW);
  
  ClosePincers(false);
}

//////////////////////////////////////////////////////////////////////


void loop()
{
  switch(currentStep)
  {
    case idling:  // Waiting in start box
    if(!CheckLine(startButtonPin))
    {
      cubesReturned = 0;
      //Serial.println(CheckLine(startButtonPin));
      delay(1000);
      OpenPincers();
      DriveForward();
      currentStep = leavingStart;
    }
    break;

    case leavingStart:  // Finding line
    if(CheckLine(linePinFL) || CheckLine(linePinFR))
    {
      currentStep = findingCube1or2;
    }
    break;

    case findingCube1or2:  // Driving towards either the first or second cube
    FollowLine();

    // Check IR sensor
    distanceSensorStatus = analogRead(rangeFinderPin);
    //Serial.println(distanceSensorStatus);
    if(!seenCube)
    {
      if(distanceSensorStatus > 300)
      {
        seenCube = true;
      }
    }
    else
    {
      if(distanceSensorStatus < 250)
      {
        seenCube = false;
        DriveForward();
        delay(400);
        CollectCube();
        currentStep = returningCube;
        checkLineMidDelay = CUBE_2_CHECK_MID_DELAY;
        checkingLineMid = 0;
        actionStartTime = millis();
      }
    }
    break;

    case findingCube3:
    FollowLine();
    switch(cube3Step)  // Figure out how many line junctions have been passed
    {
      case leavingJunc:
      if(millis() - actionStartTime > 5000)
      {
        cube3Step = lookingForCubeJunc;
      }
      break;

      case lookingForCubeJunc:
      if(CheckLine(linePinMid))
      {
        cube3Step = leavingCubeJunc;
      }
      break;

      case leavingCubeJunc:
      if(!CheckLine(linePinMid))
      {
        cube3Step = lookingForSquare;
      }
      break;

      case lookingForSquare:
      if(CheckLine(linePinMid))
      {
        actionStartTime = millis();
        cube3Step = drivingIntoSquare;
      }
      break;

      case drivingIntoSquare:
      if(millis() - actionStartTime >= 800)
      {
        CollectCube();
        currentStep = returningCube;
        checkLineMidDelay = CUBE_3_CHECK_MID_DELAY;
        checkingLineMid = 0;
        actionStartTime = millis();
      }
      break;
    }
    break;
    
    case returningCube:
    FollowLine();
    if(!checkingLineMid)
    {
      if(millis() - actionStartTime >= checkLineMidDelay)  // Delay rear sensor checking so it doesn't turn too soon
      {
        checkingLineMid = true;
      }
    }
    
    else
    {
      if(CheckLine(linePinMid))
      {
        DeliverCube();
        if(cubesReturned == 3)
        {
          ClosePincers(false);
          currentStep = finishing;
          checkLineMidDelay = FINISHING_CHECK_MID_DELAY;
          checkingLineMid = 0;
          actionStartTime = millis();
        }
        else if(cubesReturned == 2)
        {
          currentStep = findingCube3;
          cube3Step = leavingJunc;
          actionStartTime = millis();
        }
        else
        {
          currentStep = findingCube1or2;
        }
      }
    }
    break;

    case finishing:
    FollowLine();
    if(!checkingLineMid)
    {
      if(millis() - actionStartTime >= checkLineMidDelay)
      {
        checkingLineMid = true;
      }
    }
    
    else if(CheckLine(linePinMid))
    {
      DriveForward();  // Drive into box
      delay(500);
      StopMoving();
      currentStep = idling;
    }
    break;
  }
}
