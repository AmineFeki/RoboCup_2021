#include "RoboClaw.h"
#include <SoftwareSerial.h>
#include "NewPing.h"

#define SENSOR_1    2
#define SENSOR_2    3
#define SENSOR_3    4
#define SENSOR_4    5
#define LED_RED     6
#define LED_GREEN   8
#define TRIGGER_PIN 9
#define ECHO_PIN   12
#define MAX_DISTANCE 400
#define BLACK       1
#define WHITE       0
#define SPEED1   5000
#define SPEED2  -5000
#define SPEED3   3000
#define SPEED4      0
#define SPEED5   1000
#define SPEED6  -5000
#define SPEED7 3500
#define address  0x80

static uint8_t ROBOT_LeftSensorState_u8 = 0, 
               ROBOT_LeftCenterSensorState_u8 = 0, 
               ROBOT_RightCenterSensorState_u8 = 0, 
               ROBOT_RightSensorState_u8 = 0, 
               ROBOT_CenterSensorState_u8 = 0;
static uint32_t ROBOT_StartTime_u32 = 0;
static float SOUND_Distance_f = 0.0;
static float SOUND_Duration_f = 0.0;
static float SOUND_SPEED_M_f = 0.0;
static float SOUND_SPEED_CM_f = 0.0;
float ROBOT_Distance_f = 0.0;
static bool ROBOT_TrafficCircle_b = false;
static bool ROBOT_TrafficCircleComplete_b = false;
static bool ROBOT_ObjectDetected_b = false;
static bool ROBOT_CircleLabyrinth_b = false;
static bool ROBOT_MovingSlopeStart_b = false;
const uint8_t ULTRASONIC_ITERATIONS = 5;

static uint32_t previousTime = 0;
static bool timeOut = false;

SoftwareSerial serial(10, 11);
RoboClaw roboclaw(&serial, 10000);
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

typedef enum
{
  ROBOT_Idle_en,
  ROBOT_BlackFollower_en,
  ROBOT_WhiteFollower_en,
  ROBOT_TrafficCircle_en,
  ROBOT_CircleLabyrith_en,
  ROBOT_Labyrinth_en,
  ROBOT_MovingSlope_en,
  ROBOT_ObjectDetection_en,
  ROBOT_Snake_en,
  ROBOT_Finish_en
} ROBOT_States_ten;

ROBOT_States_ten ROBOT_CurrentState_en = ROBOT_Idle_en;

void ROBOT_vReadSensors();

void ROBOT_vForward();
void ROBOT_vRight();
void ROBOT_vLeft();
void ROBOT_vBackward();

void ROBOT_vForward1();
void ROBOT_vRight1();
void ROBOT_vLeft1();
void ROBOT_vBackward1();
void ROBOT_vForward5();
void ROBOT_vRight5();
void ROBOT_vLeft5();
void ROBOT_vBackward5();

void ROBOT_vBlackFollower();
void ROBOT_vWhiteFollower();
void ROBOT_vTrafficCircle();
void ROBOT_vCircleLabyrinth();
void ROBOT_vLabyrinth();
void ROBOT_vMovingSlope();
void ROBOT_vObjectDetection();
float ROBOT_fReadDistance();
void ROBOT_vWhiteFollower1();

void setup() {
  // put your setup code here, to run once:
  ROBOT_StartTime_u32 = millis();
  roboclaw.begin(38400);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  ROBOT_vReadSensors();
  switch (ROBOT_CurrentState_en)
  {
    case ROBOT_Idle_en:
      {
        ROBOT_vForward();
        if (millis() - ROBOT_StartTime_u32 > 500)             /* time-out to start the process */
        {
          ROBOT_CurrentState_en = ROBOT_BlackFollower_en;
        }
      } break;
    case ROBOT_BlackFollower_en:
      {
        ROBOT_vBlackFollower();
        if ((ROBOT_LeftSensorState_u8 == BLACK) && 
        (ROBOT_RightSensorState_u8 == BLACK) && 
        (ROBOT_LeftCenterSensorState_u8 == BLACK) && 
        (ROBOT_RightCenterSensorState_u8 == BLACK))           
        {
          ROBOT_CurrentState_en = ROBOT_WhiteFollower_en;     /* Move to the next state */
         
        }
      } break;
    case ROBOT_WhiteFollower_en:
      {
        ROBOT_vWhiteFollower1();
        if (ROBOT_TrafficCircle_b == true)
        {
          ROBOT_CurrentState_en = ROBOT_TrafficCircle_en;     /* Move to the next state */
        }
      } break;
    case ROBOT_TrafficCircle_en:
      {
        ROBOT_vTrafficCircle();
        if (ROBOT_TrafficCircleComplete_b == true)
        {
          ROBOT_CurrentState_en = ROBOT_CircleLabyrith_en;    /* Move to the next state */
        }
      } break;
    case ROBOT_CircleLabyrith_en:
      {
        ROBOT_vCircleLabyrinth();
        if (ROBOT_CircleLabyrinth_b == true)
        {
          ROBOT_CurrentState_en = ROBOT_Labyrinth_en;         /* Move to the next state */
        }
      } break;
    case ROBOT_Labyrinth_en:
      {
        ROBOT_vLabyrinth();
        if (ROBOT_MovingSlopeStart_b == true)
        {
          ROBOT_CurrentState_en = ROBOT_MovingSlope_en;       /* Move to the next state */
        }
      } break;
    case ROBOT_MovingSlope_en:
      {
        ROBOT_vMovingSlope();
        if ((ROBOT_LeftSensorState_u8 == BLACK) && 
        (ROBOT_RightSensorState_u8 == BLACK) && 
        (ROBOT_LeftCenterSensorState_u8 == BLACK) && 
        (ROBOT_RightCenterSensorState_u8 == BLACK))
        {
          roboclaw.SpeedM1M2(address, 0, 0);
          delay(2000);                                        /* Here the robot is invited to stop for 2 seconds */
          ROBOT_CurrentState_en = ROBOT_ObjectDetection_en;   /* Move to the next state */
        }
      } break;
    case ROBOT_ObjectDetection_en:
      {
        ROBOT_vObjectDetection();
        if (ROBOT_ObjectDetected_b == true)
        {
          //roboclaw.SpeedM1M2(address, 0, 0);
          digitalWrite(LED_RED, HIGH);                        /* Here the robot is invited to switch on the RED LED */
          delay(5000);                                        /* For 5 seconds */
          digitalWrite(LED_RED, LOW);                         /* Here the robot is invited to switch offf the RED LED */
          digitalWrite(LED_GREEN, HIGH);                      /* Here the robot is invited to switch on the GREEN LED */
          delay(2000);                                        /* For 2 seconds */
          digitalWrite(LED_GREEN, LOW);                       /* Here the robot is invited to switch off the GREEN LED */
          ROBOT_CurrentState_en = ROBOT_Snake_en;             /* Move to the next state */
        }

      } break;
    
    case ROBOT_Snake_en:
      {
        ROBOT_vBlackFollower();
      } break;
    default:
      {

      } break;
  }
}

void ROBOT_vReadSensors()
{
  ROBOT_LeftSensorState_u8 = digitalRead(SENSOR_1);
  ROBOT_LeftCenterSensorState_u8 = digitalRead(SENSOR_2);
  ROBOT_RightCenterSensorState_u8 = digitalRead(SENSOR_3);
  ROBOT_RightSensorState_u8 = digitalRead(SENSOR_4);
}
void ROBOT_vForward1()
{
  roboclaw.SpeedM1M2(address, SPEED5, SPEED5);
}

void ROBOT_vLeft1()
{
  roboclaw.SpeedM1M2(address, SPEED4, SPEED5);
}

void ROBOT_vRight1()
{
  roboclaw.SpeedM1M2(address, SPEED5, SPEED4);
}

void ROBOT_vBackward1()
{
  roboclaw.SpeedM1M2(address, SPEED2, SPEED2);
}

void ROBOT_vForward()
{
  roboclaw.SpeedM1M2(address, SPEED1, SPEED1);
}

void ROBOT_vLeft()
{
  roboclaw.SpeedM1M2(address, SPEED4, SPEED1);
}

void ROBOT_vLeftspec()
{
  roboclaw.SpeedM1M2(address, SPEED6, SPEED1);
}

void ROBOT_vRight()
{
  roboclaw.SpeedM1M2(address, SPEED1, SPEED4);
}

void ROBOT_vRightspec()
{
  roboclaw.SpeedM1M2(address, SPEED1, SPEED6);
}

void ROBOT_vBackward()
{
  roboclaw.SpeedM1M2(address, SPEED2, SPEED2);
}

void ROBOT_vBlackFollower()
{
  if ((ROBOT_LeftCenterSensorState_u8 == BLACK) && 
  (ROBOT_RightCenterSensorState_u8 == BLACK) && 
  (ROBOT_RightSensorState_u8 == BLACK) && 
  ( ROBOT_CurrentState_en ==ROBOT_BlackFollower_en) )
  {
    ROBOT_vRight();
  }
  else if ((ROBOT_LeftCenterSensorState_u8 == BLACK) && (ROBOT_RightCenterSensorState_u8 == BLACK))
  {
    ROBOT_vForward();
  }
  else if ((ROBOT_LeftCenterSensorState_u8 == WHITE) && (ROBOT_RightCenterSensorState_u8 == BLACK))
  {
    ROBOT_vRight();
  }
  else if ((ROBOT_LeftCenterSensorState_u8 == BLACK) && (ROBOT_RightCenterSensorState_u8 == WHITE))
  {
    ROBOT_vLeft();
  }
  else
  {
    ROBOT_vForward();
  }
}

void ROBOT_vWhiteFollower()
{
  if ((ROBOT_LeftCenterSensorState_u8 == WHITE) && 
  (ROBOT_RightCenterSensorState_u8 == WHITE) && 
  (ROBOT_LeftSensorState_u8 == WHITE) && 
  (ROBOT_RightSensorState_u8 == WHITE))
  {
    if (ROBOT_TrafficCircle_b == false)
    {
      ROBOT_vRight();
      delay(300);         /* to change with encoder steps */
      ROBOT_TrafficCircle_b = true;
    }
  }
  else if ((ROBOT_LeftCenterSensorState_u8 == WHITE) && (ROBOT_RightCenterSensorState_u8 == WHITE))
  {
    ROBOT_vForward();
  }
  else if ((ROBOT_LeftCenterSensorState_u8 == BLACK) && (ROBOT_RightCenterSensorState_u8 == WHITE))
  {
    ROBOT_vRight();
  }
  else if ((ROBOT_LeftCenterSensorState_u8 == WHITE) && (ROBOT_RightCenterSensorState_u8 == BLACK))
  {
    ROBOT_vLeft();
  }
  else
  {
    ROBOT_vForward();
  }
}

void ROBOT_vTrafficCircle()
{
  if ((ROBOT_LeftCenterSensorState_u8 == WHITE) && 
  (ROBOT_RightCenterSensorState_u8 == WHITE) && 
  (ROBOT_LeftSensorState_u8 == WHITE) && 
  (ROBOT_RightSensorState_u8 == WHITE))
  {
    ROBOT_vRight();
    delay(300); /* to change with encoder steps */
    ROBOT_TrafficCircleComplete_b = true;
  }
  else if ((ROBOT_LeftCenterSensorState_u8 == WHITE) && (ROBOT_RightCenterSensorState_u8 == WHITE))
  {
    ROBOT_vForward();
  }
  else if ((ROBOT_LeftCenterSensorState_u8 == BLACK) && (ROBOT_RightCenterSensorState_u8 == WHITE))
  {
    ROBOT_vRight();
  }
  else if ((ROBOT_LeftCenterSensorState_u8 == WHITE) && (ROBOT_RightCenterSensorState_u8 == BLACK))
  {
    ROBOT_vLeft();
  }
  else
  {
    ROBOT_vBackward();
  }
}

void ROBOT_vCircleLabyrinth()
{
  if ((ROBOT_LeftCenterSensorState_u8 == WHITE) && 
  (ROBOT_RightCenterSensorState_u8 == WHITE) && 
  (ROBOT_LeftSensorState_u8 == WHITE) && 
  (ROBOT_RightSensorState_u8 == WHITE))
  {
    ROBOT_CircleLabyrinth_b = true;
  }
  else if ((ROBOT_LeftCenterSensorState_u8 == WHITE) && (ROBOT_RightCenterSensorState_u8 == WHITE))
  {
    ROBOT_vForward();
  }
  else if ((ROBOT_LeftCenterSensorState_u8 == BLACK) && (ROBOT_RightCenterSensorState_u8 == WHITE))
  {
    ROBOT_vRight();
  }
  else if ((ROBOT_LeftCenterSensorState_u8 == WHITE) && (ROBOT_RightCenterSensorState_u8 == BLACK))
  {
    ROBOT_vLeft();
  }
  else
  {
    ROBOT_vForward();
  }
}

void ROBOT_vLabyrinth()
{
  ROBOT_vRight();
  delay(500);                /* Keep rotating for 0.5 seconds to make a 90° */
                             /* This is possible and scientifically correct as the velocity is known */
  ROBOT_vReadSensors();
  while (ROBOT_LeftSensorState_u8 == BLACK)
  {
    ROBOT_vReadSensors();
    ROBOT_vWhiteFollower();
  }
  ROBOT_vLeft();
  delay(500);                /* Keep rotating for 0.5 seconds to make a 90° */
                             /* This is possible and scientifically correct as the velocity is known */
  ROBOT_vReadSensors();
  while (ROBOT_RightSensorState_u8 == BLACK)
  {
    ROBOT_vReadSensors();
    ROBOT_vWhiteFollower();
  }
  ROBOT_vRight();
  delay(500);                /* Keep rotating for 0.5 seconds to make a 90° */
                             /* This is possible and scientifically correct as the velocity is known */
  while (!((ROBOT_LeftSensorState_u8 == WHITE) && 
  (ROBOT_RightSensorState_u8 == WHITE) && 
  (ROBOT_LeftCenterSensorState_u8 == WHITE) && 
  (ROBOT_RightCenterSensorState_u8 == WHITE)))
  {
    ROBOT_vReadSensors();
    ROBOT_vWhiteFollower();
  }
  ROBOT_MovingSlopeStart_b = true;
}

void ROBOT_vMovingSlope()
{
  if ((ROBOT_LeftCenterSensorState_u8 == BLACK) && (ROBOT_RightCenterSensorState_u8 == BLACK))
  {
    ROBOT_vForward1();
  }
  else if ((ROBOT_LeftCenterSensorState_u8 == WHITE) && (ROBOT_RightCenterSensorState_u8 == BLACK))
  {
    ROBOT_vRight1();
  }
  else if ((ROBOT_LeftCenterSensorState_u8 == BLACK) && (ROBOT_RightCenterSensorState_u8 == WHITE))
  {
    ROBOT_vLeft1();
  }
  else
  {
    ROBOT_vForward1();
  }
}

void ROBOT_vObjectDetection()
{ 
  while ( (ROBOT_LeftSensorState_u8 == BLACK) ) {
    ROBOT_vReadSensors();
    ROBOT_vMovingSlope();
  }
  while ( (ROBOT_LeftSensorState_u8 == WHITE) ) {
    ROBOT_vReadSensors();
    ROBOT_vMovingSlope();
  }
  
  roboclaw.SpeedM1M2(address,0,0);
  ROBOT_Distance_f = ROBOT_fReadDistance();
  //Serial.println(ROBOT_Distance_f);
  if (  (ROBOT_Distance_f > 30.0) || (ROBOT_Distance_f ==0) ) 
  {
        ROBOT_ObjectDetected_b = true;
  }
}

float ROBOT_fReadDistance()
{
  //delay(400);
  SOUND_SPEED_M_f = 341.0;
  SOUND_SPEED_CM_f =SOUND_SPEED_M_f /10000;
  SOUND_Duration_f = sonar.ping_median(ULTRASONIC_ITERATIONS);
  SOUND_Distance_f = (SOUND_Duration_f / 2) * SOUND_SPEED_CM_f ;
  //Serial.println(SOUND_Distance_f);
  if( (SOUND_Distance_f <= 400.0) || (SOUND_Distance_f >= 2.0) ){
    return SOUND_Distance_f;
    }
  else return 0.0;
}

void ROBOT_vWhiteFollower1()
{
  if ((ROBOT_LeftCenterSensorState_u8 == WHITE) && 
  (ROBOT_RightCenterSensorState_u8 == WHITE) && 
  (ROBOT_LeftSensorState_u8 == WHITE) && 
  (ROBOT_RightSensorState_u8 == WHITE))
  {
    if (ROBOT_TrafficCircle_b == false)
    {
      ROBOT_vRight5();
      delay(300);         /* to change with encoder steps */
      ROBOT_TrafficCircle_b = true;
    }
  }
  else if ((ROBOT_LeftCenterSensorState_u8 == WHITE) && (ROBOT_RightCenterSensorState_u8 == WHITE))
  {
    ROBOT_vForward5();
  }
  else if ((ROBOT_LeftCenterSensorState_u8 == BLACK) && (ROBOT_RightCenterSensorState_u8 == WHITE))
  {
    ROBOT_vRight5();
  }
  else if ((ROBOT_LeftCenterSensorState_u8 == WHITE) && (ROBOT_RightCenterSensorState_u8 == BLACK))
  {
    ROBOT_vLeft5();
  }
  else
  {
    ROBOT_vForward5();
  }
}

void ROBOT_vForward5()
{
  roboclaw.SpeedM1M2(address, SPEED7, SPEED7);
}

void ROBOT_vLeft5()
{
  roboclaw.SpeedM1M2(address, SPEED4, SPEED7);
}

void ROBOT_vRight5()
{
  roboclaw.SpeedM1M2(address, SPEED7, SPEED4);
}

void ROBOT_vBackward5()
{
  roboclaw.SpeedM1M2(address, SPEED2, SPEED2);
}
