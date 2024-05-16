#include <RPLidar.h>
#include <SPI.h>
#include "randomForest.h"

// Pinout
#define ENA 2
#define ENB 3
#define RPLIDAR_MCONTRL 4
#define MOTORA_IN1 30
#define MOTORA_IN2 32
#define MOTORB_IN3 34
#define MOTORB_IN4 36

// Macros
#define MOTOR_STRAIGHT_SPEED 80
#define LIDAR_RESOLUTION 240
#define LIDAR_SPEED 255
#define DISTANCE_MAX_THRESHOLD 4000          // in mm
#define LEFT_MOTOR_TUNE_DOWN_PERCENTAGE 0.93 // At 70 straight speed with 0.93 left motor tune down, the robot moves straight. Not effective at 150 straight speed.
#define MOTOR_TURNING_RATIO 0.6

// Library Objects
RPLidar lidar;

Eloquent::ML::Port::RandomForest clf;
int lidarDataSelection[80] = {88, 90, 130, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142,
                              143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156,
                              157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170,
                              171, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202,
                              203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216,
                              217, 218, 219, 220, 221, 222, 223, 224, 225, 226};
float selectedData[80];
int distanceBuffer[LIDAR_RESOLUTION];
int controlCmd;
int distanceValue = 0;
int angleValue = 0;
int qualityValue = 0;

void setup()
{
  // Pin and Serial initialization
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(MOTORA_IN1, OUTPUT);
  pinMode(MOTORA_IN2, OUTPUT);
  pinMode(MOTORB_IN3, OUTPUT);
  pinMode(MOTORB_IN4, OUTPUT);
  pinMode(RPLIDAR_MCONTRL, OUTPUT);
  Serial1.begin(9600);   // Bluetooth Module
  Serial2.begin(115200); // RPLidar
  lidar.begin(Serial2);
}

void loop()
{
  if (IS_OK(lidar.waitPoint()))
  {
    // Lidar is working
    processLidarData();
  }
  else
  {
    // Lidar is not working
    handleLidarFailure();
  }

  for (int i = 0; i < 80; i++)
  {
    selectedData[i] = distanceBuffer[lidarDataSelection[i]];
  }
  controlCmd = clf.predict(selectedData);
  moveMotors(controlCmd);
}

int angleIndexMap(int angle)
{
  // Subtract 90 degrees to offset the angles clockwise
  angle -= 90;

  // Ensure the angle stays within the valid range (0-360)
  if (angle < 0)
  {
    angle += 360;
  }
  else if (angle >= 360)
  {
    angle -= 360;
  }

  // Map the angle from 0-360 degrees to 0-239 (LIDAR_RESOLUTION)
  int index = int(map(angle, 0, 360, 0, LIDAR_RESOLUTION));

  return index;
}
void processLidarData()
{
  distanceValue = (int)lidar.getCurrentPoint().distance;
  angleValue = (int)lidar.getCurrentPoint().angle;
  qualityValue = (int)lidar.getCurrentPoint().quality;

  if (distanceValue < DISTANCE_MAX_THRESHOLD && qualityValue > 0)
  {
    int bufferIndex = angleIndexMap(angleValue);
    if (distanceValue == 0)
    {
      // If the distance value is 0, use the previous angle value
      distanceValue = distanceBuffer[bufferIndex - 1];
    }
    distanceBuffer[bufferIndex] = distanceValue;
  }
}

void handleLidarFailure()
{
  analogWrite(RPLIDAR_MCONTRL, 0); // Stop the RPLIDAR motor
  rplidar_response_device_info_t info;
  if (IS_OK(lidar.getDeviceInfo(info, 100)))
  {
    // Detected
    lidar.startScan();
    analogWrite(RPLIDAR_MCONTRL, LIDAR_SPEED);
    delay(1000);
  }
}

void moveMotors(int cmd)
{
  /******L298n Truth Table**********/
  // Motor A forward  | Motor A backward
  // In1 = HIGH       | In1 = LOW
  // In2 = LOW        | In2 = HIGH
  // Motor B forward  | Motor B backward
  // In3 = HIGH       | In3 = LOW
  // In4 = LOW        | In4 = HIGH
  // ENA = Right PWM  | ENB = Left PWM
  /*********************************/
  switch (cmd)
  {
  case 0:
    digitalWrite(MOTORA_IN1, HIGH);
    digitalWrite(MOTORA_IN2, LOW);
    digitalWrite(MOTORB_IN3, HIGH);
    digitalWrite(MOTORB_IN4, LOW);
    analogWrite(ENA, MOTOR_STRAIGHT_SPEED);
    analogWrite(ENB, MOTOR_STRAIGHT_SPEED * LEFT_MOTOR_TUNE_DOWN_PERCENTAGE);
    break;

  case 2:
    digitalWrite(MOTORA_IN1, HIGH);
    digitalWrite(MOTORA_IN2, LOW);
    digitalWrite(MOTORB_IN3, HIGH);
    digitalWrite(MOTORB_IN4, LOW);
    analogWrite(ENA, MOTOR_STRAIGHT_SPEED * MOTOR_TURNING_RATIO);
    analogWrite(ENB, MOTOR_STRAIGHT_SPEED * LEFT_MOTOR_TUNE_DOWN_PERCENTAGE);
    break;

  case 1:
    digitalWrite(MOTORA_IN1, HIGH);
    digitalWrite(MOTORA_IN2, LOW);
    digitalWrite(MOTORB_IN3, HIGH);
    digitalWrite(MOTORB_IN4, LOW);
    analogWrite(ENA, MOTOR_STRAIGHT_SPEED);
    analogWrite(ENB, MOTOR_STRAIGHT_SPEED * MOTOR_TURNING_RATIO * LEFT_MOTOR_TUNE_DOWN_PERCENTAGE);
    break;

  default:
    digitalWrite(MOTORA_IN1, LOW);
    digitalWrite(MOTORA_IN2, LOW);
    digitalWrite(MOTORB_IN3, LOW);
    digitalWrite(MOTORB_IN4, LOW);
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
    break;
  }
}