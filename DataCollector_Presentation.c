#include <RPLidar.h>
#include <SD.h>
#include <SPI.h>

// Pinout
#define ENA 2
#define ENB 3
#define RPLIDAR_MCONTRL 4
#define MOTORA_IN1 30
#define MOTORA_IN2 32
#define MOTORB_IN3 34
#define MOTORB_IN4 36
#define SD_CS 53

// Macros
#define MOTOR_STRAIGHT_SPEED 90
#define LIDAR_RESOLUTION 360
#define LIDAR_SPEED 255
#define DATA_RECORDING_ENABLED 'D'
#define DATA_RECORDING_DISABLED 'd'
#define ROBOT_FORWARD 'F'
#define ROBOT_BACKWARD 'b'
#define ROBOT_STOP 's'
#define ROBOT_FORWARDRIGHT 'R'
#define ROBOT_FORWARDLEFT 'L'
#define ROBOT_PIVOTRIGHT 'r'
#define ROBOT_PIVOTLEFT 'l'
#define ROBOT_BACKWARDLEFT 'm'
#define ROBOT_BACKWARDRIGHT 'n'
#define LEFT_MOTOR_TUNE_DOWN_PERCENTAGE 0.97
#define MOTOR_TURNING_RATIO 0.73

// Global Variables
char ControlCmd;
bool dataRecordingFlag = false;
volatile int distanceBuffer[LIDAR_RESOLUTION];
int distanceValue = 0;
int angleValue = 0;
int qualityValue = 0;

// Library Objects
RPLidar lidar;
File dataFile;

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

  // SD card initialization
  while (!SD.begin(SD_CS))
  {
    // Handler for SD card initialization failure
  }

  // RPLidar initialization
  lidar.begin(Serial2);

  setupTimer1();
}

void loop()
{
  if (dataRecordingFlag)
  {
    // Process data from lidar only when data recording is enabled
    processLidarData();
  }

  if (Serial1.available())
  {
    ControlCmd = Serial1.read();

    dataRecordingFlag = (ControlCmd == DATA_RECORDING_ENABLED) ? true : ((ControlCmd == DATA_RECORDING_DISABLED) ? false : dataRecordingFlag);
  }

  moveMotors(ControlCmd);
}

ISR(TIMER1_COMPA_vect)
{
  // distance buff conversion to string
  if (dataRecordingFlag)
  {
    String dataString = "";
    for (int i = 0; i < LIDAR_RESOLUTION; i++)
    {
      // Store the distance values in the data string
      dataString.concat(distanceBuffer[i]);
      dataString.concat(",");
    }
    dataString.concat(ControlCmd);
    File dataFile = SD.open("data.txt", FILE_WRITE);

    if (dataFile)
    {
      // Write the data string to the data file if it is open
      dataFile.println(dataString);
      dataFile.close();
    }
    else
    {
      // Error opening the data file
    }
    resetDistanceBuffer();
  }
}

void processLidarData()
{
  lidar.waitPoint();
  distanceValue = (int)lidar.getCurrentPoint().distance;
  angleValue = (int)lidar.getCurrentPoint().angle;
  qualityValue = (int)lidar.getCurrentPoint().quality;
  if (qualityValue > 0)
  {
    // Get the buffer index for the angle value
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

void resetDistanceBuffer()
{
  for (int i = 0; i < LIDAR_RESOLUTION; i++)
  {
    distanceBuffer[i] = 0;
  }
}

void moveMotors(char cmd)
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
  case ROBOT_FORWARD:
    digitalWrite(MOTORA_IN1, HIGH);
    digitalWrite(MOTORA_IN2, LOW);
    digitalWrite(MOTORB_IN3, HIGH);
    digitalWrite(MOTORB_IN4, LOW);
    analogWrite(ENA, MOTOR_STRAIGHT_SPEED);
    analogWrite(ENB, MOTOR_STRAIGHT_SPEED * LEFT_MOTOR_TUNE_DOWN_PERCENTAGE);
    break;

  case ROBOT_BACKWARD:
    digitalWrite(MOTORA_IN1, LOW);
    digitalWrite(MOTORA_IN2, HIGH);
    digitalWrite(MOTORB_IN3, LOW);
    digitalWrite(MOTORB_IN4, HIGH);
    analogWrite(ENA, MOTOR_STRAIGHT_SPEED);
    analogWrite(ENB, MOTOR_STRAIGHT_SPEED * LEFT_MOTOR_TUNE_DOWN_PERCENTAGE);
    break;

  case ROBOT_STOP:
    digitalWrite(MOTORA_IN1, LOW);
    digitalWrite(MOTORA_IN2, LOW);
    digitalWrite(MOTORB_IN3, LOW);
    digitalWrite(MOTORB_IN4, LOW);
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
    break;

  case ROBOT_FORWARDRIGHT:
    digitalWrite(MOTORA_IN1, HIGH);
    digitalWrite(MOTORA_IN2, LOW);
    digitalWrite(MOTORB_IN3, HIGH);
    digitalWrite(MOTORB_IN4, LOW);
    analogWrite(ENA, MOTOR_STRAIGHT_SPEED * MOTOR_TURNING_RATIO);
    analogWrite(ENB, MOTOR_STRAIGHT_SPEED * LEFT_MOTOR_TUNE_DOWN_PERCENTAGE);
    break;

  case ROBOT_FORWARDLEFT:
    digitalWrite(MOTORA_IN1, HIGH);
    digitalWrite(MOTORA_IN2, LOW);
    digitalWrite(MOTORB_IN3, HIGH);
    digitalWrite(MOTORB_IN4, LOW);
    analogWrite(ENA, MOTOR_STRAIGHT_SPEED);
    analogWrite(ENB, MOTOR_STRAIGHT_SPEED * MOTOR_TURNING_RATIO * LEFT_MOTOR_TUNE_DOWN_PERCENTAGE);
    break;

  case ROBOT_BACKWARDLEFT:
    digitalWrite(MOTORA_IN1, LOW);
    digitalWrite(MOTORA_IN2, HIGH);
    digitalWrite(MOTORB_IN3, LOW);
    digitalWrite(MOTORB_IN4, HIGH);
    analogWrite(ENA, MOTOR_STRAIGHT_SPEED);
    analogWrite(ENB, MOTOR_STRAIGHT_SPEED * MOTOR_TURNING_RATIO * LEFT_MOTOR_TUNE_DOWN_PERCENTAGE);
    break;

  case ROBOT_BACKWARDRIGHT:
    digitalWrite(MOTORA_IN1, LOW);
    digitalWrite(MOTORA_IN2, HIGH);
    digitalWrite(MOTORB_IN3, LOW);
    digitalWrite(MOTORB_IN4, HIGH);
    analogWrite(ENA, MOTOR_STRAIGHT_SPEED * MOTOR_TURNING_RATIO);
    analogWrite(ENB, MOTOR_STRAIGHT_SPEED * LEFT_MOTOR_TUNE_DOWN_PERCENTAGE);
    break;

  case ROBOT_PIVOTLEFT:
    digitalWrite(MOTORA_IN1, HIGH);
    digitalWrite(MOTORA_IN2, LOW);
    digitalWrite(MOTORB_IN3, LOW);
    digitalWrite(MOTORB_IN4, HIGH);
    analogWrite(ENA, MOTOR_STRAIGHT_SPEED);
    analogWrite(ENB, MOTOR_STRAIGHT_SPEED * LEFT_MOTOR_TUNE_DOWN_PERCENTAGE);
    break;

  case ROBOT_PIVOTRIGHT:
    digitalWrite(MOTORA_IN1, LOW);
    digitalWrite(MOTORA_IN2, HIGH);
    digitalWrite(MOTORB_IN3, HIGH);
    digitalWrite(MOTORB_IN4, LOW);
    analogWrite(ENA, MOTOR_STRAIGHT_SPEED);
    analogWrite(ENB, MOTOR_STRAIGHT_SPEED * LEFT_MOTOR_TUNE_DOWN_PERCENTAGE);
    break;

  case DATA_RECORDING_ENABLED:
    if (IS_OK(lidar.waitPoint()))
    {
      // Lidar is working
    }
    else
    {
      // Lidar is not working
      handleLidarFailure();
    }
    break;

  case DATA_RECORDING_DISABLED:
    lidar.stop();
    analogWrite(RPLIDAR_MCONTRL, 0);
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

void setupTimer1(void)
{
  // Configure Timer1 for 180ms
  cli(); // Disable all interrupts for register configuration
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  // 5.5 Hz (16000000/((45453+1)*64))
  OCR1A = 45453;
  // CTC
  TCCR1B |= (1 << WGM12);
  // Prescaler 64
  TCCR1B |= (1 << CS11) | (1 << CS10);
  // Output Compare Match A Interrupt Enable
  TIMSK1 |= (1 << OCIE1A);
  sei(); // Enable global interrupts
}