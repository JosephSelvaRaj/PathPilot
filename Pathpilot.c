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
#define MOTOR_STRAIGHT_SPEED 90
#define LIDAR_RESOLUTION 240
#define DISTANCE_MAX_THRESHOLD 2000
#define LIDAR_SPEED 255
#define LEFT_MOTOR_TUNE_DOWN_PERCENTAGE 0.97
#define MOTOR_TURNING_RATIO 0.73
#define NUM_OF_FEATURES 80

// Library Objects
RPLidar lidar;

Eloquent::ML::Port::RandomForest clf;
int lidarDataSelection[NUM_OF_FEATURES] = {c};
float selectedData[NUM_OF_FEATURES];
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
    Serial2.begin(115200); // RPLidar
    lidar.begin(Serial2);
    if (IS_OK(lidar.waitPoint()))
    {
        // Lidar is working
    }
    else
    {
        // Lidar is not working
        handleLidarFailure();
    }
    setupTimer1();
}

void loop()
{
    processLidarData();
}

ISR(TIMER1_COMPA_vect)
{
    for (int i = 0; i < NUM_OF_FEATURES; i++)
    {
        selectedData[i] = distanceBuffer[lidarDataSelection[i]];
    }
    controlCmd = clf.predict(selectedData);
    moveMotors(controlCmd);
    resetdataBuffer();
    resetDistanceBuffer();
}

void resetDistanceBuffer()
{
    for (int i = 0; i < LIDAR_RESOLUTION; i++)
    {
        distanceBuffer[i] = 0;
    }
}

void resetdataBuffer()
{
    for (int i = 0; i < NUM_OF_FEATURES; i++)
    {
        selectedData[i] = 0;
    }
}

void setupTimer1(void)
{
    // Configure Timer1 for 200ms
    cli(); // Disable all interrupts for register configuration
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
    // 200ms (16000000/((3124+1)*1024))
    OCR1A = 3124;
    // CTC
    TCCR1B |= (1 << WGM12);
    // Prescaler 1024
    TCCR1B |= (1 << CS12) | (1 << CS10);
    // Output Compare Match A Interrupt Enable
    TIMSK1 |= (1 << OCIE1A);
    sei(); // Enable global interrupts
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
    lidar.waitPoint();
    distanceValue = (int)lidar.getCurrentPoint().distance;
    angleValue = (int)lidar.getCurrentPoint().angle;
    qualityValue = (int)lidar.getCurrentPoint().quality;
    if (distanceValue <= DISTANCE_MAX_THRESHOLD && qualityValue > 0)
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
        // Forward
        digitalWrite(MOTORA_IN1, HIGH);
        digitalWrite(MOTORA_IN2, LOW);
        digitalWrite(MOTORB_IN3, HIGH);
        digitalWrite(MOTORB_IN4, LOW);
        analogWrite(ENA, MOTOR_STRAIGHT_SPEED);
        analogWrite(ENB, MOTOR_STRAIGHT_SPEED * LEFT_MOTOR_TUNE_DOWN_PERCENTAGE);
        break;

    case 1:
        // Left
        digitalWrite(MOTORA_IN1, HIGH);
        digitalWrite(MOTORA_IN2, LOW);
        digitalWrite(MOTORB_IN3, HIGH);
        digitalWrite(MOTORB_IN4, LOW);
        analogWrite(ENA, MOTOR_STRAIGHT_SPEED);
        analogWrite(ENB, MOTOR_STRAIGHT_SPEED * MOTOR_TURNING_RATIO * LEFT_MOTOR_TUNE_DOWN_PERCENTAGE);
        break;

    case 2:
        // Right
        digitalWrite(MOTORA_IN1, HIGH);
        digitalWrite(MOTORA_IN2, LOW);
        digitalWrite(MOTORB_IN3, HIGH);
        digitalWrite(MOTORB_IN4, LOW);
        analogWrite(ENA, MOTOR_STRAIGHT_SPEED * MOTOR_TURNING_RATIO);
        analogWrite(ENB, MOTOR_STRAIGHT_SPEED * LEFT_MOTOR_TUNE_DOWN_PERCENTAGE);
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