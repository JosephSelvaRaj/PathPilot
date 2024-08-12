#include <RPLidar.h>
#include <SPI.h>
#include <NewPing.h>
#include "XGBoost.h"

// Pinout
#define ENA 2
#define ENB 3
#define RPLIDAR_MCONTRL 4
#define BUZZER_PIN 7
#define ECHO_PIN 8
#define TRIG_PIN 9
#define MOTORA_IN1 30
#define MOTORA_IN2 32
#define MOTORB_IN3 34
#define MOTORB_IN4 36

// Macros
#define MOTOR_STRAIGHT_SPEED 150  //Normal 120. Fast 150
#define MOTOR_TURNING_RATIO 0.55 //At 90, use 0.67, 120 use 0.55
#define LEFT_MOTOR_TUNE_DOWN_PERCENTAGE 0.99
#define ULTRASONIC_THRESHOLD 8
#define NUM_OF_FEATURES 240
#define OBSTACLE_DETECT_DISTANCE 140
#define OBSTACLE_DETECT_ANGLE_MIN 145
#define OBSTACLE_DETECT_ANGLE_MAX 210
#define ULTRASONIC_MAX_DISTANCE 50
#define LIDAR_RESOLUTION 240
#define LIDAR_SPEED 255
#define DISTANCE_MAX_THRESHOLD 2000

// Library Objects
RPLidar lidar;
Eloquent::ML::Port::XGBClassifier clf;
NewPing sonar(TRIG_PIN, ECHO_PIN, ULTRASONIC_MAX_DISTANCE);

// Global Variables
int lidarDataSelection[NUM_OF_FEATURES] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239};
float selectedData[NUM_OF_FEATURES];
int distanceBuffer[LIDAR_RESOLUTION];
int controlCmd = 0;
int distanceValue = 0;
int angleValue = 0;
int qualityValue = 0;
int ultrasonicDistance = 0;
bool obstacleDetected = false;
volatile bool ultraSonicPing = true;

void setup()
{
    // Pin and Serial initialization
    pinMode(ECHO_PIN, INPUT);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(MOTORA_IN1, OUTPUT);
    pinMode(MOTORA_IN2, OUTPUT);
    pinMode(MOTORB_IN3, OUTPUT);
    pinMode(MOTORB_IN4, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(RPLIDAR_MCONTRL, OUTPUT);
    Serial.begin(9600);    // Serial Monitor
    Serial2.begin(115200); // RPLidar
    lidar.begin(Serial2);
    while (!IS_OK(lidar.waitPoint()))
    {
        handleLidarFailure();
    }

    setupTimer1(); // CHanged to 400ms
}

void loop()
{
    if (ultraSonicPing)
    {
        ultrasonicDistance = sonar.ping_cm();
        if (ultrasonicDistance == 0)
        {
            ultrasonicDistance = ULTRASONIC_MAX_DISTANCE;
        }
        ultraSonicPing = false;
    }

    processLidarData();
}

ISR(TIMER1_COMPA_vect)
{
    // Every 400ms
    if (!obstacleDetected)
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
    ultraSonicPing = true;
}

void processLidarData()
{
    lidar.waitPoint();
    distanceValue = (int)lidar.getCurrentPoint().distance;
    angleValue = (int)lidar.getCurrentPoint().angle;
    qualityValue = (int)lidar.getCurrentPoint().quality;

    if (distanceValue < DISTANCE_MAX_THRESHOLD && qualityValue > 0)
    {
        int bufferIndex = angleIndexMap(angleValue);
        if (distanceValue <= 0)
        {
            distanceValue = distanceBuffer[bufferIndex - 1];
        }
        distanceBuffer[bufferIndex] = distanceValue;

        if ((distanceValue >= 128 && bufferIndex >= OBSTACLE_DETECT_ANGLE_MIN && bufferIndex <= OBSTACLE_DETECT_ANGLE_MAX && distanceValue <= OBSTACLE_DETECT_DISTANCE) || ultrasonicDistance <= ULTRASONIC_THRESHOLD)
        {
            stopMotorsAndBuzz();
            obstacleDetected = true;
            Serial.println("UltraSonic Distance: ");
            Serial.println(ultrasonicDistance);
            Serial.println("Lidar Distance: ");
            Serial.println(distanceValue);
        }
        else if ((distanceValue >= 128 && bufferIndex >= OBSTACLE_DETECT_ANGLE_MIN && bufferIndex <= OBSTACLE_DETECT_ANGLE_MAX && distanceValue > OBSTACLE_DETECT_DISTANCE) && ultrasonicDistance > ULTRASONIC_THRESHOLD)
        {
            obstacleDetected = false;
        }
    }

    if (!obstacleDetected)
    {
        digitalWrite(BUZZER_PIN, LOW);
    }
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
    // Configure Timer1 for 182ms
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

void stopMotorsAndBuzz()
{
    // Stop the motors
    digitalWrite(MOTORA_IN1, LOW);
    digitalWrite(MOTORA_IN2, LOW);
    digitalWrite(MOTORB_IN3, LOW);
    digitalWrite(MOTORB_IN4, LOW);
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);

    // Activate the buzzer
    digitalWrite(BUZZER_PIN, HIGH);
}