#include <RPLidar.h>
#include <SPI.h>
#include <NewPing.h>
#include "randomForest.h"

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
#define MOTOR_TURNING_RATIO 0.73
#define LEFT_MOTOR_TUNE_DOWN_PERCENTAGE 0.97
#define ULTRASONIC_THRESHOLD 5
#define NUM_OF_FEATURES 80
#define MOTOR_STRAIGHT_SPEED 80
#define OBSTACLE_DETECT_DISTANCE 150
#define OBSTACLE_DETECT_ANGLE_MIN 165
#define OBSTACLE_DETECT_ANGLE_MAX 195
#define ULTRASONIC_MAX_DISTANCE 200
#define LIDAR_RESOLUTION 360
#define LIDAR_SPEED 255

// Library Objects
RPLidar lidar;
Eloquent::ML::Port::RandomForest clf;
NewPing sonar(TRIG_PIN, ECHO_PIN, ULTRASONIC_MAX_DISTANCE);
int lidarDataSelection[NUM_OF_FEATURES] = {46, 47, 48, 50, 51, 53, 54, 100, 102, 103, 104, 107, 132, 207,
                                           209, 211, 212, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234,
                                           235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248,
                                           249, 250, 251, 252, 253, 254, 255, 256, 257, 258, 301, 302, 303, 306,
                                           307, 308, 309, 310, 311, 312, 313, 314, 315, 316, 317, 318, 319, 320,
                                           321, 322, 323, 324, 325, 326, 327, 328, 329, 330};
float selectedData[NUM_OF_FEATURES];
int distanceBuffer[LIDAR_RESOLUTION];
int controlCmd;
int distanceValue = 0;
int angleValue = 0;
int qualityValue = 0;
int ultrasonicDistance = 0;
bool obstacleDetected = false;

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
    if (IS_OK(lidar.waitPoint())) //
    {
        // Lidar is working
    }
    else
    {
        // Lidar is not working
        handleLidarFailure();
    }
    setupTimer1(); // CHanged to 400ms
}

void loop()
{
    ultrasonicDistance = sonar.ping_cm();
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
    // Configure Timer1 for 400ms
    cli(); // Disable all interrupts for register configuration
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
    // 2.5 Hz (16000000/((6249+1)*1024))
    OCR1A = 6249;
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

    if (qualityValue > 0)
    {
        int bufferIndex = angleIndexMap(angleValue);
        if (distanceValue == 0)
        {
            distanceValue = distanceBuffer[bufferIndex - 1];
        }
        distanceBuffer[bufferIndex] = distanceValue;

        if ((bufferIndex >= OBSTACLE_DETECT_ANGLE_MIN && bufferIndex <= OBSTACLE_DETECT_ANGLE_MAX && distanceValue <= OBSTACLE_DETECT_DISTANCE) || ultrasonicDistance <= ULTRASONIC_THRESHOLD)
        {
            stopMotorsAndBuzz();
            obstacleDetected = true;
            Serial.println("UltraSonic Distance: ");
            Serial.println(ultrasonicDistance);
            Serial.println("Lidar Distance: ");
            Serial.println(distanceValue);
        }
        else if ((bufferIndex >= OBSTACLE_DETECT_ANGLE_MIN && bufferIndex <= OBSTACLE_DETECT_ANGLE_MAX && distanceValue > OBSTACLE_DETECT_DISTANCE) && ultrasonicDistance > ULTRASONIC_THRESHOLD)
        {
            obstacleDetected = false;
        }

        if (!obstacleDetected)
        {
            digitalWrite(BUZZER_PIN, LOW);
        }
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