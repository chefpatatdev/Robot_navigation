#include <Arduino.h>
#include <PID_v1.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <RPLidar.h>

// Encoder pins
#define encoderAint 21
#define encoderAana 48
#define encoderBint 20
#define encoderBana 49

// Motor driver pins
#define enA 13
#define in1 11
#define in2 12

#define enB 8
#define in3 10
#define in4 9

// RGB
#define redPin 5
#define greenPin 7
#define bluePin 6

// temperature sensor
#define ONE_WIRE_BUS 4
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Lidar
#define RPLIDAR_MOTOR 3
RPLidar lidar;
#define cone 15
#define savedistance 20
float minDistance = 100000;
float angleAtMinDist = 0;
int objectCounter = 0;
float distancel = 0;
float anglel = 0;

#define MAX_PID_VALUE 250

int currentInstructionCode = 0;

const int chargeT = 45;
const int workingT = 60;

// positioning
double coordX = 0.0;     // current position
double coordY = 0.0;     // current position
double targetCoordX = 0; // target position
double targetCoordY = 0; // target position
double distanceToTarget = 0;
double turnAngleToTarget = 0;
double dWheel = 0;     // avr distance in mm
double absOrAngle = 0; // absolute orientation angle
double prevAbsOrAngle = 0;
double deltaAngle = 0;
bool canUpdateLocation = false;
double maxTravelDistance = 1.0;
double tempAngle = 0;

// global string to read new coordinates
String X;
String Y;

// encoders
byte encoderAintLast = 0;
int wheelBTicks = 0;     // the number of the pulses
int wheelBTicksPrev = 0; // the number of the pulses 50 ms ago
double wheelBSpeed = 0;  // pulse/50ms
boolean directionB;      // the rotation direction

byte encoderBintLast = 0;
int wheelATicks = 0;     // the number of the pulses
int wheelATicksPrev = 0; // the number of the pulses 50 ms ago
double wheelASpeed = 0;  // pulse/50ms
boolean directionA;      // the rotation direction

// timers
unsigned long previousMillis = 0;
unsigned long previousMillisForward = 0;
unsigned long previousMillisLidar = 0;
unsigned long previousMillisNavigation = 0;

// pid
double motorPowerA = 0; // Power supplied to the motor PWM value.
double setpointA = 0;
double motorPowerB = 0; // Power supplied to the motor PWM value.
double setpointB = 0;

const double KpA = 3.5, KiA = 0.120, KdA = 0;
const double KpB = 2.8, KiB = 0.090, KdB = 0;

double afgelegdeWegTicks = 0; // distance covered in pulses
PID pidA(&wheelASpeed, &motorPowerA, &setpointA, KpA, KiA, KdA, DIRECT);
PID pidB(&wheelBSpeed, &motorPowerB, &setpointB, KpB, KiB, KdB, DIRECT);

    enum robotStates
    {
        IDLING,
        NAVIGATING,
        LOW_BATTERY,
        CHARGING,
        HALT,
        OVERHEAT,
        OBJECT_DETECTED,
    };

robotStates robotState = IDLING;

    enum instruction
    {
    FW_1,
    TURN_2,
    FW_3,
    TURN_4,
    FW_5,
    TURN_6,
    FW_7,
    STOP
    };

void changeState(robotStates newState)
{
    robotState = newState;
}

void resetSetPoints() // reset the setpoints
{
    setpointA = 0;
    setpointB = 0;
}

void resetParametersPID() // flush the parameters of the controller, so that the robot stops moving when we command it to
{
    pidA.SetOutputLimits(0.0, 1.0);  // Forces minimum up to 0.0
    pidA.SetOutputLimits(-1.0, 0.0); // Forces maximum down to 0.0
    pidA.SetOutputLimits(-MAX_PID_VALUE, MAX_PID_VALUE);
    pidB.SetOutputLimits(0.0, 1.0);  // Forces minimum up to 0.0
    pidB.SetOutputLimits(-1.0, 0.0); // Forces maximum down to 0.0
    pidB.SetOutputLimits(-MAX_PID_VALUE, MAX_PID_VALUE);
}

void forward(double distance, int instructionCode) // distance to cover in meter
{

    if (currentInstructionCode == instructionCode)
    {
        unsigned long currentMillis = millis();

        if (afgelegdeWegTicks < abs(distance) * 9180)
        {
            if (distance > 0)
            {
                setpointA = 60;
                setpointB = 60;
            }
            else
            {
                setpointA = -60;
                setpointB = -60;
            }

            if (currentMillis - previousMillisForward >= 50)
            { // per 50ms

                previousMillisForward = currentMillis;
                afgelegdeWegTicks += abs(wheelASpeed);
            }
        }
        else
        {
            afgelegdeWegTicks = 0;
            resetSetPoints();
            resetParametersPID();
            tempAngle = prevAbsOrAngle * RAD_TO_DEG;
            currentInstructionCode++;
        }
    }
}

void turn(double angle, int instructionCode)
{
    if (currentInstructionCode == instructionCode)
    {
        double targetError = angle - prevAbsOrAngle * RAD_TO_DEG;

        if (angle > 0)
        {
            if (targetError > 0)
            {
                setpointA = -40; // wiel links vooruit/wiel rechts achteruit (beide voor numTicks)
                setpointB = 40;
            }
            else
            {
                afgelegdeWegTicks = 0;
                resetSetPoints();
                resetParametersPID();
                tempAngle = prevAbsOrAngle * RAD_TO_DEG;
                currentInstructionCode++;
            }
        }
        else
        {
            if (targetError < 0)
            {
                setpointA = 40; // wiel rechts vooruit/wiel links achteruit (beide voor numTicks)
                setpointB = -40;
            }
            else
            {
                afgelegdeWegTicks = 0;
                resetSetPoints();
                resetParametersPID();
                tempAngle = prevAbsOrAngle * RAD_TO_DEG;
                currentInstructionCode++;
            }
        }
    }
}

/*
Calculate the speed of each wheel, each 50 ms. The wheelspeed is in ticks/50ms.
canUpdateLocation is set to true so the updateLocation function is in sync with the speed calculation..
*/

void calculateSpeed()
{
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= 50)
    {
        canUpdateLocation = true;
        previousMillis = currentMillis;
        wheelASpeed = (double)(wheelATicks - wheelATicksPrev);
        wheelBSpeed = (double)(wheelBTicks - wheelBTicksPrev);

        wheelATicksPrev = wheelATicks;
        wheelBTicksPrev = wheelBTicks;
    }
}

void wheelSpeedB()
{
    int encoderAintCurrent = digitalRead(encoderAint);
    if ((encoderAintLast == LOW) && encoderAintCurrent)
    {
        int encoderAanaCurrent = digitalRead(encoderAana);
        if (encoderAanaCurrent == LOW && directionB)
        {
            directionB = false; // Reverse
        }
        else if (encoderAanaCurrent == HIGH && !directionB)
        {
            directionB = true; // Forward
        }
    }
    encoderAintLast = encoderAintCurrent;

    if (!directionB)
    {
        wheelBTicks++;
    }
    else
    {
        wheelBTicks--;
    }
}

void wheelSpeedA()
{
    int encoderBintCurrent = digitalRead(encoderBint);
    if ((encoderBintLast == LOW) && encoderBintCurrent)
    {
        int encoderBanaCurrent = digitalRead(encoderBana);
        if (encoderBanaCurrent == LOW && directionA)
        {
            directionA = false; // Reverse
        }
        else if (encoderBanaCurrent == HIGH && !directionA)
        {
            directionA = true; // Forward
        }
    }
    encoderBintLast = encoderBintCurrent;

    if (directionA)
    {
        wheelATicks++;
    }
    else
    {
        wheelATicks--;
    }
}

void EncoderInit() // initialize the encoders
{
    directionB = true; // default -> Forward
    pinMode(encoderAana, INPUT);
    attachInterrupt(digitalPinToInterrupt(encoderAint), wheelSpeedB, CHANGE);

    directionA = true; // default -> Forward
    pinMode(encoderBana, INPUT);
    attachInterrupt(digitalPinToInterrupt(encoderBint), wheelSpeedA, CHANGE);
}

void constrainMotorPower() // constrain the motor so we dont exceed a certain value
{
    motorPowerA = constrain(motorPowerA, -MAX_PID_VALUE, MAX_PID_VALUE);
    motorPowerB = constrain(motorPowerB, -MAX_PID_VALUE, MAX_PID_VALUE);
}

void driveMotors()
{
    if (motorPowerA < 0)
    {
        digitalWrite(in1, HIGH); // Omkeren van de polariteit van de linker motor om de robot naar voor te sturen
        digitalWrite(in2, LOW);
    }
    else
    {
        digitalWrite(in1, LOW); // Omkeren van de polariteit van de linker motor om de robot naar achter te sturen
        digitalWrite(in2, HIGH);
    }
    analogWrite(enA, abs(motorPowerA));

    if (motorPowerB > 0)
    {
        digitalWrite(in3, HIGH); // Omkeren van de polariteit van de rechter motor om de robot naar voor te sturen
        digitalWrite(in4, LOW);
    }
    else
    {
        digitalWrite(in3, LOW); // Omkeren van de polariteit van de rechter motor om de robot naar achter te sturen
        digitalWrite(in4, HIGH);
    }
    analogWrite(enB, abs(motorPowerB));
}

void updateLocation()
{

    if (canUpdateLocation)
    {
        canUpdateLocation = false;
        Serial.print(coordX);
        Serial.print(" , ");
        Serial.print(coordY);
        Serial.print(" , ");
        Serial.println(prevAbsOrAngle * RAD_TO_DEG);

        double distancePerTick = 0.109; // mm, hoger dan experimenteel bepaalde waarde 
        double pivotDiam = 200;         // mm

        dWheel = (wheelASpeed * distancePerTick + wheelBSpeed * distancePerTick) / 2;
        deltaAngle = (wheelBSpeed * distancePerTick - wheelASpeed * distancePerTick) / (pivotDiam); // alles in mm; hoek in radialen
        // Serial.println(prevAbsOrAngle*RAD_TO_DEG);
        coordX += dWheel * cos((float)(prevAbsOrAngle + (deltaAngle / 2))) / 1000; // omzetting naar meter
        coordY += dWheel * sin((float)(prevAbsOrAngle + (deltaAngle / 2))) / 1000; // "          "      "
        prevAbsOrAngle += deltaAngle;                                              // hoek in rad
    }
}

// set the RGB to a color
void setColor(int redValue, int greenValue, int blueValue)
{
    analogWrite(redPin, redValue);
    analogWrite(greenPin, greenValue);
    analogWrite(bluePin, blueValue);
}

void calculateNavigation()
{
    distanceToTarget = sqrt((targetCoordX - coordX) * (targetCoordX - coordX) + (targetCoordY - coordY) * (targetCoordY - coordY));
    turnAngleToTarget = atan2(targetCoordY - coordY, targetCoordX - coordX) * RAD_TO_DEG;
    currentInstructionCode = 0; // reset the currentInstructionCode so the first instruction is turning
}

void temperature()
{
    sensors.requestTemperatures();
    if (!charging())
    {
        if (sensors.getTempCByIndex(0) > workingT || sensors.getTempCByIndex(1) > workingT || sensors.getTempCByIndex(2) > workingT)
        {
            changeState(OVERHEAT);
        }
        if (charging())
        {
            if (sensors.getTempCByIndex(0) > chargeT || sensors.getTempCByIndex(1) > chargeT || sensors.getTempCByIndex(2) > chargeT)
            {
                changeState(OVERHEAT);
            }
        }
    }
}

void startlidar()
{
    while (true)
    {
        lidar.waitPoint();
        analogWrite(RPLIDAR_MOTOR, 0);
        rplidar_response_device_info_t info;
        if (IS_OK(lidar.getDeviceInfo(info, 100)))
        {
            lidar.startScan();
            analogWrite(RPLIDAR_MOTOR, 255);
            return;
        }
    }
}

bool measure(int directionlook)
{
    lidar.waitPoint(500);
    RPLidarMeasurement current_point = lidar.getCurrentPoint();
    distancel = current_point.distance;
    anglel = current_point.angle;
    // Serial.println("distance: " + String(distancel));
    // Serial.println("angle   : " + String(anglel));
    if (distancel > 0 && distancel < minDistance && anglel < (directionlook + cone) && anglel > (directionlook - cone))
    {
        minDistance = distancel;
    }

    // Serial.println(minDistance);
    if (minDistance < savedistance * 10)
    {
        minDistance = 100000;
        return true;
    }
    return false;
}

void navigate()
{
    turn(turnAngleToTarget, 0);
    forward(distanceToTarget, 1);
}

void setup()
{
    pinMode(enA, OUTPUT);
    pinMode(enB, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
    pinMode(redPin, OUTPUT);
    pinMode(greenPin, OUTPUT);
    pinMode(bluePin, OUTPUT);

    pidA.SetMode(AUTOMATIC); // PID is set to automatic mode
    pidA.SetSampleTime(50);  // Set PID sampling frequency is 50ms
    pidA.SetOutputLimits(-MAX_PID_VALUE, MAX_PID_VALUE);
    pidB.SetMode(AUTOMATIC); // PID is set to automatic mode
    pidB.SetSampleTime(50);  // Set PID sampling frequency is 50ms
    pidB.SetOutputLimits(-MAX_PID_VALUE, MAX_PID_VALUE);
    Serial.begin(115200);

    Serial1.begin(115200); // For RPLidar
    lidar.begin(Serial1);
    pinMode(RPLIDAR_MOTOR, OUTPUT);
    startlidar();

    EncoderInit(); // Initialize the module
}

void loop()
{
    calculateSpeed();
    pidA.Compute();
    pidB.Compute();
    constrainMotorPower();
    updateLocation();
    driveMotors();

    //state machine
    if (robotState == IDLING)
    {
        setColor(0, 255, 0);
        Serial.println("Enter x-coord.");
        while (Serial.available() == 0)
            ;
        X = Serial.readStringUntil('\n');
        targetCoordX = X.toDouble();

        Serial.println("Enter y-coord.");
        while (Serial.available() == 0)
            ;
        Y = Serial.readStringUntil('\n');
        targetCoordY = Y.toDouble();

        calculateNavigation();
        changeState(NAVIGATING);

        /*Serial.print(distanceToTarget);
         Serial.print(" , ");
         Serial.println(turnAngleToTarget);*/
    }
    else if (robotState == NAVIGATING)
    {

        setColor(255, 0, 0);
        navigate();
        unsigned long currentMillis = millis();
        if (currentMillis - previousMillisNavigation >= 1000)
        { // recalculate the navigation every 1000ms
            previousMillisNavigation = currentMillis;
            afgelegdeWegTicks = 0;
            resetSetPoints();
            resetParametersPID();

            tempAngle = prevAbsOrAngle * RAD_TO_DEG;
            if (distanceToTarget > 0.05)
            {
                calculateNavigation();
            }
            else
            {
                changeState(HALT);
            }
        }
    }
    else if (robotState == HALT)
    {
        setColor(0, 0, 255);
        afgelegdeWegTicks = 0;
        resetSetPoints();
        resetParametersPID();
        tempAngle = prevAbsOrAngle * RAD_TO_DEG;
        changeState(IDLING);
        currentInstructionCode = 0;
    }
    else if (robotState == OBJECT_DETECTED)
    {
        setColor(0, 0, 255);
        afgelegdeWegTicks = 0;
        resetSetPoints();
        resetParametersPID();
        tempAngle = prevAbsOrAngle * RAD_TO_DEG;
        if (millis() - previousMillisLidar >= 1000)
        {
            changeState(NAVIGATING);
        }

        currentInstructionCode = 0;
    }
    else
    {
        setColor(255, 255, 255);
    }


    if (measure(270))
    {
        robotState = OBJECT_DETECTED;
        previousMillisLidar = millis();
    }

}