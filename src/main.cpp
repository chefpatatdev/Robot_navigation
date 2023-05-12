#include <Arduino.h>
#include <PID_v1.h>
#include <Wire.h>
#include "INA219.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <RPLidar.h>

#define RPLIDAR_MOTOR 3
/*Encoder pins*/
#define encoderAint 21
#define encoderAana 48
#define encoderBint 20
#define encoderBana 49
/*Motor pins*/
#define enA 13
#define in1 11
#define in2 12

#define enB 8
#define in3 10
#define in4 9

#define redPin 1
#define greenPin 2
#define bluePin  3

/*temperatuur sensor*/
#define ONE_WIRE_BUS 4
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

RPLidar lidar;
#define cone 15
#define savedistance 50
float minDistance = 100000;
float angleAtMinDist = 0;

//I2C poorten van INA219
INA219 ina1(0x40);
INA219 ina2(0x41);
INA219 ina3(0x42);
INA219 ina4(0x43);

#define MAX_PID_VALUE 250

const int minI = 2960;
const int minV = 3.5;
const int chargeT = 45;
const int workingT = 60;


//positionering
double coordX = 0.0;  //huidige positie
double coordY = 0.0;
double dWheel = 0;// avr distance in mm
double absOrAngle = 0; // absolute orientatie hoek
double prevAbsOrAngle = 0;
double deltaAngle = 0;
bool canupdatelocation = false;
double maxTravelDistance = 1.0;



byte encoderAintLast = 0;
int wheelBTicks = 0; // the number of the pulses
int wheelBTicksPrev = 0;
double wheelBSpeed = 0;
boolean directionB; // the rotation direction

byte encoderBintLast = 0;
int wheelATicks = 0; // the number of the pulses
int wheelATicksPrev = 0;
double wheelASpeed = 0;
boolean directionA; // the rotation direction
unsigned long previousMillis = 0;
unsigned long previousMillisForward = 0;



double motorPowerA = 0; // Power supplied to the motor PWM value.
double setpointA = 0;
double motorPowerB = 0; // Power supplied to the motor PWM value.
double setpointB = 0;
const double KpA = 3.5, KiA = 0.117, KdA = 0;
const double KpB = 3.5, KiB = 0.105, KdB = 0;

double afgelegdeWegTicks = 0;
PID pidA(&wheelASpeed, &motorPowerA, &setpointA, KpA, KiA, KdA, DIRECT);
PID pidB(&wheelBSpeed, &motorPowerB, &setpointB, KpB, KiB, KdB, DIRECT);

enum robotStates
{
    IDLE,
    NAVIGATING,
    LOW_BATTERY,
    CHARGING,
    HALT,
    OVERHEAT,
};
robotStates robotState = IDLE;

void changeState(robotStates newState)
{
    robotState = newState;
}

void resetSetPoints()
{
    setpointA = 0;
    setpointB = 0;
}

void resetParametersPID()
{
    pidA.SetOutputLimits(0.0, 1.0);  // Forces minimum up to 0.0
    pidA.SetOutputLimits(-1.0, 0.0); // Forces maximum down to 0.0
    pidA.SetOutputLimits(-MAX_PID_VALUE, MAX_PID_VALUE);
    pidB.SetOutputLimits(0.0, 1.0);  // Forces minimum up to 0.0
    pidB.SetOutputLimits(-1.0, 0.0); // Forces maximum down to 0.0
    pidB.SetOutputLimits(-MAX_PID_VALUE, MAX_PID_VALUE);
}

void forward(double distance)
{ // distance in meter
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
        { // per 10ms
            previousMillisForward = currentMillis;
            afgelegdeWegTicks += abs(wheelASpeed);
            // Serial.println(afgelegdeWegTicks);
        }
    }
    else
    {
        afgelegdeWegTicks = 0;
        resetSetPoints();
        resetParametersPID();
    }
}

void turn(double angle)
{
    double ticksPerOmw = 1941;
    double wheelDiam = 65; // mm
    double wheelCirc = wheelDiam * PI;
    double pivotDiam = 200; // mm
    double pivotCirc = pivotDiam * PI;
    double arcLength = abs(angle) / 360 * pivotCirc; // mm
    double numRev = arcLength / wheelCirc;
    double numTicks = numRev * ticksPerOmw;

    if (afgelegdeWegTicks < numTicks)
    {
        if (angle > 0)
        {
            setpointA = -40; // wiel links vooruit/wiel rechts achteruit (beide voor numTicks)
            setpointB = 40;
        }
        else
        {
            setpointA = 40; // wiel rechts vooruit/wiel links achteruit (beide voor numTicks)
            setpointB = -40;
        }
        unsigned long currentMillis = millis();
        if (currentMillis - previousMillisForward >= 50)
        {
            previousMillisForward = currentMillis;
            afgelegdeWegTicks += abs(wheelASpeed);
        }
    }
    else
    {
        afgelegdeWegTicks = 0;
        resetSetPoints();
        resetParametersPID();
    }
}

void calculateSpeed()
{
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= 50)
    {
        canupdatelocation = true;
        previousMillis = currentMillis;
        wheelASpeed = (double)(wheelATicks - wheelATicksPrev);
        wheelBSpeed = (double)(wheelBTicks - wheelBTicksPrev);

        wheelATicksPrev = wheelATicks;
        wheelBTicksPrev = wheelBTicks;
        /*Serial.print(wheelASpeed);
          Serial.print(" , ");
          Serial.println(wheelBSpeed);*/
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

void EncoderInit()
{
    directionB = true; // default -> Forward
    pinMode(encoderAana, INPUT);
    attachInterrupt(digitalPinToInterrupt(encoderAint), wheelSpeedB, CHANGE);

    directionA = true; // default -> Forward
    pinMode(encoderBana, INPUT);
    attachInterrupt(digitalPinToInterrupt(encoderBint), wheelSpeedA, CHANGE);
}

void constrainMotorPower()
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

void updateLocation() {

    if (canupdatelocation)
    {
        canupdatelocation = false;
        Serial.print(coordX/1000);
        Serial.print(" , ");
        Serial.print(coordY/1000);
        Serial.print(" , ");

        double distancePerTick = 0.109; //mm
        double pivotDiam = 200; //mm

        dWheel = (wheelASpeed * distancePerTick + wheelBSpeed * distancePerTick) / 2;
        deltaAngle = (wheelBSpeed * distancePerTick - wheelASpeed  * distancePerTick) / (pivotDiam); // alles in mm; hoek in radialen
                Serial.print(deltaAngle*RAD_TO_DEG);
        Serial.print(" , ");
        Serial.println(prevAbsOrAngle*RAD_TO_DEG);
        coordX += dWheel * cos((float)(prevAbsOrAngle + (deltaAngle / 2))); // omzetting naar meter
        coordY += dWheel * sin((float)(prevAbsOrAngle + (deltaAngle / 2))); // "          "      "
        prevAbsOrAngle += deltaAngle; //hoek in rad
    }
}


void setColor(int redValue, int greenValue, int blueValue) {
    analogWrite(redPin, redValue);
    analogWrite(greenPin, greenValue);
    analogWrite(bluePin, blueValue);
}

void statusBattery() {
    if (charging()) {
        changeState(CHARGING);
        setColor(255, 165, 0); // Orange color
    }
    else if (avrCurrent() < minI || !avrVoltage()) {
        changeState(LOW_BATTERY);
        setColor(255, 0, 0); // Red Color
    }
    else {
        setColor(0, 255, 0); // Green Color
    }
}


float avrCurrent() {
    return (ina1.getCurrent_mA() + ina2.getCurrent_mA() + ina3.getCurrent_mA()) / 3;
}
bool charging() {
    return false;
}

bool avrVoltage() {
    float loadvoltage1 = ina1.getBusVoltage_V() + (ina1.getShuntVoltage_mV() / 1000);
    float loadvoltage2 = ina2.getBusVoltage_V() + (ina2.getShuntVoltage_mV() / 1000);
    float loadvoltage3 = ina3.getBusVoltage_V() + (ina3.getShuntVoltage_mV() / 1000);

    return !(loadvoltage1 < minV || loadvoltage2 < minV || loadvoltage3 < minV);

}

void temperature() {
    sensors.requestTemperatures();
    if (!charging()) {
        if (sensors.getTempCByIndex(0) > workingT || sensors.getTempCByIndex(1) > workingT || sensors.getTempCByIndex(2) > workingT) {
            changeState(OVERHEAT);
        }
        if (charging()) {
            if (sensors.getTempCByIndex(0) > chargeT || sensors.getTempCByIndex(1) > chargeT || sensors.getTempCByIndex(2) > chargeT) {
                changeState(OVERHEAT);
            }

        }
    }
}
bool measure(int directionlook) {
    if (IS_OK(lidar.waitPoint())) {

        RPLidarMeasurement current_point = lidar.getCurrentPoint();
        float distance = current_point.distance;
        float angle = current_point.angle;
        if (lidar.getCurrentPoint().startBit) {
            minDistance = 100000;
        }
        else if (distance > 0 && distance < minDistance && angle < (directionlook + cone) && angle >(directionlook - cone)) {
            minDistance = distance;
        }
    }
    else {
        analogWrite(RPLIDAR_MOTOR, 0);
        rplidar_response_device_info_t info;
        if (IS_OK(lidar.getDeviceInfo(info, 100))) {
            lidar.startScan();
            analogWrite(RPLIDAR_MOTOR, 255);
            delay(1000);
        }
    }
    return minDistance < savedistance * 10;
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

    //Serial1.begin(115200);  // For RPLidar
    //lidar.begin(Serial1);
    //pinMode(RPLIDAR_MOTOR, OUTPUT);

    EncoderInit(); // Initialize the module
}

void loop()
{
    calculateSpeed();
    constrainMotorPower();
    //temperature();

    pidA.Compute();
    pidB.Compute();
    switch (robotState)
    {
    case IDLE:
        updateLocation();
        forward(1);
        break;
    case NAVIGATING:
        break;
    case LOW_BATTERY:
        //setColor(255, 0, 0); // Red Color
        break;
    case CHARGING:
        setColor(255, 165, 0); // Orange color
        //statusBattery()//avrCurrent(), charging());
        break;
    case HALT:
        break;
    case OVERHEAT:
        //blinkLed(500)
        break;
    }

    driveMotors();

}