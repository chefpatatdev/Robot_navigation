#include <Arduino.h>
#include <PID_v1.h>
#include <Wire.h>
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

#define redPin 5
#define greenPin 7
#define bluePin  6

/*temperatuur sensor*/
#define ONE_WIRE_BUS 4
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

RPLidar lidar;
#define cone 15
#define savedistance 50
float minDistance = 100000;
float angleAtMinDist = 0;
int objectCounter = 0;

#define MAX_PID_VALUE 250

int currentInstructionCode = 0;

const int chargeT = 45;
const int workingT = 60;

//positionering
double coordX = 0.0;  //huidige positie
double coordY = 0.0;
double targetCoordX = 1; //target positie
double targetCoordY = 1;
double distanceToTarget = 0;
double turnAngleToTarget = 0;

String X;
String Y;

double dWheel = 0;// avr distance in mm
double absOrAngle = 0; // absolute orientatie hoek
double prevAbsOrAngle = 0;
double deltaAngle = 0;
bool canupdatelocation = false;
double maxTravelDistance = 1.0;
double tempAngle =0;


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
unsigned long previousMillisLidar = 0;



double motorPowerA = 0; // Power supplied to the motor PWM value.
double setpointA = 0;
double motorPowerB = 0; // Power supplied to the motor PWM value.
double setpointB = 0;
//const double KpA = 3.0, KiA = 0.142, KdA = 0;
//const double KpB = 2.5, KiB = 0.083, KdB = 0;
const double KpA = 3.5, KiA = 0.120, KdA = 0;
const double KpB = 2.8, KiB = 0.090, KdB = 0;

double afgelegdeWegTicks = 0;
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
};
robotStates robotState = IDLING;

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

void forward(double distance,int instructionCode)
{ // distance in meter

if(currentInstructionCode == instructionCode){
  //Serial.println("forward");
  unsigned long currentMillis = millis();
  /*          if (currentMillis - previousMillisLidar >= 1000)
        {
                      previousMillisForward = currentMillis;
    if(directionA == 1 && directionB == 0){
        Serial.println("forward");

      if(measure(270)==1){
        if(objectCounter > 10){
          robotState = HALT;
          objectCounter = 0;
        }
      objectCounter++;
    }
    }
        if(directionA == 0 && directionB == 1){
          Serial.println("back");
          if(measure(90)==1){
      if(objectCounter > 10){
        robotState = HALT;
        objectCounter = 0;
      }
      objectCounter++;
    }
    }
        }*/
    
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
            // Serial.println(afgelegdeWegTicks);
        }
    }
    else
    {
        afgelegdeWegTicks = 0;
        resetSetPoints();
        resetParametersPID();
        objectCounter = 0;
        tempAngle = prevAbsOrAngle*RAD_TO_DEG;
        currentInstructionCode++;
    }
}
}

void turn(double angle,int instructionCode)
{
    
    if(currentInstructionCode == instructionCode){
    //Serial.println("turn");
    double targetError = angle -prevAbsOrAngle*RAD_TO_DEG;
        //Serial.print(targetError);
        //Serial.print(" , ");
        //Serial.println(prevAbsOrAngle*RAD_TO_DEG);

        if (angle > 0)
        {
            if (targetError>0)
            {
              setpointA = -40; // wiel links vooruit/wiel rechts achteruit (beide voor numTicks)
              setpointB = 40;
            }else{
              afgelegdeWegTicks = 0;
              resetSetPoints();
              resetParametersPID();
              tempAngle = prevAbsOrAngle*RAD_TO_DEG;
              currentInstructionCode++;
            }
        }
        else
        {
            if (targetError<0)
            {
            setpointA = 40; // wiel rechts vooruit/wiel links achteruit (beide voor numTicks)
            setpointB = -40;
            }else{
              afgelegdeWegTicks = 0;
              resetSetPoints();
              resetParametersPID();
              tempAngle = prevAbsOrAngle*RAD_TO_DEG;
              currentInstructionCode++;
            }
        }
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
        Serial.println(coordY/1000);

        double distancePerTick = 0.109; //mm
        double pivotDiam = 200; //mm

        dWheel = (wheelASpeed * distancePerTick + wheelBSpeed * distancePerTick) / 2;
        deltaAngle = (wheelBSpeed * distancePerTick - wheelASpeed  * distancePerTick) / (pivotDiam); // alles in mm; hoek in radialen
        //Serial.println(prevAbsOrAngle*RAD_TO_DEG);
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

bool charging() {
    return false;
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
    /*if (IS_OK(lidar.waitPoint())) {

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
        rplidar_response_device_info_t info;
        if (IS_OK(lidar.getDeviceInfo(info, 100))) {
            lidar.startScan();
            Serial.println("LIDAR START");
        }
    }*/
    if (IS_OK(lidar.waitPoint())){
          RPLidarMeasurement current_point = lidar.getCurrentPoint();
        float distance = current_point.distance;
        float angle = current_point.angle;
        if (lidar.getCurrentPoint().startBit) {
            minDistance = 100000;
        }
        else if (distance > 0 && distance < minDistance && angle < (directionlook + cone) && angle >(directionlook - cone)) {
            minDistance = distance;
        }
        Serial.println(minDistance);
    }

    return minDistance < savedistance * 10;
}

void navigate(){
  //Serial.println(currentInstructionCode);
  turn(turnAngleToTarget,0);
  forward(distanceToTarget,1);
  if(currentInstructionCode == 2){
    changeState(HALT);
  }
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

    Serial1.begin(115200);  // For RPLidar
    lidar.begin(Serial1);
    pinMode(RPLIDAR_MOTOR, OUTPUT);
    
    EncoderInit(); // Initialize the module
    //analogWrite(RPLIDAR_MOTOR, 255);
    //delay(2000);
}

void loop(){
    calculateSpeed();
    constrainMotorPower();
    //Serial.println(robotState);
    updateLocation();
    pidA.Compute();
    pidB.Compute();
    switch (robotState)
    {
    case IDLING:

        setColor(255,0,255);
        Serial.println("Enter x-coord.");
        while(Serial.available()==0);
        X = Serial.readStringUntil('\n');
        targetCoordX = X.toDouble();


        Serial.println("Enter y-coord.");
        while(Serial.available()==0);
        Y = Serial.readStringUntil('\n');
        targetCoordY = Y.toDouble();
        //Serial.println(targetCoordX);
      //Serial.println(targetCoordY);


      changeState(NAVIGATING);
      distanceToTarget = sqrt((targetCoordX-coordX)*(targetCoordX-coordX)+(targetCoordY-coordY)*(targetCoordY-coordY));
      turnAngleToTarget = atan2(targetCoordY - coordY, targetCoordX - coordX)*RAD_TO_DEG;
      Serial.println(distanceToTarget);
      Serial.println(turnAngleToTarget);
        break;
    case NAVIGATING:
        Serial.println("jghjhg");
        Serial.println(robotState);
        setColor(255,0,0);
        navigate();
        break;
    case LOW_BATTERY:
        //setColor(255, 0, 0); // Red Color
        break;
    case CHARGING:
        setColor(255, 165, 0); // Orange color
        break;
    case HALT:
        setColor(255, 0, 0);
        afgelegdeWegTicks = 0;
        resetSetPoints();
        resetParametersPID();
        objectCounter = 0;
        tempAngle = prevAbsOrAngle*RAD_TO_DEG;
        changeState(IDLING);
        currentInstructionCode =0; //2DE KEER LEZEN GAAT NIET EN NAAR VERKEERDE COORDINATEN
        break;
    case OVERHEAT:
        //blinkLed(500)
        break;
    }
 
    driveMotors();

}