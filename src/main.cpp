#include <Arduino.h>
#include <PID_v1.h>

#define MAX_PID_VALUE 250
/*Encoder pins*/
#define encoder0pinA 10
#define encoder0pinB A0
#define encoder1pinA 11 
#define encoder1pinB A1 
/*Motor pins*/
#define enA 7
#define in1 9
#define in2 8

#define enB 6
#define in3 4
#define in4 5


byte encoder0PinALast = 0;
int wheelBTicks = 0; // the number of the pulses
int wheelBTicksPrev = 0;
double wheelBSpeed = 0;
boolean directionB; // the rotation direction


byte encoder1PinALast = 0;
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
instruction instructionState = FW_1;

enum robotStates{
  IDLE,
  NAVIGATING,
  HALT,
};

void incrementState(){
      instructionState = static_cast<instruction>(instructionState + 1);
}

void resetSetPoints(){
    setpointA = 0;
    setpointB = 0;
}

void resetParametersPID(){
pidA.SetOutputLimits(0.0, 1.0);  // Forces minimum up to 0.0
pidA.SetOutputLimits(-1.0, 0.0);  // Forces maximum down to 0.0
pidA.SetOutputLimits(-MAX_PID_VALUE, MAX_PID_VALUE);
pidB.SetOutputLimits(0.0, 1.0);  // Forces minimum up to 0.0
pidB.SetOutputLimits(-1.0, 0.0);  // Forces maximum down to 0.0
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
    incrementState();
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
    incrementState();
  }
}

void calculateSpeed()
{
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= 50)
  { // per 10ms
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
  int encoder0pinACurrent = digitalRead(encoder0pinA);
  if ((encoder0PinALast == LOW) && encoder0pinACurrent)
  {
    int encoder0pinBCurrent = digitalRead(encoder0pinB);
    if (encoder0pinBCurrent == LOW && directionB)
    {
      directionB = false; // Reverse
    }
    else if (encoder0pinBCurrent == HIGH && !directionB)
    {
      directionB = true; // Forward
    }
  }
  encoder0PinALast = encoder0pinACurrent;

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
  int encoder1pinACurrent = digitalRead(encoder1pinA);
  if ((encoder1PinALast == LOW) && encoder1pinACurrent)
  {
    int encoder1pinBCurrent = digitalRead(encoder1pinB);
    if (encoder1pinBCurrent == LOW && directionA)
    {
      directionA = false; // Reverse
    }
    else if (encoder1pinBCurrent == HIGH && !directionA)
    {
      directionA = true; // Forward
    }
  }
  encoder1PinALast = encoder1pinACurrent;

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
  pinMode(encoder0pinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder0pinA), wheelSpeedB, CHANGE);

  directionA = true; // default -> Forward
  pinMode(encoder1pinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder1pinA), wheelSpeedA, CHANGE);
}

void setup()
{
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pidA.SetMode(AUTOMATIC); // PID is set to automatic mode
  pidA.SetSampleTime(50);  // Set PID sampling frequency is 50ms
  pidA.SetOutputLimits(-MAX_PID_VALUE, MAX_PID_VALUE);
  pidB.SetMode(AUTOMATIC); // PID is set to automatic mode
  pidB.SetSampleTime(50);  // Set PID sampling frequency is 50ms
  pidB.SetOutputLimits(-MAX_PID_VALUE, MAX_PID_VALUE);
  Serial.begin(9600);
  EncoderInit(); // Initialize the module
}



void loop()
{
  calculateSpeed();
  motorPowerA = constrain(motorPowerA, -MAX_PID_VALUE, MAX_PID_VALUE);
  motorPowerB = constrain(motorPowerB, -MAX_PID_VALUE, MAX_PID_VALUE);

  pidA.Compute();
  pidB.Compute();

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
  
  Serial.print(motorPowerA);
  Serial.print(" , ");
  Serial.println(motorPowerB);


  switch (instructionState)
  {
  case FW_1:
    forward(1);
    break;
  case TURN_2:
    turn(90);
    break;
  case FW_3:
    forward(1);
    break;
  case TURN_4:
    turn(90);
    break;
  case FW_5:
    forward(1);
    break;
  case TURN_6:
    turn(90);
    break;    
  case FW_7:
    forward(1);
    break;
  case STOP:
    break;
  default:
    instructionState = STOP;
    break;
  }
}