#include "WProgram.h"
#include <RobotParams.h>
#include <TimeInfo.h>
#include <OdometricLocalizer.h>
#include <SpeedController.h>
#include <BatteryMonitor.h>
#include <Servo.h> 
#include <Messenger.h>
#include <digitalWriteFast.h>  // library for high performance reads and writes by jrraines
                               // see http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1267553811/0
                               // and http://code.google.com/p/digitalwritefast/

// It turns out that the regular digitalRead() calls are too slow and bring the arduino down when
// used in the interrupt routines while the motor is running at full speed creating more than
// 40000 encoder ticks per second per motor.

#define c_UpdateInterval 20 // update interval in milli seconds
#define c_MaxMotorCV 30   // range is 0 ... 89 (half servo range)

// set robot params wheel diameter [m], trackwidth [m], ticks per revolution
RobotParams _RobotParams = RobotParams(0.0762, 0.37, 9750);
TimeInfo _TimeInfo = TimeInfo();
Servo _RightServo;  // create servo object to control right motor
Servo _LeftServo;  // create servo object to control left motor

// Quadrature encoders
// Left encoder
#define c_LeftEncoderInterrupt 4
#define c_LeftEncoderPinA 19
#define c_LeftEncoderPinB 25
#define LeftEncoderIsReversed
volatile bool _LeftEncoderASet;
volatile bool _LeftEncoderBSet;
volatile long _LeftEncoderTicks = 0;

// Right encoder
#define c_RightEncoderInterrupt 5
#define c_RightEncoderPinA 18
#define c_RightEncoderPinB 24
volatile bool _RightEncoderASet;
volatile bool _RightEncoderBSet;
volatile long _RightEncoderTicks = 0;

OdometricLocalizer _OdometricLocalizer(&_RobotParams, &_TimeInfo);

SpeedController _SpeedController(&_OdometricLocalizer, &_RobotParams, &_TimeInfo);
bool _SpeedControllerIsInitialized = false;

#define c_ScaledBatteryVInPin 8 // analog input pin for the battery voltage divider
#define c_VInToVBatteryRatio 2.921
BatteryMonitor _BatteryMonitor(c_ScaledBatteryVInPin, c_VInToVBatteryRatio);

// Instantiate Messenger object with the message function and the default separator (the space character)
Messenger _Messenger = Messenger();

bool _IsInitialized = false;

void setup()
{
  Serial.begin(115200);

  SetupEncoders();

  _Messenger.attach(OnMssageCompleted);

  _RightServo.attach(2);  // attaches the servo on specified pin to the servo object 
  _LeftServo.attach(3);  // attaches the servo on specified pin to the servo object
  _RightServo.write(90);
  _LeftServo.write(90);

  delay(100);
  _TimeInfo.Update();
}

void SetupEncoders()
{
  // Quadrature encoders
  // Left encoder
  pinMode(c_LeftEncoderPinA, INPUT);      // sets pin A as input
  digitalWrite(c_LeftEncoderPinA, LOW);  // turn on pullup resistors
  pinMode(c_LeftEncoderPinB, INPUT);      // sets pin B as input
  digitalWrite(c_LeftEncoderPinB, LOW);  // turn on pullup resistors
  _LeftEncoderASet = digitalReadFast(c_LeftEncoderPinA);   // read the input pin
  _LeftEncoderBSet = digitalReadFast(c_LeftEncoderPinB);   // read the input pin
  attachInterrupt(c_LeftEncoderInterrupt, HandleLeftMotorInterruptA, RISING);
  
  // Right encoder
  pinMode(c_RightEncoderPinA, INPUT);      // sets pin A as input
  digitalWrite(c_RightEncoderPinA, LOW);  // turn on pullup resistors
  pinMode(c_RightEncoderPinA, INPUT);      // sets pin A as input
  digitalWrite(26, LOW);  // turn on pullup resistors
  pinMode(26, INPUT);      // sets pin B as input
  digitalWrite(c_RightEncoderPinB, LOW);  // turn on pullup resistors
  _RightEncoderASet = digitalReadFast(c_RightEncoderPinA);   // read the input pin
  _RightEncoderBSet = digitalReadFast(c_RightEncoderPinB);   // read the input pin
  attachInterrupt(c_RightEncoderInterrupt, HandleRightMotorInterruptA, RISING); 
}

void loop()
{
  ReadSerial();

  unsigned long milliSecsSinceLastUpdate = millis() - _TimeInfo.LastUpdateMillisecs;
  if(milliSecsSinceLastUpdate >= c_UpdateInterval)
  {
    //Serial.println(milliSecsSinceLastUpdate);
    // time for another update
    _TimeInfo.Update();
    if (_IsInitialized)
    {
      DoWork();
    }
    else
    {
      RequestInitialization();
    }
  }
}

void DoWork()
{
  _OdometricLocalizer.Update(_LeftEncoderTicks, _RightEncoderTicks);
  _BatteryMonitor.Update();
  _SpeedController.Update(_BatteryMonitor.VoltageIsTooLow);
  IssueCommands();
    
  Serial.print("o\t"); // o indicates odometry message
  Serial.print(_OdometricLocalizer.X, 3);
  Serial.print("\t");
  Serial.print(_OdometricLocalizer.Y, 3);
  Serial.print("\t");
  Serial.print(_OdometricLocalizer.Heading, 3);
  Serial.print("\t");
  Serial.print(_OdometricLocalizer.V, 3);
  Serial.print("\t");
  Serial.print(_OdometricLocalizer.VLeft, 3);
  Serial.print("\t");
  Serial.print(_OdometricLocalizer.VRight, 3);
  Serial.print("\t");
  Serial.print(_OdometricLocalizer.Omega, 3);
  Serial.print("\t");
  Serial.print(_LeftEncoderTicks);
  Serial.print("\t");
  Serial.print(_RightEncoderTicks);
  Serial.print("\n");

  Serial.print("b\t"); // o indicates battery info message
  Serial.print(_BatteryMonitor.BatteryVoltage, 3);
  Serial.print("\t");
  Serial.print(_BatteryMonitor.VoltageIsTooLow);
  Serial.print("\n");
}

void RequestInitialization()
{
    _IsInitialized = true;
    
    if (!_SpeedControllerIsInitialized)
    {
      _IsInitialized = false;

      Serial.print("InitializeSpeedController"); // requesting initialization of the speed controller
      Serial.print("\n");
    }
    
    if (!_BatteryMonitor.IsInitialized)
    {
      _IsInitialized = false;

      Serial.print("InitializeBatteryMonitor"); // requesting initialization of the battery monitor
      Serial.print("\n");
    }
}

void IssueCommands()
{
  float normalizedRightMotorCV, normalizedLeftMotorCV;
  
  normalizedRightMotorCV = _SpeedController.NormalizedLeftCV;
  normalizedLeftMotorCV = _SpeedController.NormalizedRightCV;
  
  /*
  Serial.print("Speed: ");
  Serial.print(_SpeedController.DesiredVelocity);
  Serial.print("\t");
  Serial.print(_SpeedController.DesiredAngularVelocity);
  Serial.print("\t");
  Serial.print(_SpeedController.LeftError);
  Serial.print("\t");
  Serial.print(_SpeedController.RightError);
  Serial.print("\t");
  Serial.print(normalizedRightMotorCV);
  Serial.print("\t");
  Serial.print(normalizedLeftMotorCV);
  Serial.print("\n");
  */
  
  float rightServoValue = mapFloat(normalizedRightMotorCV, -1, 1, 90.0 - c_MaxMotorCV, 90.0 + c_MaxMotorCV);     // scale it to use it with the servo (value between 0 and 180) 
  float leftServoValue = mapFloat(normalizedLeftMotorCV, -1, 1, 90.0 - c_MaxMotorCV, 90.0 + c_MaxMotorCV);     // scale it to use it with the servo (value between 0 and 180) 
 
 
  /*
  Serial.print("Servos: ");
  Serial.print(rightServoValue);
  Serial.print("\t");
  Serial.print(leftServoValue);
  Serial.print("\n");
  */

  _RightServo.write(rightServoValue);     // sets the servo position according to the scaled value (0 ... 179)
  _LeftServo.write(leftServoValue);     // sets the servo position according to the scaled value (0 ... 179)
}


// Interrupt service routines for the left motor's quadrature encoder
void HandleLeftMotorInterruptA()
{
  // Test transition; since the interrupt will only fire on 'rising' we don't need to read pin A
  _LeftEncoderBSet = digitalReadFast(c_LeftEncoderPinB);   // read the input pin

  // and adjust counter + if A leads B
  #ifdef LeftEncoderIsReversed
    _LeftEncoderTicks -= _LeftEncoderBSet ? -1 : +1;
  #else
    _LeftEncoderTicks += _LeftEncoderBSet ? -1 : +1;
  #endif
}

// Interrupt service routines for the right motor's quadrature encoder
void HandleRightMotorInterruptA()
{
  // Test transition; since the interrupt will only fire on 'rising' we don't need to read pin A
  _RightEncoderBSet = digitalReadFast(c_RightEncoderPinB);   // read the input pin

  // and adjust counter + if A leads B
  #ifdef RightEncoderIsReversed
    _RightEncoderTicks -= _RightEncoderBSet ? -1 : +1;
  #else
    _RightEncoderTicks += _RightEncoderBSet ? -1 : +1;
  #endif
}


void ReadSerial()
{
  while (Serial.available())
  {
    _Messenger.process(Serial.read());
  }
}

// Define messenger function
void OnMssageCompleted()
{
  if (_Messenger.checkString("s"))
  {
    SetSpeed();
    return;
  }

  if (_Messenger.checkString("SpeedControllerGains"))
  {
    SetSpeedControllerGains();
    return;
  }
  
  if (_Messenger.checkString("BatteryMonitorParams"))
  {
    InitializeBatteryMonitor();
  }

  // clear out unrecognized content
  while(_Messenger.available())
  {
    _Messenger.readInt();
  }
}

void SetSpeed()
{
  _SpeedController.DesiredVelocity = GetFloatFromBaseAndExponent(_Messenger.readInt(), _Messenger.readInt());
  _SpeedController.DesiredAngularVelocity = GetFloatFromBaseAndExponent(_Messenger.readInt(), _Messenger.readInt());
}

void SetSpeedControllerGains()
{
  _SpeedController.PParam = GetFloatFromBaseAndExponent(_Messenger.readInt(), _Messenger.readInt());
  _SpeedController.IParam = GetFloatFromBaseAndExponent(_Messenger.readInt(), _Messenger.readInt());
  _SpeedController.DParam = GetFloatFromBaseAndExponent(_Messenger.readInt(), _Messenger.readInt());
  
  _SpeedControllerIsInitialized = true;

  Serial.print("PID Params: ");
  Serial.print(_SpeedController.PParam);
  Serial.print("\t");
  Serial.print(_SpeedController.IParam);
  Serial.print("\t");
  Serial.print(_SpeedController.DParam);
  Serial.print("\n");
}

void InitializeBatteryMonitor()
{
  float voltageTooLowlimit = GetFloatFromBaseAndExponent(_Messenger.readInt(), _Messenger.readInt());
  _BatteryMonitor.InitializeLowVoltageLimit(voltageTooLowlimit);

  Serial.print("battery monitor Params: ");
  Serial.print(voltageTooLowlimit);
  Serial.print("\n");
}

float GetFloatFromBaseAndExponent(int base, int exponent)
{
  return base * pow(10, exponent);
}

long mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

