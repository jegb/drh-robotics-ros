#include "WProgram.h"
#include <RobotParams.h>
#include <TimeInfo.h>
#include <QuadratureEncoder.h>
#include <OdometricLocalizer.h>
#include <SpeedController.h>
#include <Servo.h> 
#include "Psx_analog.h"      // Includes the Psx Library to access a Sony Playstation controller
#include <Messenger.h>

#define c_MaxSpeed 30.0   // range is 0 ... 90 (half servo range)

// Sony Playstation 2 Controller
#define c_PsxDataPin 36
#define c_PsxCommandPin 35
#define c_PsxAttPin 33
#define c_PsxClockPin 34

Psx _Psx;
// set robot params wheel diameter, trackwidth, counts per revolution
RobotParams _RobotParams = RobotParams(0.075, 0.37, 19500);
TimeInfo _TimeInfo = TimeInfo();
Servo _RightServo;  // create servo object to control right motor
Servo _LeftServo;  // create servo object to control left motor

// The quadrature encoder for the left motor uses external interupt 5 on pin 18 and the regular input at pin 24
QuadratureEncoder _LeftEncoder(18, 24, 26, false);

// The quadrature encoder for the right motor uses external interupt 4 on pin 19 and the regular input at pin 25
QuadratureEncoder _RightEncoder(19, 25, true);

OdometricLocalizer _OdometricLocalizer(&_LeftEncoder, &_RightEncoder, &_RobotParams, &_TimeInfo);
SpeedController _SpeedController(0, 0, 0, &_LeftEncoder, &_RightEncoder, &_RobotParams, &_TimeInfo);

// Instantiate Messenger object with the message function and the default separator (the space character)
Messenger _Messenger = Messenger(); 

void setup()
{
  Serial.begin(115200);
  _Psx.setupPins(c_PsxDataPin, c_PsxCommandPin, c_PsxAttPin, c_PsxClockPin);  // Defines what each pin is used (Data Pin #, Cmnd Pin #, Att Pin #, Clk Pin #)
  _Psx.initcontroller(psxAnalog);
  
  _RightServo.attach(2);  // attaches the servo on specified pin to the servo object 
  _LeftServo.attach(3);  // attaches the servo on specified pin to the servo object 

  attachInterrupt(5, HandleLeftMotorInterruptA, CHANGE); // Pin 18 
  attachInterrupt(4, HandleRightMotorInterruptA, CHANGE); // Pin 19

  _Messenger.attach(OnMssageCompleted);

  delay(100);
}

void loop()
{
  ReadSerial();
  _TimeInfo.Update();
  _OdometricLocalizer.Update();
  _SpeedController.Update(0, 0);
  Serial.print(_OdometricLocalizer.X, 3);
  Serial.print("\t");
  Serial.print(_OdometricLocalizer.Y, 3);
  Serial.print("\t");
  Serial.print(_OdometricLocalizer.Heading, 3);
  Serial.print("\t");
  Serial.print(_OdometricLocalizer.V, 3);
  Serial.print("\t");
  Serial.print(_OdometricLocalizer.Omega, 3);
  Serial.print("\t");
  Serial.print(_LeftEncoder.GetPosition());
  Serial.print("\t");
  Serial.print(_RightEncoder.GetPosition());
  Serial.print("\n");

  IssueCommands();
  delay(10);   // waits for the servo to get there 
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
  
  // clear out unrecognized content
  while(_Messenger.available())
  {
    _Messenger.readInt();
  }
}

void IssueCommands()
{
  float rigthSpeed, leftSpeed;
  
  _Psx.poll(); // poll the Sony Playstation controller
  if (_Psx.Controller_mode == 115)
  {
    // analog mode; we use the right joystick to determine the desired speed

    float mainSpeed = -(_Psx.Right_y - 128.0);    
    float rightLeftRatio = -(_Psx.Right_x - 128) / 128.0;
    
    rigthSpeed = mainSpeed + rightLeftRatio * 128;
    leftSpeed = mainSpeed - rightLeftRatio * 128;
  }
  else
  {
    rigthSpeed = 0;
    leftSpeed = 0;
  }
  
  /*
  Serial.print("Speed: ");
  Serial.print(rigthSpeed);
  Serial.print("\t");
  Serial.print(leftSpeed);
  Serial.print("\n");
  */
  
  float rightServoValue = map(rigthSpeed, -128.0, 127.0, 90.0 - c_MaxSpeed, 90.0 + c_MaxSpeed);     // scale it to use it with the servo (value between 0 and 180) 
  float leftServoValue = map(leftSpeed, -128.0, 127.0, 90.0 - c_MaxSpeed, 90.0 + c_MaxSpeed);     // scale it to use it with the servo (value between 0 and 180) 
 
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
  _LeftEncoder.OnAChanged();
}

// Interrupt service routines for the right motor's quadrature encoder
void HandleRightMotorInterruptA()
{
  _RightEncoder.OnAChanged();
}

