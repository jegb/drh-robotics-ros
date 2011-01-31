#include "WProgram.h"
#include <Servo.h> 
#include <digitalWriteFast.h>  // library for high performance reads and writes by jrraines
                               // see http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1267553811/0
                               // and http://code.google.com/p/digitalwritefast/

// It turns out that regular digitalRead() calls are too slow and bring the arduino down when
// I use them in the interrupt routines when the motor runs at full speed which results in 
// more than 40000 ticks per second per motor. 



// Quadrature encoders
// Left encoder
#define c_LeftEncoderInterrupt 4
#define c_LeftEncoderPinA 19
#define c_LeftEncoderPinB 25
#define c_LeftEncoderIsReversed true
volatile bool _LeftEncoderASet;
volatile bool _LeftEncoderBSet;
volatile long _LeftEncoderTicks = 0;

// Right encoder
#define c_RightEncoderInterrupt 5
#define c_RightEncoderPinA 18
#define c_RightEncoderPinB 24
#define c_RightEncoderIsReversed false
volatile bool _RightEncoderASet;
volatile bool _RightEncoderBSet;
volatile long _RightEncoderTicks = 0;

Servo _RightServo;  // create servo object to control right motor
Servo _LeftServo;  // create servo object to control left motor

int potpin = 0;  // analog pin used to connect the potentiometer
int val;    // variable to read the value from the analog pin


void setup()
{
  Serial.begin(115200);
  
  _RightServo.attach(2);  // attaches the servo on specified pin to the servo object 
  _LeftServo.attach(3);  // attaches the servo on specified pin to the servo object
  
  _RightServo.write(179);
  _LeftServo.write(179);
  
  
  
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
  val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023) 
  val = map(val, 0, 1023, 0, 179);     // scale it to use it with the servo (value between 0 and 180) 

  _RightServo.write(val);
  _LeftServo.write(val);

  Serial.print(_LeftEncoderTicks);
  Serial.print("\t");
  Serial.print(_RightEncoderTicks);
  Serial.print("\n");

  delay(20);
}


// Interrupt service routines for the left motor's quadrature encoder
void HandleLeftMotorInterruptA()
{
  // Test transition
  _LeftEncoderASet = digitalReadFast(c_LeftEncoderPinA);   // read the input pin
  _LeftEncoderBSet = digitalReadFast(c_LeftEncoderPinB);   // read the input pin

  // and adjust counter + if A leads B
  if (c_LeftEncoderIsReversed)
    _LeftEncoderTicks -= (_LeftEncoderASet != _LeftEncoderBSet) ? +1 : -1;
  else
    _LeftEncoderTicks += (_LeftEncoderASet != _LeftEncoderBSet) ? +1 : -1;
}

// Interrupt service routines for the right motor's quadrature encoder
void HandleRightMotorInterruptA()
{
  // Test transition
  _RightEncoderASet = digitalReadFast(c_RightEncoderPinA);   // read the input pin
  _RightEncoderBSet = digitalReadFast(c_RightEncoderPinB);   // read the input pin

  // and adjust counter + if A leads B
  if (c_RightEncoderIsReversed)
    _RightEncoderTicks -= (_RightEncoderASet != _RightEncoderBSet) ? +1 : -1;
  else
    _RightEncoderTicks += (_RightEncoderASet != _RightEncoderBSet) ? +1 : -1;
}

