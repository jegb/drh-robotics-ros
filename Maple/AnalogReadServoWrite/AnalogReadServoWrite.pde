#include <Servo.h> 

const int c_AnalogInPin = 15; // Analog input pin that the potentiometer is attached to

const int c_RightServoOutPin = 8;    // Output pin that the servo is attached to
const int c_LeftServoOutPin = 9;    // Output pin that the servo is attached to

Servo _RightServo;
Servo _LeftServo;

#define c_MaxMotorCV 90      // range is 0 ... 90 (half servo range)



void setup()
{
    // Configure the ADC pin
    pinMode(c_AnalogInPin, INPUT_ANALOG);

    _RightServo.attach(c_RightServoOutPin);  // attaches the servo on specified pin to the servo object 
    _LeftServo.attach(c_LeftServoOutPin);  // attaches the servo on specified pin to the servo object 
}

void loop()
{
    int sensorValue = analogRead(c_AnalogInPin);
    float normalizedRightMotorCV = sensorValue / 4095.0;
    float normalizedLeftMotorCV = normalizedRightMotorCV;
    
    float rightServoValue = mapFloat(normalizedRightMotorCV, 0, 1, 90.0 - c_MaxMotorCV, 90.0 + c_MaxMotorCV);     // scale it to use it with the servo (value between 0 and 180) 
    float leftServoValue = mapFloat(normalizedLeftMotorCV, 0, 1, 90.0 - c_MaxMotorCV, 90.0 + c_MaxMotorCV);     // scale it to use it with the servo (value between 0 and 180) 
    
    _RightServo.write(rightServoValue);
    _LeftServo.write(leftServoValue);
    
    if(SerialUSB.isConnected() && (SerialUSB.getDTR() || SerialUSB.getRTS()))
    {
      SerialUSB.print("sensor = " );
      SerialUSB.print(sensorValue);
      SerialUSB.print(", normalizedRightMotorCV = " );
      SerialUSB.print(normalizedRightMotorCV);
      SerialUSB.print(", servo = " );
      SerialUSB.println(rightServoValue);
    }

    delay(20);         // Wait for x milliseconds
}

long mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

