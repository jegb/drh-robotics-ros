/*
  OdometricLocalizer.h - Quadrature encoder library.
  Dr. Rainer Hessmer, December, 2010.
  Released into the public domain.

  Inspired by the code from http://www.ridgesoft.com/tutorials.htm,
  specifically http://www.ridgesoft.com/articles/trackingposition/TrackingPosition.pdf
*/

#ifndef OdometricLocalizer_h
#define OdometricLocalizer_h

#include "QuadratureEncoder.h"
#include "RobotParams.h"
#include "TimeInfo.h"
#include "WProgram.h"

#define PI 3.14159265
#define TwoPI 6.28318531


class OdometricLocalizer
{
private:
	QuadratureEncoder* _pLeftEncoder;
	QuadratureEncoder* _pRightEncoder;
	RobotParams* _pRobotParams;
	TimeInfo* _pTimeInfo;
	float _DistancePerCount;
	float _RadiansPerCount;
	long _PreviousLeftCounts;
	long _PreviousRightCounts;

public:
	// pinA and pinB must be one of the external interupt pins
	OdometricLocalizer(QuadratureEncoder* pLeftEncoder, QuadratureEncoder* pRightEncoder, RobotParams* pRobotParams, TimeInfo* pTimeInfo)
	{
		_pLeftEncoder = pLeftEncoder;
		_pRightEncoder = pRightEncoder;
		_pRobotParams = pRobotParams;
		_pTimeInfo = pTimeInfo;

		_PreviousLeftCounts = _pLeftEncoder->GetPosition();
		_PreviousRightCounts = pRightEncoder->GetPosition();
	}

	float X;  // x coord in global frame
	float Y;  // y coord in global frame
	float Heading;  // heading (radians) in the global frame. The value lies in (-PI, PI]
	
	float V;  // forward speed
	float Omega;  // angular speed (radians per sec)

	// Must be periodically called
	void Update()
	{
		long leftCounts = _pLeftEncoder->GetPosition();
		long rightCounts = _pRightEncoder->GetPosition();

		long deltaLeft = leftCounts - _PreviousLeftCounts;
		long deltaRight = rightCounts - _PreviousRightCounts;

		float deltaDistance = 0.5f * (float)(deltaLeft + deltaRight) * _pRobotParams->DistancePerCount;
		float deltaX = deltaDistance * (float)cos(Heading);
		float deltaY = deltaDistance * (float)sin(Heading);
		float deltaHeading = (float)(deltaRight - deltaLeft) * _pRobotParams->RadiansPerCount;

		X += deltaX;
		Y += deltaY;
		Heading += deltaHeading;
		// limit heading to -Pi <= heading < Pi
		if (Heading > PI)
		{
			Heading -= TwoPI;
		}
		else
		{
			if (Heading <= -PI)
			{
				Heading += TwoPI;
			}
		}
		
		V = deltaDistance / _pTimeInfo->SecondsSinceLastUpdate;
		Omega = deltaHeading / _pTimeInfo->SecondsSinceLastUpdate;

		_PreviousLeftCounts = leftCounts;
		_PreviousRightCounts = rightCounts;
	}
};

#endif
