/*
  OdometricLocalizer.h - Quadrature encoder library.
  Dr. Rainer Hessmer, December, 2010.
  Released into the public domain.

  Inspired by the code from http://www.ridgesoft.com/tutorials.htm,
  specifically http://www.ridgesoft.com/articles/trackingposition/TrackingPosition.pdf
*/

#ifndef OdometricLocalizer_h
#define OdometricLocalizer_h

#include "RobotParams.h"
#include "TimeInfo.h"

#define TwoPI 6.28318531


class OdometricLocalizer
{
private:
	RobotParams* _pRobotParams;
	TimeInfo* _pTimeInfo;
	double _DistancePerCount;
	double _RadiansPerCount;

	long _PreviousLeftEncoderCounts;
	long _PreviousRightEncoderCounts;

public:
	OdometricLocalizer(RobotParams* pRobotParams, TimeInfo* pTimeInfo)
	{
		_pRobotParams = pRobotParams;
		_pTimeInfo = pTimeInfo;
	}

	double X;  // x coord in global frame
	double Y;  // y coord in global frame
	double Heading;  // heading (radians) in the global frame. The value lies in (-PI, PI]
	
	double VLeft;   // left motor speed
	double VRight;  // right motor speed
	double V;  // forward speed
	double Omega;  // angular speed (radians per sec)

	// Must be periodically called
	void Update(long leftEncoderCounts, long rightEncoderCounts)
	{
		long deltaLeft = leftEncoderCounts - _PreviousLeftEncoderCounts;
		long deltaRight = rightEncoderCounts - _PreviousRightEncoderCounts;

		VLeft = deltaLeft * _pRobotParams->DistancePerCount / _pTimeInfo->SecondsSinceLastUpdate;
		VRight = deltaRight * _pRobotParams->DistancePerCount / _pTimeInfo->SecondsSinceLastUpdate;

		double deltaDistance = 0.5f * (double)(deltaLeft + deltaRight) * _pRobotParams->DistancePerCount;
		double deltaX = deltaDistance * (double)cos(Heading);
		double deltaY = deltaDistance * (double)sin(Heading);

		double deltaHeading = (double)(deltaRight - deltaLeft) * _pRobotParams->RadiansPerCount;

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

		_PreviousLeftEncoderCounts = leftEncoderCounts;
		_PreviousRightEncoderCounts = rightEncoderCounts;
	}

	void Reset()
	{
		X = 0.0;  // x coord in global frame
		Y = 0.0;  // y coord in global frame
		Heading = 0.0;  // heading (radians) in the global frame. The value lies in (-PI, PI]
	
		VLeft = 0.0;   // left motor speed
		VRight = 0,0;  // right motor speed
		V = 0.0;  // forward speed
		Omega = 0.0;  // angular speed (radians per sec)

		_PreviousLeftEncoderCounts = 0;
		_PreviousRightEncoderCounts = 0;
	}
};

#endif
