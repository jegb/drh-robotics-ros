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
#include "WProgram.h"

#define PI 3.14159265
#define TwoPI 6.28318531


class OdometricLocalizer
{
	/*
	*/

private:
	QuadratureEncoder* _pLeftEncoder;
	QuadratureEncoder* _pRightEncoder;
	float _DistancePerCount;
	float _RadiansPerCount;
	long _PreviousLeftCounts;
	long _PreviousRightCounts;

public:
	// pinA and pinB must be one of the external interupt pins
	OdometricLocalizer(QuadratureEncoder* pLeftEncoder, QuadratureEncoder* pRightEncoder, float wheelDiameter, float trackWidth, int countsPerRevolution)
	{
		_pLeftEncoder = pLeftEncoder;
		_pRightEncoder = pRightEncoder;

		_DistancePerCount = (PI * wheelDiameter) / (float)countsPerRevolution;
		_RadiansPerCount = PI * (wheelDiameter / trackWidth) / countsPerRevolution;

		_PreviousLeftCounts = _pLeftEncoder->GetPosition();
		_PreviousRightCounts = pRightEncoder->GetPosition();
	}

	float X;
	float Y;
	float HeadingRad;

	// Must be periodically called
	void Update()
	{
		long leftCounts = _pLeftEncoder->GetPosition();
		long rightCounts = _pRightEncoder->GetPosition();

		long deltaLeft = leftCounts - _PreviousLeftCounts;
		long deltaRight = rightCounts - _PreviousRightCounts;

		float deltaDistance = 0.5f * (float)(deltaLeft + deltaRight) * _DistancePerCount;
		float deltaX = deltaDistance * (float)cos(HeadingRad);
		float deltaY = deltaDistance * (float)sin(HeadingRad);
		float deltaHeading = (float)(deltaRight - deltaLeft) * _RadiansPerCount;

		X += deltaX;
		Y += deltaY;
		HeadingRad += deltaHeading;
		// limit heading to -Pi <= heading < Pi
		if (HeadingRad > PI)
		{
			HeadingRad -= TwoPI;
		}
		else
		{
			if (HeadingRad <= -PI)
			{
				HeadingRad += TwoPI;
			}
		}

		_PreviousLeftCounts = leftCounts;
		_PreviousRightCounts = rightCounts;
	}
};

#endif
