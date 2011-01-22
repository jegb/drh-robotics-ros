/*
  SpeedController.h - Contains the PID controllers to control the speed of the robot

  Dr. Rainer Hessmer,December, 2010.
  Released into the public domain.
*/

#ifndef SpeedController_h
#define SpeedController_h

#include "OdometricLocalizer.h"
#include "RobotParams.h"
#include "TimeInfo.h"
#include "WProgram.h"

class SpeedController
{
private:
	float _PParam, _IParam, _DParam;
	QuadratureEncoder* _pLeftEncoder;
	QuadratureEncoder* _pRightEncoder;
	long _PreviousLeftCounts;
	long _PreviousRightCounts;

	RobotParams* _pRobotParams;
	TimeInfo* _pTimeInfo;

	float _LastLeftError, _LastRightError;
	float _ErrorIntegralAcrossWheels;
	
public:
	
	float PParam;
	float IParam;
	float DParam;

	float DesiredVelocity;
	float DesiredAngularVelocity;

	// normalized control values for the left and right motor
	float NormalizedLeftCV;
	float NormalizedRightCV;

	SpeedController(QuadratureEncoder* pLeftEncoder, QuadratureEncoder* pRightEncoder, RobotParams* pRobotParams, TimeInfo* pTimeInfo)
	{
		_pLeftEncoder = pLeftEncoder;
		_pRightEncoder = pRightEncoder;
		_PreviousLeftCounts = _pLeftEncoder->GetPosition();
		_PreviousRightCounts = pRightEncoder->GetPosition();

		_pRobotParams = pRobotParams;
		_pTimeInfo = pTimeInfo;

		PParam = 0.0;
		IParam = 0.0;
		DParam = 0.0;

		DesiredVelocity = 0.0;
		DesiredAngularVelocity = 0.0;
	}
	
	void Update()
	{
		float angularVelocityCountsOffset = DesiredAngularVelocity * _pTimeInfo->SecondsSinceLastUpdate / _pRobotParams->RadiansPerCount;
		float expectedBaseCounts = DesiredVelocity * _pTimeInfo->SecondsSinceLastUpdate / _pRobotParams->DistancePerCount;
		
		float expectedLeftCount = expectedBaseCounts - angularVelocityCountsOffset;
		float expectedRightCount = expectedBaseCounts + angularVelocityCountsOffset;
		
		long leftCounts = _pLeftEncoder->GetPosition();
		long rightCounts = _pRightEncoder->GetPosition();

		long deltaLeft = leftCounts - _PreviousLeftCounts;
		long deltaRight = rightCounts - _PreviousRightCounts;

		_PreviousLeftCounts = leftCounts;
		_PreviousRightCounts = rightCounts;

		float leftError = expectedLeftCount - (float)deltaLeft;
		float rightError = expectedLeftCount - (float)deltaRight;
		
		float errorAcrossWheels = (float)deltaLeft - (float)deltaRight + 2 * angularVelocityCountsOffset;
		_ErrorIntegralAcrossWheels += errorAcrossWheels * _pTimeInfo->SecondsSinceLastUpdate;
		
		// in order to avoid adding up an ever increasing integral term we cap it so that the integral contribution is never greater than the max CV
		float integralContribution = IParam * _ErrorIntegralAcrossWheels;
		if (integralContribution > 1.0)
		{
			integralContribution = 1.0;
		}
		else if (integralContribution < -1.0)
		{
			integralContribution = -1.0;
		}
		
		NormalizedLeftCV = PParam * leftError + DParam * (leftError - _LastLeftError) - integralContribution;
		NormalizedLeftCV = constrain(NormalizedLeftCV, -1, +1);

		NormalizedRightCV = PParam * rightError + DParam * (rightError - _LastRightError) + integralContribution;
		NormalizedRightCV = constrain(NormalizedRightCV, -1, +1);
		
		_LastLeftError = leftError;
		_LastRightError = rightError;
	}
};

#endif
