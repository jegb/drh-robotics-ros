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
	RobotParams* _pRobotParams;
	TimeInfo* _pTimeInfo;

	float _LastLeftError, _LastRightError;
	float _ErrorIntegralAcrossWheels;
	
public:

	SpeedController(float pParam, float iParam, float dParam, QuadratureEncoder* pLeftEncoder, QuadratureEncoder* pRightEncoder, RobotParams* pRobotParams, TimeInfo* pTimeInfo)
	{
		_PParam = pParam;
		_IParam = iParam;
		_DParam = dParam;

		_pLeftEncoder = pLeftEncoder;
		_pRightEncoder = pRightEncoder;

		_pRobotParams = pRobotParams;
		_pTimeInfo = pTimeInfo;
	}
	
	// normalized control values for the left and right motor
	float NormalizedLeftCV;
	float NormalizedRightCV;
	
	void Update(float desiredVelocity, float desiredAngularVelocity)
	{
		float angularVelocityCountsOffset = desiredAngularVelocity * _pTimeInfo->SecondsSinceLastUpdate / _pRobotParams->RadiansPerCount;
		float expectedBaseCounts = desiredVelocity * _pTimeInfo->SecondsSinceLastUpdate / _pRobotParams->DistancePerCount;
		
		float expectedLeftCount = expectedBaseCounts - angularVelocityCountsOffset;
		float expectedRightCount = expectedBaseCounts + angularVelocityCountsOffset;
		
		float leftError = expectedLeftCount - (float)_pLeftEncoder->GetPosition();
		float rightError = expectedLeftCount - (float)_pLeftEncoder->GetPosition();
		
		float errorAcrossWheels = (float)_pLeftEncoder->GetPosition() - (float)_pLeftEncoder->GetPosition() + 2 * angularVelocityCountsOffset;
		_ErrorIntegralAcrossWheels = errorAcrossWheels * _pTimeInfo->SecondsSinceLastUpdate;
		
		NormalizedLeftCV = _PParam * leftError + _DParam * (leftError - _LastLeftError) - _IParam * _ErrorIntegralAcrossWheels;
		NormalizedLeftCV = constrain(NormalizedLeftCV, -1, +1);

		NormalizedRightCV = _PParam * rightError + _DParam * (rightError - _LastRightError) + _IParam * _ErrorIntegralAcrossWheels;
		NormalizedRightCV = constrain(NormalizedRightCV, -1, +1);
		
		_LastLeftError = leftError;
		_LastRightError = rightError;
	}
};

#endif
