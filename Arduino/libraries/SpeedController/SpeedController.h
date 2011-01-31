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
	OdometricLocalizer* _pOdometricLocalizer;

	RobotParams* _pRobotParams;
	TimeInfo* _pTimeInfo;

	float _LastLeftError, _LastRightError;

	float _LeftErrorIntegral;
	float _RightErrorIntegral;
	//float _ErrorIntegralAcrossWheels;

	float ClipIntegralContribution(float integralContribution)
	{
		if (integralContribution > 1.0)
		{
			return 1.0;
		}
		else if (integralContribution < -1.0)
		{
			return -1.0;
		}
	} 
	
public:
	
	float PParam;
	float IParam;
	float DParam;

	float DesiredVelocity;
	float DesiredAngularVelocity;

	float LeftError;
	float RightError;

	// normalized control values for the left and right motor
	float NormalizedLeftCV;
	float NormalizedRightCV;

	SpeedController(OdometricLocalizer* pOdometricLocalizer, RobotParams* pRobotParams, TimeInfo* pTimeInfo)
	{
		_pOdometricLocalizer = pOdometricLocalizer;

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
		float angularVelocityOffset = 0.5 * DesiredAngularVelocity * _pRobotParams->TrackWidth;
		
		float expectedLeftSpeed = DesiredVelocity - angularVelocityOffset;
		float expectedRightSpeed = DesiredVelocity + angularVelocityOffset;

		LeftError = expectedLeftSpeed - _pOdometricLocalizer->VLeft;
		RightError = expectedRightSpeed - _pOdometricLocalizer->VRight;

		_LeftErrorIntegral += LeftError;
		_RightErrorIntegral += RightError;

		// Avoiding runaway integral error contributions
		float maxIntegralValue = 1.0 / IParam;
		_LeftErrorIntegral = constrain(_LeftErrorIntegral, -maxIntegralValue, +maxIntegralValue);
		_RightErrorIntegral = constrain(_RightErrorIntegral, -maxIntegralValue, +maxIntegralValue);

		float leftErrorDifferential = (LeftError - _LastLeftError);
		float rightErrorDifferential = (RightError - _LastRightError);
		
		//float errorAcrossWheels = _pOdometricLocalizer->VLeft - _pOdometricLocalizer->VRight + 2 * angularVelocityOffset;
		//_ErrorIntegralAcrossWheels += errorAcrossWheels * _pTimeInfo->SecondsSinceLastUpdate;
		
		NormalizedLeftCV = PParam * LeftError + DParam * leftErrorDifferential + IParam * _LeftErrorIntegral;
		NormalizedLeftCV = constrain(NormalizedLeftCV, -1, +1);

		NormalizedRightCV = PParam * RightError + DParam * rightErrorDifferential + IParam * _RightErrorIntegral;
		NormalizedRightCV = constrain(NormalizedRightCV, -1, +1);
		
		_LastLeftError = LeftError;
		_LastRightError = RightError;
	}
};

#endif
