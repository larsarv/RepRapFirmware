/*
 * CoreXYUKinematics.cpp
 *
 *  Created on: 4 Jun 2017
 *      Author: Lars
 */

#include "CoreXYUKinematics.h"
#include "GCodes/GCodes.h"

CoreXYUKinematics::CoreXYUKinematics() : CoreBaseKinematics(KinematicsType::coreXYU)
{
}

// Return the name of the current kinematics
const char *CoreXYUKinematics::GetName(bool forStatusReport) const
{
	return (forStatusReport) ? "coreXYU" : "CoreXYU";
}

// Convert motor coordinates to machine coordinates. Used after homing and after individual motor moves.
void CoreXYUKinematics::MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numDrives, float machinePos[]) const
{
	// Convert the axes
	machinePos[X_AXIS] = ((motorPos[X_AXIS] * stepsPerMm[Y_AXIS]) - (motorPos[Y_AXIS] * stepsPerMm[X_AXIS]))
								/(2 * axisFactors[X_AXIS] * stepsPerMm[X_AXIS] * stepsPerMm[Y_AXIS]);
	machinePos[Y_AXIS] = ((motorPos[X_AXIS] * stepsPerMm[Y_AXIS]) + (motorPos[Y_AXIS] * stepsPerMm[X_AXIS]))
								/(2 * axisFactors[Y_AXIS] * stepsPerMm[X_AXIS] * stepsPerMm[Y_AXIS]);
	machinePos[U_AXIS] = ((motorPos[U_AXIS] * stepsPerMm[V_AXIS]) - (motorPos[V_AXIS] * stepsPerMm[U_AXIS]))
								/(2 * axisFactors[V_AXIS] * stepsPerMm[U_AXIS] * stepsPerMm[V_AXIS]);
	machinePos[V_AXIS] = ((motorPos[U_AXIS] * stepsPerMm[V_AXIS]) + (motorPos[V_AXIS] * stepsPerMm[U_AXIS]))
								/(2 * axisFactors[V_AXIS] * stepsPerMm[U_AXIS] * stepsPerMm[V_AXIS]);


	machinePos[Z_AXIS] = motorPos[Z_AXIS]/stepsPerMm[Z_AXIS];

	// Convert any additional axes and the extruders
	for (size_t drive = COREXYU_AXES; drive < numDrives; ++drive)
	{
		machinePos[drive] = motorPos[drive]/stepsPerMm[drive];
	}
}

// Calculate the movement fraction for a single axis motor
float CoreXYUKinematics::MotorFactor(size_t drive, const float directionVector[]) const
{
	switch(drive)
	{
	case X_AXIS:
		return (directionVector[X_AXIS] * axisFactors[X_AXIS]) + (directionVector[Y_AXIS] * axisFactors[Y_AXIS]);
	case Y_AXIS:
		return (directionVector[Y_AXIS] * axisFactors[Y_AXIS]) - (directionVector[X_AXIS] * axisFactors[X_AXIS]);
	case U_AXIS: // X2, Use Y and U to calculate
		return (directionVector[U_AXIS] * axisFactors[U_AXIS]) + (directionVector[Y_AXIS] * axisFactors[Y_AXIS]);
	case V_AXIS: // Y2, Use Y and U to calculate
		return (directionVector[Y_AXIS] * axisFactors[Y_AXIS]) - (directionVector[U_AXIS] * axisFactors[U_AXIS]);
	default:
		return directionVector[drive];
	}
}

// End
