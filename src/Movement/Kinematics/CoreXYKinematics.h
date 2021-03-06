/*
 * CoreXYKinematics.h
 *
 *  Created on: 6 May 2017
 *      Author: David
 */

#ifndef SRC_MOVEMENT_KINEMATICS_COREXYKINEMATICS_H_
#define SRC_MOVEMENT_KINEMATICS_COREXYKINEMATICS_H_

#include "CoreBaseKinematics.h"

class CoreXYKinematics : public CoreBaseKinematics
{
public:
	CoreXYKinematics();

	// Overridden base class functions. See Kinematics.h for descriptions.
	const char *GetName(bool forStatusReport) const override;
	void MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numDrives, float machinePos[]) const override;

protected:
	float MotorFactor(size_t drive, const float directionVector[]) const override;
};

#endif /* SRC_MOVEMENT_KINEMATICS_COREXYKINEMATICS_H_ */
