/*
 * CoreXYUKinematics.h
 *
 *  Created on: 4 Jun 2017
 *      Author: Lars
 */

#ifndef SRC_MOVEMENT_KINEMATICS_COREXYUKINEMATICS_H_
#define SRC_MOVEMENT_KINEMATICS_COREXYUKINEMATICS_H_

#include "CoreBaseKinematics.h"

const size_t U_AXIS = 3; // X2
const size_t V_AXIS = 4; // Y2
const size_t COREXYU_AXES = 5;

class CoreXYUKinematics : public CoreBaseKinematics
{
public:
	CoreXYUKinematics();

	// Overridden base class functions. See Kinematics.h for descriptions.
	const char *GetName(bool forStatusReport) const override;
	void MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numDrives, float machinePos[]) const override;

protected:
	float MotorFactor(size_t drive, const float directionVector[]) const override;
};

#endif /* SRC_MOVEMENT_KINEMATICS_COREXYKINEMATICS_H_ */
