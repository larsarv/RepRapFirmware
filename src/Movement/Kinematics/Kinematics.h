/*
 * Kinematics.h
 *
 *  Created on: 24 Apr 2017
 *      Author: David
 */

#ifndef SRC_MOVEMENT_KINEMATICS_H_
#define SRC_MOVEMENT_KINEMATICS_H_

#include "GCodes/GCodeBuffer.h"
#include "Movement/BedProbing/RandomProbePointSet.h"

// Different types of kinematics we support. Each of these has a class to represent it.
// These must have the same numeric assignments as the K parameter of the M669 command, as documented in ???
enum class KinematicsType : uint8_t
{
	cartesian = 0,
	coreXY,
	coreXZ,
	linearDelta,
	scara,
	coreXYU,

	unknown				// this one must be last!
};

// Different types of low-level motion we support
enum class MotionType : uint8_t
{
	linear,
	segmentFreeDelta
};

class Kinematics
{
public:
	// Functions that must be defined in each derived class that implements a kinematics

	// Return the name of the current kinematics.
	// If 'forStatusReport' is true then the string must be the one for that kinematics expected by DuetWebControl and PanelDue.
	// Otherwise it should be in a format suitable fore printing.
	// For any new kinematics, the same string can be returned regardless of the parameter.
	virtual const char *GetName(bool forStatusReport = false) const = 0;

	// Set or report the parameters from a M665, M666 or M669 command
	// If 'mCode' is an M-code used to set parameters for the current kinematics (which should only ever be 665, 666, 667 or 669)
	// then search for parameters used to configure the current kinematics. if any are found, perform appropriate actions and return true.
	// If errors were discovered while processing parameters, put an appropriate error message in 'reply' and set 'error' to true.
	// If no relevant parameters are found, print the existing ones to 'reply' and return false.
	// if 'mCode' does not apply to this kinematics, call the base class version of this function, which will print a suitable error message.
	virtual bool SetOrReportParameters(unsigned int mCode, GCodeBuffer& gb, StringRef& reply, bool& error);

	// Convert Cartesian coordinates to motor positions measured in steps from reference position
	// 'machinePos' is a set of axis and extruder positions to convert
	// 'stepsPerMm' is as configured in M92. On a Scara or polar machine this would actually be steps per degree.
	// 'numAxes' is the number of machine axes to convert, which will always be at least 3
	// 'motorPos' is the output vector of motor positions
	// Return true if successful, false if we were unable to convert
	virtual bool CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numAxes, int32_t motorPos[]) const = 0;

	// Convert motor positions (measured in steps from reference position) to Cartesian coordinates
	// 'motorPos' is the input vector of motor positions
	// 'stepsPerMm' is as configured in M92. On a Scara or polar machine this would actually be steps per degree.
	// 'numDrives' is the number of machine drives to convert, which will always be at least 3
	// 'machinePos' is the output set of converted axis and extruder positions
	virtual void MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numDrives, float machinePos[]) const = 0;

	// Return true if the kinematics supports auto calibration based on bed probing.
	// Normally returns false, but overridden for delta kinematics.
	virtual bool SupportsAutoCalibration() const { return false; }

	// Perform auto calibration. Override this implementation in kinematics that support it.
	virtual void DoAutoCalibration(size_t numFactors, const RandomProbePointSet& probePoints, StringRef& reply)
	pre(SupportsAutoCalibration()) { }

	// Set the default parameters that are changed by auto calibration back to their defaults.
	// Do nothing if auto calibration is not supported.
	virtual void SetCalibrationDefaults() { }

	// Write the parameters that are set by auto calibration to the config-override.g file, returning true if success
	// Just return true if auto calibration is not supported.
	virtual bool WriteCalibrationParameters(FileStore *f) const { return true; }

	// Get the bed tilt fraction for the specified axis.
	// Usually this is only relevant if we are auto calibrating the bed tilt, however you can also specify bed tilt manually if you wanted to.
	virtual float GetTiltCorrection(size_t axis) const { return 0.0; }

	// Return true if the specified XY position is reachable by the print head reference point.
	// The default implementation assumes a rectangular reachable area, so it just used the bed dimensions give in the M208 commands.
	virtual bool IsReachable(float x, float y) const;

	// Limit the Cartesian position that the user wants to move to
	// The default implementation just applies the rectangular limits set up by M208 to those axes that have been homed.
	virtual void LimitPosition(float coords[], size_t numAxes, uint16_t axesHomed) const;

	// Return the set of axes that must have been homed before bed probing is allowed
	// The default implementation requires just X and Y, but some kinematics require additional axes to be homed (e.g. delta, CoreXZ)
	virtual uint16_t AxesToHomeBeforeProbing() const { return (1 << X_AXIS) | (1 << Y_AXIS); }

	// Return the initial Cartesian coordinates we assume after switching to this kinematics
	virtual void GetAssumedInitialPosition(size_t numAxes, float positions[]) const;

	// Override this one if any axes do not use the line motion (e.g. for segmentation-free delta motion)
	virtual MotionType GetMotionType(size_t axis) const { return MotionType::linear; }

	// Override the virtual destructor if your constructor allocates any dynamic memory
	virtual ~Kinematics() { }

	// Factory function to create a particular kinematics object and return a pointer to it.
	// When adding new kinematics, you will need to extend this function to handle your new kinematics type.
	static Kinematics *Create(KinematicsType k);

	// Functions that return information held in this base class
	KinematicsType GetKinematicsType() const { return type; }

	bool UseSegmentation() const { return useSegmentation; }
	bool UseRawG0() const { return useRawG0; }
	float GetSegmentsPerSecond() const pre(UseSegmentation()) { return segmentsPerSecond; }
	float GetMinSegmentLength() const pre(UseSegmentation()) { return minSegmentLength; }

protected:
	// This constructor is used by derived classes that implement non-segmented linear motion
	Kinematics(KinematicsType t);

	// This constructor is used by derived classes that implement segmented linear motion
	Kinematics(KinematicsType t, float segsPerSecond, float minSegLength, bool doUseRawG0);

	float segmentsPerSecond;				// if we are using segmentation, the target number of segments/second
	float minSegmentLength;					// if we are using segmentation, the minimum segment size

private:
	bool useSegmentation;					// true if we have to approximate linear movement using segmentation
	bool useRawG0;							// true if we normally use segmentation but we do not need to segment travel moves
	KinematicsType type;
};

#endif /* SRC_MOVEMENT_KINEMATICS_H_ */
