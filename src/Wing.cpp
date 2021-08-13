/*
 * Wing.cpp
 *
 *  Created on: 15-Jul-2008
 *      Author: malem303
 */

#include "Wing.h"
#include  "Utils.h"

Wing::Wing(double span, double chordAtRoot, double chordAtTip, double dihedralAngle, double incidenceAngle, double sweepAngle, double liftCurveSlope,  vector<double> dragCoefficientParameters, vector<double> momentCoefficientParameters, matrix<double> selfInducedWashVelocityModel) :
	IndividualElement(selfInducedWashVelocityModel),
	rightWing(RIGHT_WING, span/2, chordAtRoot, chordAtTip, Utils::toVec3(-dihedralAngle, incidenceAngle, sweepAngle), liftCurveSlope, dragCoefficientParameters, momentCoefficientParameters, selfInducedWashVelocityModel),
	leftWing(LEFT_WING, span/2, chordAtRoot, chordAtTip, Utils::toVec3(dihedralAngle, incidenceAngle, -sweepAngle), liftCurveSlope, dragCoefficientParameters, momentCoefficientParameters, selfInducedWashVelocityModel)
{
	// TODO Auto-generated constructor stub

}

Wing::~Wing() {
	// TODO Auto-generated destructor stub
}

void Wing::translateInertialVelocitiesToIELA(IndividualElement & centerOfGravity)
{
	rightWing.translateInertialVelocitiesToIELA(centerOfGravity);
	leftWing.translateInertialVelocitiesToIELA(centerOfGravity);
}

void Wing::computeAerodynamicVelocities(double density)
{
	rightWing.computeAerodynamicVelocities(density);
	leftWing.computeAerodynamicVelocities(density);
}


void Wing::computeForcesAndMomentsAtIELA(IndividualElement & centerOfGravity, double density)
{
	rightWing.computeForcesAndMomentsAtIELA(centerOfGravity, density);
	leftWing.computeForcesAndMomentsAtIELA(centerOfGravity, density);
}

/*! updatePhysics step 2 */
void Wing::sumForcesAndMomentsToCenterOfGravity(IndividualElement & centerOfGravity)
{
	rightWing.sumForcesAndMomentsToCenterOfGravity(centerOfGravity);
	leftWing.sumForcesAndMomentsToCenterOfGravity(centerOfGravity);
}
void Wing::updtateWashVelocities(double density, vector<double> externalWashVelocity, vector<double> externalWashAngularVelocity)
{
	//TODO : verify this 0.5 factor
	rightWing.updtateWashVelocities(density, externalWashVelocity, externalWashAngularVelocity);
	leftWing.updtateWashVelocities(density, externalWashVelocity, externalWashAngularVelocity);
}

