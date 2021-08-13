/*
 * IndividualElement.cpp
 *
 *  Created on: 14-Jul-2008
 *      Author: malem303
 */

#include "IndividualElement.h"

#include "Utils.h"
#include "Constants.h"
#include "CenterOfGravity.h"

IndividualElement::IndividualElement(matrix<double> selfInducedWashVelocityModel, vector<double> distanceIELAFromReferenceAxes, vector<double> rotationAnglesIERAFromIELA) :
	distanceIELAFromReferenceAxes(distanceIELAFromReferenceAxes), rotationAnglesIERAFromIELA(rotationAnglesIERAFromIELA),
	aerodynamicVelocityIELA(Constants::ZERO_VEC_3), washVelocityIELA(Constants::ZERO_VEC_3), washAngularVelocityIELA(Constants::ZERO_VEC_3), inertialVelocityIELA(Constants::ZERO_VEC_3), inertialAngularVelocityIELA(Constants::ZERO_VEC_3), forcesIELA(Constants::ZERO_VEC_3), momentsIELA(Constants::ZERO_VEC_3), selfInducedWashVelocityModel(selfInducedWashVelocityModel)
{
}

IndividualElement::~IndividualElement()
{
	// TODO Auto-generated destructor stub
}

vector<double> IndividualElement::computeDistanceIELAFromCenterOfGravity(IndividualElement & centerOfGravity)
{
	return prod(
		Utils::transformationCenterOfGravityFromReferenceAxes(), // reference axes aren't oriented like IELA axes
		(distanceIELAFromReferenceAxes - centerOfGravity.getDistanceIELAFromReferenceAxes())
		);
}

vector<double> IndividualElement::computeSelfInducedWashVelocity(double density)
{
	double aerodynamicVelocityIELANorm = Utils::vectorNorm(aerodynamicVelocityIELA);


	if (aerodynamicVelocityIELANorm == 0.0 || density == 0.0)
		return Constants::ZERO_VEC_3;

	return  (-1.0 / (2.0 * density * aerodynamicVelocityIELANorm)) * prod(selfInducedWashVelocityModel, forcesIELA);
}

void IndividualElement::updatePhysics(IndividualElement & centerOfGravity, double density)
{
	/*! step 1 : translate earth velocities to IELA
	 * Note : this method is virtual redefine this step as needed
	 */
	translateInertialVelocitiesToIELA(centerOfGravity);

	/*! step 2 : compute aerodynamic velocities
	 * Note : this method is virtual redefine this step as needed
	 */
	computeAerodynamicVelocities(density);

	/*! step 3 : compute forcesIELA and momentsIELA and translate them to IELA
	 * Note : this method is virtual redefine this step as needed
	 */
	computeForcesAndMomentsAtIELA(centerOfGravity, density);

	/*! step 4 : add this IndividualElement's contribution to center of gravity's forcesIELA and momentsIELA
	 * Note : this method is virtual redefine this step as needed
	 */
	sumForcesAndMomentsToCenterOfGravity(centerOfGravity);


}



void IndividualElement::translateInertialVelocitiesToIELA(IndividualElement & centerOfGravity)
{
	// Translate CoG velocities to IELA :
	vector<double> distanceIELAFromIndividualElement = computeDistanceIELAFromCenterOfGravity(centerOfGravity);


	inertialVelocityIELA = centerOfGravity.getInertialVelocityIELA() + Utils::crossProduct(centerOfGravity.getInertialAngularVelocityIELA(), distanceIELAFromIndividualElement);
	inertialAngularVelocityIELA = centerOfGravity.getInertialAngularVelocityIELA();
}

void IndividualElement::computeAerodynamicVelocities(double density)
{
	aerodynamicVelocityIELA = inertialVelocityIELA - washVelocityIELA;
	aerodynamicAngularVelocityIELA = inertialAngularVelocityIELA - washAngularVelocityIELA;
}


void IndividualElement::computeForcesAndMomentsAtIELA(IndividualElement & centerOfGravity, double density)
{
	// by default do nothing!
}


void IndividualElement::sumForcesAndMomentsToCenterOfGravity(IndividualElement & centerOfGravity)
{
	// add to center of gravity
	// TODO : begin semaphore
//	CenterOfGravity & refCenterOfGravity = dynamic_cast<CenterOfGravity&>(centerOfGravity);
	centerOfGravity.setForcesIELA(centerOfGravity.getForcesIELA() + forcesIELA);
	centerOfGravity.setMomentsIELA(centerOfGravity.getMomentsIELA() + momentsIELA + Utils::crossProduct(computeDistanceIELAFromCenterOfGravity(centerOfGravity), forcesIELA));
	// TODO : end  semaphore
}

void IndividualElement::updtateWashVelocities(double density, vector<double> externalWashVelocity, vector<double> externalWashAngularVelocity)
{
	washVelocityIELA = computeSelfInducedWashVelocity(density) + externalWashVelocity;
	//TODO angular wash
}
