/*
 * Fuselage.cpp
 *
 *  Created on: 14-Jul-2008
 *      Author: malem303
 */

#include "Fuselage.h"
#include "Utils.h"
Fuselage::Fuselage(double caracteristicArea, double caracteristicLenght, matrix<double> selfInducedWashVelocityModel, matrix<double> mutualInfluenceModelForForces, matrix<double> mutualInfluenceModelForMoments, vector<double> distanceIELAFromReferenceAxes, vector<double> rotationAnglesIERAFromIELA) :
	IndividualElement(selfInducedWashVelocityModel, distanceIELAFromReferenceAxes, rotationAnglesIERAFromIELA), caracteristicArea(caracteristicArea), caracteristicLenght(caracteristicLenght), mutualInfluenceModelForForces(mutualInfluenceModelForForces), mutualInfluenceModelForMoments(mutualInfluenceModelForMoments)
{
	// TODO Auto-generated constructor stub

}


Fuselage::~Fuselage() {
	// TODO Auto-generated destructor stub
}

void Fuselage::computeForcesAndMomentsAtIELA(IndividualElement & centerOfGravity, double density)
{

	matrix<double> transformation = Utils::transformationMatrix(rotationAnglesIERAFromIELA);
	vector<double> aerodynamicVelocityIERA = prod(Utils::transformationMatrix(rotationAnglesIERAFromIELA), aerodynamicVelocityIELA);
	vector<double> aerodynamicAngularVelocityIERA = prod(Utils::transformationMatrix(rotationAnglesIERAFromIELA), aerodynamicAngularVelocityIELA);

	// forcesIELA and momentsIELA

	double forceCoefficient = (0.5 * density * caracteristicArea * aerodynamicVelocityIERA(0));

	vector<double> forcesIERA = forceCoefficient * prod(-mutualInfluenceModelForForces, aerodynamicVelocityIERA);
	vector<double> momentsIERA = forceCoefficient * caracteristicLenght * prod(mutualInfluenceModelForMoments, aerodynamicVelocityIERA);

	forcesIELA = prod(trans(Utils::transformationMatrix(rotationAnglesIERAFromIELA)), forcesIERA);
	momentsIELA = prod(trans(Utils::transformationMatrix(rotationAnglesIERAFromIELA)), momentsIERA);


}

