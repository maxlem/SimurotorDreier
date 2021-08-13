/*
 * CenterOfGravity.cpp
 *
 *  Created on: 17-Jul-2008
 *      Author: malem303
 */

#include "CenterOfGravity.h"
#include "Utils.h"
#include "IndividualElement.h"
#include "Constants.h"

using namespace boost::numeric::ublas;

CenterOfGravity::CenterOfGravity(vector<double> distanceIELAFromReferenceAxes, double mass, matrix<double> intertiaTensor, vector<double> positionEarthAxes, vector<double> angularPositionEarthAxes, vector<double> positionDerivativeEarthAxes, vector<double> angularPositionDerivativeEarthAxes, vector<double> inertialVelocityDerivativeCenterOfGravity, vector<double> inertialAngularVelocityDerivativeCenterOfGravity):
	IndividualElement(Constants::ZERO3x3, distanceIELAFromReferenceAxes), positionEarthAxes(positionEarthAxes),
	angularPositionEarthAxes(angularPositionEarthAxes), positionDerivativeEarthAxes(positionDerivativeEarthAxes), angularPositionDerivativeEarthAxes(angularPositionDerivativeEarthAxes),
	inertialVelocityDerivativeCenterOfGravity(inertialVelocityDerivativeCenterOfGravity), inertialAngularVelocityDerivativeCenterOfGravity(inertialAngularVelocityDerivativeCenterOfGravity),
	interiaTensor(intertiaTensor), mass(mass)
{

}

CenterOfGravity::~CenterOfGravity() {
	// TODO Auto-generated destructor stub
}

matrix<double> CenterOfGravity::buildMassMatrix(double mass) const
{
	return mass * Constants::IDENTITY3x3;
}
vector<double> CenterOfGravity::getState() const
{
	vector<double> state(12);

	for (int i = 0; i < 3; i++)
	{
		state(i) = positionEarthAxes(i);
		state(i+3) = inertialVelocityIELA(i);
		state(i+6) = inertialAngularVelocityIELA(i);
		state(i+9) = angularPositionEarthAxes(i);
	}

    return state;
}


void CenterOfGravity::addGravityForces()
{
	// TODO : for the moment, the mass and mass distribution remain constant,
	// it should be updated with fuel burn rate, cargo unloading, etc. See Dreier 2007 p.44 for an insight

	forcesIELA += prod(Utils::transformationMatrix(angularPositionEarthAxes), Utils::toVec3(0.0, 0.0, Constants::GRAVITY * mass));

}

void CenterOfGravity::resetForces()
{
	// TODO : for the moment, the mass and mass distribution remain constant,
	// it should be updated with fuel burn rate, cargo unloading, etc. See Dreier 2007 p.44 for an insight

	forcesIELA = Constants::ZERO_VEC_3;
	momentsIELA = Constants::ZERO_VEC_3;

}

void CenterOfGravity::updateVelocities()
{
	inertialVelocityIELA = prod(Utils::transformationMatrix(angularPositionEarthAxes), positionDerivativeEarthAxes);
	inertialAngularVelocityIELA = prod(Utils::buildEulerTransformationMatrix(angularPositionEarthAxes(0), angularPositionEarthAxes(1)), angularPositionDerivativeEarthAxes);
}

void CenterOfGravity::updateDerivatives()
{
	positionDerivativeEarthAxes = prod(trans(Utils::transformationMatrix(angularPositionEarthAxes)), inertialVelocityIELA);

	inertialVelocityDerivativeCenterOfGravity = prod(Utils::invertMatrix(buildMassMatrix(mass)), (forcesIELA - Utils::crossProduct(inertialAngularVelocityIELA, mass * inertialVelocityIELA)));

	matrix<double> inertiaTensorInverse= Utils::invertMatrix(interiaTensor);
	//TODO if the matrix is singular, manage error

	inertialAngularVelocityDerivativeCenterOfGravity = prod(inertiaTensorInverse, (momentsIELA - Utils::crossProduct(inertialAngularVelocityIELA, prod(interiaTensor, inertialAngularVelocityIELA))));
	matrix<double> eulerTransformationInverse = Utils::buildEulerTransformationMatrixInverse(angularPositionEarthAxes(0), angularPositionEarthAxes(1));
	//TODO if the matrix is singular, manage error

	angularPositionDerivativeEarthAxes = prod(eulerTransformationInverse, inertialAngularVelocityIELA);
}

