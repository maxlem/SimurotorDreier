/*
 * Aircraft.cpp
 *
 *  Created on: 11-Jul-2008
 *      Author: malem303
 */

#include "Aircraft.h"
#include "Constants.h"
#include "Utils.h"
#include "ControlRigging.h"
#include "AircraftFactory.h"

#include <math.h>
#include <boost/numeric/ublas/storage.hpp>
#include <boost/numeric/ublas/io.hpp>

#include <list>

Aircraft::Aircraft(bool usePhi) : usePhi(usePhi)
{
	atmosphericConditions = new AtmosphericConditions();
	centerOfGravity = AircraftFactory::createCenterOfGravity(Utils::toVec3(10, 0, 5), Constants::ZERO_VEC_3, Constants::ZERO_VEC_3, Constants::ZERO_VEC_3);
    fuselage = AircraftFactory::createFuselage();
	horizontalStabilizer = AircraftFactory::createHorizontalStabilizer();
	verticalStabilizer = AircraftFactory::createVerticalStabilizer();
	mainRotor = AircraftFactory::createMainRotor();
	tailRotor = AircraftFactory::createTailRotor();
}

Aircraft::~Aircraft() {

	delete atmosphericConditions;
	delete fuselage;
	delete mainRotor;
	delete tailRotor;
	delete horizontalStabilizer;
	delete verticalStabilizer;
	delete centerOfGravity;
}
void Aircraft::updateControls(double collectivePitchStickPosition, double longitudinalSwashplateStickPosition, double lateralSwashplateStickPosition, double tailRotorCollectivePitchStick)
{
	mainRotor->setCollectivePitchStickPosition(collectivePitchStickPosition);
	mainRotor->setLongitudinalSwashplateStickPosition(longitudinalSwashplateStickPosition);
	mainRotor->setLateralSwashplateStickPosition(lateralSwashplateStickPosition);
	tailRotor->setCollectivePitchStickPosition(tailRotorCollectivePitchStick);
}

void Aircraft::updatePhysics()
{

	updateAtmosphericConditions();

	centerOfGravity->resetForces();
	centerOfGravity->updateVelocities();


	fuselage->updatePhysics(*centerOfGravity, atmosphericConditions->getDensity());


	horizontalStabilizer->updatePhysics(*centerOfGravity, atmosphericConditions->getDensity());


	verticalStabilizer->updatePhysics(*centerOfGravity, atmosphericConditions->getDensity());


	mainRotor->updatePhysics(*centerOfGravity, atmosphericConditions->getDensity());


	tailRotor->updatePhysics(*centerOfGravity, atmosphericConditions->getDensity());

	updateWashVelocities();



	centerOfGravity->addGravityForces();


}

void Aircraft::updateWashVelocities()
{
	// TODO : find a less hard-coded way to compute that



	// Mutual influences :


	double density = atmosphericConditions->getDensity();
	vector<double> externalWashAgularVelocity = Constants::ZERO_VEC_3;
	vector<double> externalWashVelocity(3);


	// mainRotor on fuselage
	externalWashVelocity = mainRotor->computeExternalWashInfluence(fuselage, density);
	fuselage->updtateWashVelocities(density, externalWashVelocity, externalWashAgularVelocity);

	// fuselage on mainRotor
	vector<double> fuselageSelfInducedWashVelocity = fuselage->computeSelfInducedWashVelocity(density);

	externalWashVelocity = 	Utils::toVec3(0.0, 0.0, fuselageSelfInducedWashVelocity(2));
	mainRotor->updtateWashVelocities(density, externalWashVelocity, externalWashAgularVelocity);

	// mainRotor and fuselage on horizontalStabilizer
	externalWashVelocity = mainRotor->computeExternalWashInfluence(horizontalStabilizer, density) + Utils::toVec3(fuselageSelfInducedWashVelocity(0), 0.0, 0.0);
	horizontalStabilizer->updtateWashVelocities(density, externalWashVelocity, externalWashAgularVelocity);

	// verticalStabilizer on tailRotor
	vector<double> verticalStabilizerSelfInducedVelocity = verticalStabilizer->computeSelfInducedWashVelocity(density);

	externalWashVelocity = 	Utils::toVec3(0.0, verticalStabilizerSelfInducedVelocity(1), 0.0);
	tailRotor->updtateWashVelocities(density, externalWashVelocity, externalWashAgularVelocity);

	// tailRotor on verticalStabilizer
	vector<double> tailRotorSelfInducedWashVelocity = tailRotor->computeSelfInducedWashVelocity(density);

	externalWashVelocity = 	Utils::toVec3(0.0, tailRotorSelfInducedWashVelocity(1), 0.0);
	verticalStabilizer->updtateWashVelocities(density, externalWashVelocity, externalWashAgularVelocity);



}



void Aircraft::updateAtmosphericConditions()
{
	// TODO: this is a stub!
	// The atmospheric condition change according to meteo (pressure, wind, temperature) and altitude, see Dreier 2007, chap 5
}


void Aircraft::printForces()
{
	using namespace std;

	cout << "----------------------------" << endl;

	cout << "fext" << centerOfGravity->getForcesIELA() << endl;
	cout << "fuse" << fuselage->getForcesIELA() << endl;
	cout << "wing" << horizontalStabilizer->getForcesIELA() << endl;
	cout << "fin " << verticalStabilizer->getForcesIELA() << endl;
	cout << "main " << mainRotor->getForcesIELA() << endl;
	cout << "tail " << tailRotor->getForcesIELA() << endl;

	cout << "----------------------------" << endl;
}

void Aircraft::printWashes()
{
	using namespace std;
	cout << "----------Wash velocities ------------------" << endl;

	cout << "CoG" << centerOfGravity->getWashVelocityIELA() << endl;
	cout << "fuse" << fuselage->getWashVelocityIELA() << endl;
	cout << "wing" << horizontalStabilizer->getWashVelocityIELA() << endl;
	cout << "fin " << verticalStabilizer->getWashVelocityIELA() << endl;
	cout << "main " << mainRotor->getWashVelocityIELA() << endl;
	cout << "tail " << tailRotor->getWashVelocityIELA() << endl;

	cout << "----------------------------" << endl;
}
vector<double> Aircraft::buildTrimVector()
{
	vector<double> trimVector(6);

	trimVector(0) = mainRotor->getCollectivePitch();
	trimVector(1) = mainRotor->getLongitudinalSwashplateAngle();
	trimVector(2) = mainRotor->getLateralSwashplateAngle();
	trimVector(3) = tailRotor->getCollectivePitch();

	vector<double> angularPositionEarthAxes = centerOfGravity->getAngularPositionEarthAxes();
	if(usePhi)
		trimVector(4) =  angularPositionEarthAxes(0);
	else
		trimVector(4) =  angularPositionEarthAxes(2);

	trimVector(5) =  angularPositionEarthAxes(1);

	return trimVector;
}

void Aircraft::applyTrimVector(vector<double> trimVector)
{
	mainRotor->setCollectivePitch(trimVector(0));
	mainRotor->setLongitudinalSwashplateAngle(trimVector(1));
	mainRotor->setLateralSwashplateAngle(trimVector(2));
	tailRotor->setCollectivePitch(trimVector(3));

	vector<double> angularPositionEarthAxes = centerOfGravity->getAngularPositionEarthAxes();
	if(usePhi)
		angularPositionEarthAxes(0) = trimVector(4);
	else
		angularPositionEarthAxes(2) = trimVector(4);

	angularPositionEarthAxes(1) = trimVector(5);

	updatePhysics();
	centerOfGravity->updateDerivatives();

}

vector<double> Aircraft::buildAccelerationVector()
{
	vector<double> invertialVelocityDerivative = centerOfGravity->getInertialVelocityDerivativeCenterOfGravity();
	vector<double> invertialAngularVelocityDerivative = centerOfGravity->getInertialAngularVelocityDerivativeCenterOfGravity();

	vector<double> accelerationVector(6);

	for(int i = 0; i < 3; i++)
	{
		accelerationVector(i) = invertialVelocityDerivative(i);
		accelerationVector(i + 3) = invertialAngularVelocityDerivative(i);
	}
	return accelerationVector;

}

double Aircraft::jacobianTrim(vector<double> initialPositionEarthAxes, vector<double> initialAngularPositionEarthAxes, vector<double> initialVelocityEarthAxes, vector<double> initialAngularVelocityEarthAxes)
{
	centerOfGravity->setPositionEarthAxes(initialPositionEarthAxes);
	centerOfGravity->setAngularPositionEarthAxes(initialAngularPositionEarthAxes);
	centerOfGravity->setVelocityEarthAxes(initialVelocityEarthAxes);
	centerOfGravity->setAngularVelocityEarthAxes(initialAngularVelocityEarthAxes);

	vector<double> currentTrimVector = buildTrimVector();
	vector<double> perturbedTrimVector = currentTrimVector;
	applyTrimVector(perturbedTrimVector);

	vector<double> currentAccelerationVector = buildAccelerationVector();
	vector<double> perturbedAccelerationVector = currentAccelerationVector;
	std::cout << "acc : " << currentAccelerationVector << std::endl;

	double accelerationVectorNorm = Utils::vectorNorm(perturbedAccelerationVector);


	matrix<double> jacobianMatrix(6,6);
	matrix<double> accelerationMatrixMinusPerturbation(6,6);
	matrix<double> accelerationMatrixPlusPerturbation(6,6);

	int numberOfIterationLeft = 20;
	const double PERTURBATION_SIZE = 0.01;
	vector<double> trimGradient = Constants::ZERO_VEC_3;

	while (accelerationVectorNorm > 0.001 && numberOfIterationLeft > 0)
	{

		for(int jacobianRowIndex = 0; jacobianRowIndex < 6; jacobianRowIndex++)
		{
			for(int jacobianColumnIndex = 0; jacobianColumnIndex < 6; jacobianColumnIndex++)
			{
				perturbedTrimVector(jacobianColumnIndex) = currentTrimVector(jacobianColumnIndex) + PERTURBATION_SIZE;
				applyTrimVector(perturbedTrimVector);
				printForces();
				printWashes();
				perturbedAccelerationVector = buildAccelerationVector();
				accelerationMatrixMinusPerturbation(jacobianRowIndex, jacobianColumnIndex) = perturbedAccelerationVector(jacobianRowIndex);


				perturbedTrimVector(jacobianColumnIndex) = currentTrimVector(jacobianColumnIndex) - PERTURBATION_SIZE;
				applyTrimVector(perturbedTrimVector);
				printForces();
				printWashes();
				perturbedAccelerationVector = buildAccelerationVector();
				accelerationMatrixPlusPerturbation(jacobianRowIndex, jacobianColumnIndex) = perturbedAccelerationVector(jacobianRowIndex);


//				applyTrimVector(currentTrimVector);
				perturbedTrimVector = currentTrimVector;


			}
		}

		jacobianMatrix = (accelerationMatrixPlusPerturbation - accelerationMatrixMinusPerturbation) / (2 * PERTURBATION_SIZE);

		using namespace std;

//		cout << "jac : " << jacobianMatrix << endl;

		trimGradient = prod(Utils::invertMatrix(jacobianMatrix), currentAccelerationVector);

		currentTrimVector -= trimGradient;

		applyTrimVector(currentTrimVector);
		currentAccelerationVector = buildAccelerationVector();


		cout << "grad : " << trimGradient << endl;
		cout << "trim : " << perturbedTrimVector << endl;
		cout << "acc : " << currentAccelerationVector << endl;

		numberOfIterationLeft--;
		accelerationVectorNorm = Utils::vectorNorm(currentAccelerationVector);

		cout << "norm = " << accelerationVectorNorm << endl;

	}

	return accelerationVectorNorm;
}



