#include "cute.h"
#include "ide_listener.h"
#include "cute_runner.h"
#include "RotorTest.h"
#include "Common.h"
#include "Rotor.h"

void Rotor_computeAerodynamicVelocities() {

	vector<double> distIELAfromRefCog = Utils::toVec3(10.0, 0.0, 5.0);

	double vinf_kts = 5.0;
	double roc_fps = 0.0;

	CenterOfGravity * centerOfGravity = AircraftFactory::createCenterOfGravity(distIELAfromRefCog, Constants::ZERO_VEC_3, Constants::ZERO_VEC_3, Utils::toVec3(vinf_kts * 1.6878, 0.0, -roc_fps/60.0));
	centerOfGravity->resetForces();
	centerOfGravity->updateVelocities();

	Rotor * mainRotor = AircraftFactory::createMainRotor();

	mainRotor->translateInertialVelocitiesToIELA(*centerOfGravity);
	mainRotor->computeAerodynamicVelocities(Constants::RHO_SSL);

	ASSERT(Utils::equals(Utils::toVec3(8.439, 0, -20.0), mainRotor->getAerodynamicVelocityIELA(), epsilon));
	ASSERT(Utils::equals(Constants::ZERO_VEC_3, mainRotor->getAerodynamicAngularVelocityIELA(), epsilon));

	Rotor * tailRotor = AircraftFactory::createTailRotor();

	tailRotor->translateInertialVelocitiesToIELA(*centerOfGravity);
	tailRotor->computeAerodynamicVelocities(Constants::RHO_SSL);

	ASSERT(Utils::equals(Utils::toVec3(8.439, 20.0, 0.0), tailRotor->getAerodynamicVelocityIELA(), epsilon));
	ASSERT(Utils::equals(Constants::ZERO_VEC_3, tailRotor->getAerodynamicAngularVelocityIELA(), epsilon));
}

void Rotor_computeForcesAndMomentsAtIELA() {

	vector<double> distIELAfromRefCog = Utils::toVec3(10.0, 0.0, 5.0);

	double vinf_kts = 5.0;
	double roc_fps = 0.0;

	CenterOfGravity * centerOfGravity = AircraftFactory::createCenterOfGravity(distIELAfromRefCog, Constants::ZERO_VEC_3, Constants::ZERO_VEC_3, Utils::toVec3(vinf_kts * 1.6878, 0.0, -roc_fps/60.0));
	centerOfGravity->resetForces();
	centerOfGravity->updateVelocities();

	Rotor * mainRotor = AircraftFactory::createMainRotor();

	mainRotor->translateInertialVelocitiesToIELA(*centerOfGravity);
	mainRotor->computeAerodynamicVelocities(Constants::RHO_SSL);
	mainRotor->computeForcesAndMomentsAtIELA(*centerOfGravity, Constants::RHO_SSL);


	std::stringstream stringStream;
	stringStream << mainRotor->getForcesIELA();
	ASSERTM(stringStream.str(), Utils::equals(Utils::toVec3(253.57361653093861, 1.4049707985473521, -5230.6627916676116), mainRotor->getForcesIELA(), epsilon)); //TODO make sure it is just double vs single error

	stringStream << mainRotor->getMomentsIELA();
	ASSERTM(stringStream.str(), Utils::equals(Utils::toVec3(-441.80421848682914, 0, 8439.7958723766205), mainRotor->getMomentsIELA(), epsilon));

	Rotor * tailRotor = AircraftFactory::createTailRotor(45.0/57.295);

	tailRotor->setRotationAnglesIERAFromIELA(Utils::toVec3(-1.57078, 0, 0));
	tailRotor->setCollectivePitch(5.0/57.295);
	tailRotor->translateInertialVelocitiesToIELA(*centerOfGravity);
	tailRotor->computeAerodynamicVelocities(Constants::RHO_SSL);
	tailRotor->computeForcesAndMomentsAtIELA(*centerOfGravity, Constants::RHO_SSL);


	std::stringstream stringStream2;
	stringStream2 << tailRotor->getForcesIELA();
	ASSERTM(stringStream2.str(), Utils::equals(Utils::toVec3(-0.6238061238020467, -231.52289349362104, 0.060640369986853973), tailRotor->getForcesIELA(), epsilon)); //TODO make sure it is just double vs single error

	stringStream << tailRotor->getMomentsIELA();
	ASSERTM(stringStream2.str(), Utils::equals(Utils::toVec3(0, 69.649809328259877, 0.0011371581515876444), tailRotor->getMomentsIELA(), epsilon));

}
void Rotor_computeExternalWashInfluence()
{


	IndividualElement * fuselage = AircraftFactory::createFuselage();

	Rotor * mainRotor = AircraftFactory::createMainRotor();
	mainRotor->setInertialVelocityIELA(Utils::toVec3(1.2, 2.3, 4.5));
	mainRotor->setWashVelocityIELA(Utils::toVec3(4.5, 6.7, 4.3));

	mainRotor->computeAerodynamicVelocities(Constants::RHO_SSL);
	mainRotor->setForcesIELA(Utils::toVec3(443.9, 2.7, 544.93));
	mainRotor->setMomentsIELA(Utils::toVec3(44.5, 436.7, 455.3));

	vector<double> externalWashVelocity = mainRotor->computeExternalWashInfluence(fuselage, Constants::RHO_SSL);

	ASSERT(Utils::equals(Utils::toVec3(0, 0, -6.8446750095422297), externalWashVelocity, epsilon));

}
cute::suite make_suite_RotorTest(){
	cute::suite s;
	s.push_back(CUTE(Rotor_computeAerodynamicVelocities));
	s.push_back(CUTE(Rotor_computeForcesAndMomentsAtIELA));
	s.push_back(CUTE(Rotor_computeExternalWashInfluence));
	return s;
}



