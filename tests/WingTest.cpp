#include "cute.h"
#include "ide_listener.h"
#include "cute_runner.h"
#include "WingTest.h"
#include "Common.h"
#include "AircraftFactory.h"

void Wing_translateInertialVelocitiesToIELA() {

	double radius = 2.0;

	vector<double> distIELAfromRefIndividualElement = Utils::toVec3(26.9, 0, 5.25);
	vector<double> distIELAfromRefCog = Utils::toVec3(10.0, 0.0, 5.0);

	double vinf_kts = 5.0;
	double roc_fps = 0.0;

	CenterOfGravity * centerOfGravity = AircraftFactory::createCenterOfGravity(distIELAfromRefCog, Constants::ZERO_VEC_3, Constants::ZERO_VEC_3, Utils::toVec3(vinf_kts * 1.6878, 0.0, -roc_fps/60.0));
	centerOfGravity->resetForces();centerOfGravity->updateVelocities();;

	IndividualElement * horizStab = AircraftFactory::createHorizontalStabilizer();

	horizStab->translateInertialVelocitiesToIELA(*centerOfGravity);

	ASSERT(Utils::equals(Utils::toVec3(8.439, 0.0, 0.0), horizStab->getInertialVelocityIELA(), epsilon));

}

void Wing_computeAerodynamicVelocities() {

	IndividualElement * horizStab = AircraftFactory::createHorizontalStabilizer();


	horizStab->setInertialVelocityIELA(4.0 * Constants::UNIT_VEC_3);
	horizStab->setInertialAngularVelocityIELA(1.5 * Constants::UNIT_VEC_3);

	horizStab->setWashVelocityIELA(2.0 * Constants::UNIT_VEC_3);
	horizStab->setWashAngularVelocityIELA(0.5 * Constants::UNIT_VEC_3);

	horizStab->computeAerodynamicVelocities(Constants::RHO_SSL);

	ASSERT(Utils::equals(horizStab->getAerodynamicVelocityIELA(), 2.0 * Constants::UNIT_VEC_3, epsilon));
	ASSERT(Utils::equals(horizStab->getAerodynamicAngularVelocityIELA(), 1.0 * Constants::UNIT_VEC_3, epsilon));

}


void Wing_computeForcesAndMomentsAtIELA() {


	vector<double> distIELAfromRefCog = Utils::toVec3(10.0, 0.0, 5.0);

	double vinf_kts = 5.0;
	double roc_fps = 0.0;

	CenterOfGravity * centerOfGravity = AircraftFactory::createCenterOfGravity(distIELAfromRefCog, Constants::ZERO_VEC_3, Constants::ZERO_VEC_3, Utils::toVec3(vinf_kts * 1.6878, 0.0, -roc_fps/60.0));
	centerOfGravity->resetForces();
	centerOfGravity->updateVelocities();

	centerOfGravity->setInertialVelocityIELA(Utils::toVec3(7.8, 8.9, 9.0));
	centerOfGravity->setInertialAngularVelocityIELA(Utils::toVec3(4.5, 5.6, 6.7));

	IndividualElement * horizStab = AircraftFactory::createHorizontalStabilizer();
	horizStab->setWashVelocityIELA(Utils::toVec3(1.2, 2.3, 3.4));

	horizStab->translateInertialVelocitiesToIELA(*centerOfGravity);
	horizStab->computeAerodynamicVelocities(Constants::RHO_SSL);
	horizStab->computeForcesAndMomentsAtIELA(*centerOfGravity, Constants::RHO_SSL);


	std::stringstream stringStream;
	stringStream << horizStab->getForcesIELA();
	ASSERTM(stringStream.str(), Utils::equals(Utils::toVec3(2796.27802293631, 0, -121.854094689264), horizStab->getForcesIELA(), epsilon));

	stringStream << horizStab->getMomentsIELA();
	ASSERTM(stringStream.str(), Utils::equals(Utils::toVec3(1050.59749914387, 0, -966.90842459447), horizStab->getMomentsIELA(), epsilon));
}

void Wing_sumForcesAndMomentsToCenterOfGravity(){

	vector<double> distIELAfromRefCog = Utils::toVec3(10.0, 0.0, 5.0);

	double vinf_kts = 5.0;
	double roc_fps = 0.0;

	CenterOfGravity * centerOfGravity = AircraftFactory::createCenterOfGravity(distIELAfromRefCog, Constants::ZERO_VEC_3, Constants::ZERO_VEC_3, Utils::toVec3(vinf_kts * 1.6878, 0.0, -roc_fps/60.0));
	centerOfGravity->resetForces();centerOfGravity->updateVelocities();;

	IndividualElement * horizStab = AircraftFactory::createHorizontalStabilizer();


	horizStab->setForcesIELA(Utils::toVec3(-0.02107019, 0.0, 0.0));
	horizStab->setMomentsIELA(Utils::toVec3(0.0, 0.0, 0.0));
	horizStab->sumForcesAndMomentsToCenterOfGravity(*centerOfGravity);


	std::stringstream stringStream;
	stringStream << centerOfGravity->getForcesIELA();

	ASSERTM(stringStream.str(), Utils::equals(Utils::toVec3(-0.02107019, 0.0, 0.0), centerOfGravity->getForcesIELA(), epsilon));

	stringStream.flush();
	stringStream << centerOfGravity->getMomentsIELA();

	ASSERTM(stringStream.str(), Utils::equals( Utils::toVec3(0.0, 0.00526754744, 0.0) , centerOfGravity->getMomentsIELA(), epsilon));
}

cute::suite make_suite_WingTest(){
	cute::suite s;
	s.push_back(CUTE(Wing_translateInertialVelocitiesToIELA));
	s.push_back(CUTE(Wing_computeAerodynamicVelocities));
	s.push_back(CUTE(Wing_computeForcesAndMomentsAtIELA));
	s.push_back(CUTE(Wing_sumForcesAndMomentsToCenterOfGravity));
	return s;
}



