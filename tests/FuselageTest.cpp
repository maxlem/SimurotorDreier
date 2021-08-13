#include "cute.h"
#include "ide_listener.h"
#include "cute_runner.h"
#include "FuselageTest.h"
#include "Fuselage.h"
#include "Aircraft.h"
#include "AircraftFactory.h"
#include "Common.h"

void Fuselage_ComputeForcesAndMomentsAtIELA(){
	vector<double> distIELAfromRefFuselage = Utils::toVec3(10.0, 0.0, 5.0);
	vector<double> distIELAfromRefCog = Utils::toVec3(10.0, 0.0, 5.0);

	double vinf_kts = 5.0;
	double roc_fps = 0.0;
	CenterOfGravity * centerOfGravity = AircraftFactory::createCenterOfGravity(distIELAfromRefCog, Constants::ZERO_VEC_3, Constants::ZERO_VEC_3, Utils::toVec3(vinf_kts * 1.6878, 0.0, -roc_fps/60.0));
	centerOfGravity->updateVelocities();
	centerOfGravity->resetForces();

	centerOfGravity->setInertialVelocityIELA(Utils::toVec3(7.8, 8.9, 9.0));
	centerOfGravity->setInertialAngularVelocityIELA(Utils::toVec3(4.5, 5.6, 6.7));

	IndividualElement * fuselage = AircraftFactory::createFuselage();

	fuselage->setWashVelocityIELA(Utils::toVec3(1.2, 2.3, 3.4));
	fuselage->setWashAngularVelocityIELA(Utils::toVec3(9.8, 7.6, 4.3));
	fuselage->translateInertialVelocitiesToIELA(*centerOfGravity);
	fuselage->computeAerodynamicVelocities(Constants::RHO_SSL);
	fuselage->computeForcesAndMomentsAtIELA(*centerOfGravity, Constants::RHO_SSL);

	std::stringstream stringStream;
	stringStream << fuselage->getForcesIELA();

	ASSERTM(stringStream.str(), Utils::equals(Utils::toVec3(-1.4584942218000001, -4.4285467842000008, -1.8072177677399999), fuselage->getForcesIELA(), epsilon));

	stringStream << fuselage->getMomentsIELA();
	ASSERTM(stringStream.str(), Utils::equals(Utils::toVec3(0, 42.383403575934956, -13.37192214834168), fuselage->getMomentsIELA(), epsilon));

}

cute::suite make_suite_FuselageTest(){
	cute::suite s;
	s.push_back(CUTE(Fuselage_ComputeForcesAndMomentsAtIELA));
	return s;
}



