#include "cute.h"
#include "ide_listener.h"
#include "cute_runner.h"
#include "HalfWingTest.h"
#include "AircraftFactory.h"
#include "Common.h"

void HalfWing_computeForcesAndMomentsAtIELA() {


	vector<double> distIELAfromRefCog = Utils::toVec3(10.0, 0.0, 5.0);

	double vinf_kts = 5.0;
	double roc_fps = 0.0;

	CenterOfGravity * centerOfGravity = AircraftFactory::createCenterOfGravity(distIELAfromRefCog, Constants::ZERO_VEC_3, Constants::ZERO_VEC_3, Utils::toVec3(vinf_kts * 1.6878, 0.0, -roc_fps/60.0));
	centerOfGravity->resetForces();
	centerOfGravity->updateVelocities();

	centerOfGravity->setInertialVelocityIELA(Utils::toVec3(7.8, 8.9, 9.0));
	centerOfGravity->setInertialAngularVelocityIELA(Utils::toVec3(4.5, 5.6, 6.7));

	IndividualElement * verticalStab = AircraftFactory::createVerticalStabilizer();

	verticalStab->setRotationAnglesIERAFromIELA(Utils::toVec3(90.0/57.295, 0.0, 0.0));
	verticalStab->setWashVelocityIELA(Utils::toVec3(1.2, 2.3, 3.4));

	verticalStab->translateInertialVelocitiesToIELA(*centerOfGravity);
	verticalStab->computeAerodynamicVelocities(Constants::RHO_SSL);
	verticalStab->computeForcesAndMomentsAtIELA(*centerOfGravity, Constants::RHO_SSL);

	std::cout << "fin " << verticalStab->getForcesIELA() << std::endl;

	std::stringstream stringStream;
	stringStream << verticalStab->getForcesIELA();
	ASSERTM(stringStream.str(), Utils::equals(Utils::toVec3(2734.77498135105, -124.091934352812, -0.00265197921514406), verticalStab->getForcesIELA(), epsilon));

	stringStream << verticalStab->getMomentsIELA();
	ASSERTM(stringStream.str(), Utils::equals(Utils::toVec3(227.915519480046, 5022.8700479344, 0.107344180239945), verticalStab->getMomentsIELA(), epsilon));




}

cute::suite make_suite_HalfWingTest(){
	cute::suite s;
	s.push_back(CUTE(HalfWing_computeForcesAndMomentsAtIELA));
	return s;
}



