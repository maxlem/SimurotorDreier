#include "cute.h"
#include "ide_listener.h"
#include "cute_runner.h"
#include "IndividualElementTest.h"

#include "Common.h"


void IndividualElement_computeSelfInducedWashVelocity(){

	double radius = 2.0;

	IndividualElement individualElement = createIndividualElement(radius);

	vector<double> res = individualElement.computeSelfInducedWashVelocity(Constants::RHO_SSL);
	ASSERT(Utils::equals(Constants::ZERO_VEC_3, res, epsilon));

	res = individualElement.computeSelfInducedWashVelocity(0.0);
	ASSERT(Utils::equals(Constants::ZERO_VEC_3, res, epsilon));

	individualElement.setInertialVelocityIELA(4.0 * Constants::UNIT_VEC_3);
	individualElement.setWashVelocityIELA(2.0 * Constants::UNIT_VEC_3);

	individualElement.computeAerodynamicVelocities(Constants::RHO_SSL);
	individualElement.setForcesIELA(2.5 * Constants::UNIT_VEC_3);

	res = individualElement.computeSelfInducedWashVelocity(Constants::RHO_SSL);
	//(-1)/(2*0.002378*root((2^2+2^2+2^2), 2))*1/pi*2^2*2.5 = -12.0753

	std::stringstream stringStream;
	stringStream << res;

	ASSERTM(stringStream.str(), Utils::equals(-12.0753 * Constants::UNIT_VEC_3, res, epsilon));

}


void IndividualElement_translateInertialVelocitiesToIELA(){

	double radius = 2.0;

	vector<double> distIELAfromRefIndividualElement = Utils::toVec3(26.9, 0, 5.25);
	vector<double> distIELAfromRefCog = Utils::toVec3(10.0, 0.0, 5.0);

	double vinf_kts = 5.0;
	double roc_fps = 0.0;
	CenterOfGravity centerOfGravity = createCenterOfGravity(distIELAfromRefCog, Constants::ZERO_VEC_3, Constants::ZERO_VEC_3, Utils::toVec3(vinf_kts * 1.6878, 0.0, -roc_fps/60.0));

	centerOfGravity.updateVelocities();
	centerOfGravity.resetForces();

	IndividualElement individualElement = createIndividualElement(radius, distIELAfromRefIndividualElement);

	individualElement.translateInertialVelocitiesToIELA(centerOfGravity);

	ASSERT(Utils::equals(Utils::toVec3(8.439, 0.0, 0.0), individualElement.getInertialVelocityIELA(), epsilon));

}




void IndividualElement_computeAerodynamicVelocities(){

	IndividualElement individualElement = createIndividualElement();


	individualElement.setInertialVelocityIELA(4.0 * Constants::UNIT_VEC_3);
	individualElement.setInertialAngularVelocityIELA(1.5 * Constants::UNIT_VEC_3);

	individualElement.setWashVelocityIELA(2.0 * Constants::UNIT_VEC_3);
	individualElement.setWashAngularVelocityIELA(0.5 * Constants::UNIT_VEC_3);

	individualElement.computeAerodynamicVelocities(Constants::RHO_SSL);

	ASSERT(Utils::equals(individualElement.getAerodynamicVelocityIELA(), 2.0 * Constants::UNIT_VEC_3, epsilon));
	ASSERT(Utils::equals(individualElement.getAerodynamicAngularVelocityIELA(), 1.0 * Constants::UNIT_VEC_3, epsilon));

}

void IndividualElement_computeForcesAndMomentsAtIELA(){

	CenterOfGravity centerOfGravity = createCenterOfGravity();
	centerOfGravity.updateVelocities();
	centerOfGravity.resetForces();
	IndividualElement individualElement = createIndividualElement();

	vector<double> forcesIELABefore = individualElement.getForcesIELA();
	vector<double> momentsIELABefore = individualElement.getMomentsIELA();


	individualElement.computeForcesAndMomentsAtIELA(centerOfGravity, Constants::RHO_SSL);

	//shall have no effect

	ASSERT(Utils::equals(forcesIELABefore, individualElement.getForcesIELA(), epsilon));
	ASSERT(Utils::equals(momentsIELABefore, individualElement.getMomentsIELA(), epsilon));
}

void IndividualElement_sumForcesAndMomentsToCenterOfGravity(){

	CenterOfGravity centerOfGravity = createCenterOfGravity();
	centerOfGravity.updateVelocities();
	centerOfGravity.resetForces();
	IndividualElement individualElement = createIndividualElement();

	vector<double> forcesCoGBefore = centerOfGravity.getForcesIELA();
	vector<double> momentsCoGBefore = centerOfGravity.getMomentsIELA();


	individualElement.setForcesIELA(2.0 * Constants::UNIT_VEC_3);
	individualElement.setMomentsIELA(3.0 * Constants::UNIT_VEC_3);

	individualElement.sumForcesAndMomentsToCenterOfGravity(centerOfGravity);

	vector<double> expectedForcesCoG = forcesCoGBefore + 2.0 * Constants::UNIT_VEC_3;



	std::stringstream stringStream;
	stringStream << centerOfGravity.getForcesIELA();

	ASSERTM(stringStream.str(), Utils::equals(expectedForcesCoG, centerOfGravity.getForcesIELA(), epsilon));

	vector<double> expectedMomentsCoG = momentsCoGBefore + 3.0 * Constants::UNIT_VEC_3 + Utils::crossProduct(Constants::ZERO_VEC_3, expectedForcesCoG);

	stringStream.flush();
	stringStream << centerOfGravity.getMomentsIELA();

	ASSERTM(stringStream.str(), Utils::equals(expectedMomentsCoG, centerOfGravity.getMomentsIELA(), epsilon));
}



cute::suite make_suite_IndividualElementTest(){

	cute::suite s;
	s.push_back(CUTE(IndividualElement_computeSelfInducedWashVelocity));
	s.push_back(CUTE(IndividualElement_translateInertialVelocitiesToIELA));
	s.push_back(CUTE(IndividualElement_computeAerodynamicVelocities));
	s.push_back(CUTE(IndividualElement_computeForcesAndMomentsAtIELA));
	s.push_back(CUTE(IndividualElement_sumForcesAndMomentsToCenterOfGravity));
	return s;
}
