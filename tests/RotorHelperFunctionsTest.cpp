#include "cute.h"
#include "ide_listener.h"
#include "cute_runner.h"
#include "RotorHelperFunctionsTest.h"
#include "AircraftFactory.h"
#include "Rotor.h"
#include "Common.h"



Rotor * mainRotor;
CenterOfGravity * centerOfGravity;

double vinf_kts;
double roc_fps;


double tipLossFactor;
double epsilon0;
/*! \f$ \overline{p_{a}} \f$  Dreier : pahat */
double normalizedAerodynamicRollVelocity;

/*! \f$ \overline{q_{a}} \f$  Dreier : pahat */
double normalizedAerodynamicPitchVelocity;

/*! \f$ \overline{p_{i}} \f$  Dreier : pihat */
double normalizedInertialRollVelocity;

/*! \f$ \overline{q_{i}} \f$  Dreier : qihat */
double normalizedInertialPitchVelocity;


double normalizedNaturalFlappingFrequency;
double normalizedRotorVelocity;
double inertialRotorVelocity;


/*! \f$ \left(\theta_{collective}, \theta_{swash_{side}}, \theta_{swash_{front}}\right) \f$  Dreier : (theta0, ca1, cb1)*/
double swashplateAnglesArray[4] = {0.611, 0.777, 0.888, 0.999};
vector<double> swashplateAngles;

/*! \f$ \left(\theta_{flap_{cone}}, \theta_{swash_{front}}, \theta_{swash_{side}}\right) \f$  Dreier : (beta0, a1, b1)*/
double flappingAnglesArray[3] = {0.911, 0.822, 0.733};
vector<double> flappingAngles;

double advanceRatio;
double inertialInflowRatio;
double aerodynamicInflowRatio;

double lockNumber;

vector<double>  inertialVelocityDerivative;
vector<double>  inertialAngularVelocityDerivative;


vector<double> accelerationVector;


void setUp()
{
vector<double> distIELAfromRefCog = Utils::toVec3(10.0, 0.0, 5.0);

vinf_kts = 5.0;
roc_fps = 0.0;

centerOfGravity = AircraftFactory::createCenterOfGravity(distIELAfromRefCog, Constants::ZERO_VEC_3, Constants::ZERO_VEC_3, Utils::toVec3(vinf_kts * 1.6878, 0.0, -roc_fps/60.0));
centerOfGravity->resetForces();
centerOfGravity->updateVelocities();

mainRotor = AircraftFactory::createMainRotor();

mainRotor->translateInertialVelocitiesToIELA(*centerOfGravity);
mainRotor->computeAerodynamicVelocities(Constants::RHO_SSL);

/*! \f$ \overline{p_{a}} \f$  Dreier : pahat */
normalizedAerodynamicRollVelocity = 0.111;
/*! \f$ \overline{q_{a}} \f$  Dreier : qahat */
normalizedAerodynamicPitchVelocity = 0.222;

normalizedInertialRollVelocity = 0.342;

/*! \f$ \overline{q_{i}} \f$  Dreier : qihat */
normalizedInertialPitchVelocity = 0.876;




/*! Dreier : fom*/
normalizedRotorVelocity = 0.654;

inertialRotorVelocity = 0.369;

tipLossFactor = 0.333;
epsilon0 = 0.444;
advanceRatio = 0.555;
aerodynamicInflowRatio = 0.666;
inertialInflowRatio = 0.191;
lockNumber = 0.432;
normalizedNaturalFlappingFrequency = 0.765;

double radius = 22.0; //mainRotor->radius;

inertialVelocityDerivative = Utils::toVec3(0.192, 0.283, 0.374);
inertialAngularVelocityDerivative = Utils::toVec3(0.352, 0.583, 0.999);

double accelerationArray[] = /*TODO*/ {normalizedInertialRollVelocity, normalizedInertialPitchVelocity, inertialAngularVelocityDerivative(0) / pow(inertialRotorVelocity, 2.0), inertialAngularVelocityDerivative(1) / pow(inertialRotorVelocity, 2.0), inertialVelocityDerivative(0) / (radius * pow(inertialRotorVelocity, 2.0)), inertialVelocityDerivative(1) / (radius * pow(inertialRotorVelocity, 2.0)), inertialVelocityDerivative(2) / (radius * pow(inertialRotorVelocity, 2.0))};
accelerationVector = Utils::toVector(accelerationArray, 7);


/*! \f$ \left(\theta_{collective}, \theta_{twist}, \theta_{swash_{side}}, \theta_{swash_{front}}\right) \f$  Dreier : (theta0, twist, ca1, cb1)*/
swashplateAngles = Utils::toVector(swashplateAnglesArray, 4);

/*! \f$ \left(\theta_{flap_{cone}}, \theta_{swash_{front}}, \theta_{swash_{side}}\right) \f$  Dreier : (beta0, a1, b1)*/
flappingAngles= Utils::toVector(flappingAnglesArray, 3);


}

void tearDown()
{
	delete mainRotor;
	delete centerOfGravity;
}



void RotorHelperFunctions_computeXForceFactor()
{
	setUp();

	double xForceFactor =  mainRotor->computeXForceFactor(advanceRatio, aerodynamicInflowRatio, tipLossFactor, epsilon0, swashplateAngles, flappingAngles, normalizedAerodynamicRollVelocity, normalizedAerodynamicPitchVelocity);

	ASSERT(Utils::equals(-0.00394178042, xForceFactor, pow(epsilon, 2.0)));
	tearDown();

}
void RotorHelperFunctions_computeYForceFactor()
{
	setUp();
	double yForceFactor =  mainRotor->computeYForceFactor(advanceRatio, aerodynamicInflowRatio, tipLossFactor, epsilon0, swashplateAngles, flappingAngles, normalizedAerodynamicRollVelocity, normalizedAerodynamicPitchVelocity);

	ASSERT(Utils::equals(-0.032478, yForceFactor, pow(epsilon, 2.0)));
	tearDown();
}

void RotorHelperFunctions_computeZForceFactor()
{

	setUp();
	double zForceFactor =  mainRotor->computeZForceFactor(advanceRatio, aerodynamicInflowRatio, tipLossFactor, epsilon0, swashplateAngles, normalizedAerodynamicRollVelocity);

	ASSERT(Utils::equals(-0.0428686365, zForceFactor, pow(epsilon, 2.0)));
	tearDown();
}

void RotorHelperFunctions_computeXMomentFactor()
{
	setUp();
	double xMomentFactor = mainRotor->computeXMomentFactor(advanceRatio, aerodynamicInflowRatio, tipLossFactor, epsilon0, swashplateAngles, flappingAngles, normalizedAerodynamicRollVelocity);

	ASSERT(Utils::equals(-0.0038718467699, xMomentFactor, pow(epsilon, 2.0)));
	// TODO : warning this has been only code reviewed
	tearDown();
}
void RotorHelperFunctions_computeYMomentFactor()
{
	setUp();
	double yMomentFactor = mainRotor->computeYMomentFactor(advanceRatio, aerodynamicInflowRatio, tipLossFactor, epsilon0, swashplateAngles, flappingAngles, normalizedAerodynamicRollVelocity);

	ASSERT(Utils::equals(0.0000329422048, yMomentFactor, pow(epsilon, 2.0)));
	// TODO : warning this has been only code reviewed
	tearDown();
}
void RotorHelperFunctions_computeZFlappingMomentFactor()
{
	setUp();
	double zFlappingMomentFactor =  mainRotor->computeZFlappingMomentFactor(advanceRatio, aerodynamicInflowRatio, tipLossFactor, epsilon0, swashplateAngles, flappingAngles);

	ASSERT(Utils::equals(-0.009839041, zFlappingMomentFactor, pow(epsilon, 2.0)));
	tearDown();
}
void RotorHelperFunctions_computeZSwashplateMomentFactor()
{
	setUp();
	double zSwashplateMomentFactor =  mainRotor->computeZSwashplateMomentFactor(advanceRatio, aerodynamicInflowRatio, tipLossFactor, swashplateAngles, flappingAngles);

	ASSERT(Utils::equals(-0.001931483, zSwashplateMomentFactor, pow(epsilon, 2.0)));
	tearDown();
}
void RotorHelperFunctions_computeZRollPitchMomentFactor()
{
	setUp();
	double zRollPitchMomentFactor =  mainRotor->computeZRollPitchMomentFactor(advanceRatio, aerodynamicInflowRatio, tipLossFactor, swashplateAngles, flappingAngles, normalizedAerodynamicRollVelocity, normalizedAerodynamicPitchVelocity);

	ASSERT(Utils::equals(0.00005380936, zRollPitchMomentFactor, pow(epsilon, 2.0)));
	tearDown();
}
void RotorHelperFunctions_computeZMomentFactor()
{
	setUp();

	double zMomentFactor = mainRotor->computeZMomentFactor(advanceRatio, aerodynamicInflowRatio, tipLossFactor, epsilon0, swashplateAngles, flappingAngles, normalizedAerodynamicRollVelocity, normalizedAerodynamicPitchVelocity);

	ASSERT(Utils::equals(-0.0117167151, zMomentFactor, pow(epsilon, 2.0)));
	tearDown();
}

void RotorHelperFunctions_solveImplicitThrustWashRelation()
{
	setUp();

	aerodynamicInflowRatio =  mainRotor->solveImplicitThrustWashRelation(tipLossFactor, epsilon0, normalizedAerodynamicRollVelocity, swashplateAngles, advanceRatio, inertialInflowRatio, aerodynamicInflowRatio);

	ASSERT(Utils::equals(0.0712561, aerodynamicInflowRatio, epsilon));
	ASSERT(Utils::equals(0.3544775, epsilon0, epsilon));
	ASSERT(Utils::equals(0.9, tipLossFactor, epsilon));
	tearDown();
}

void RotorHelperFunctions_solveFlappingEquation()
{
	setUp();

	flappingAngles = mainRotor->solveFlappingEquation(lockNumber, tipLossFactor, epsilon0, advanceRatio, aerodynamicInflowRatio, normalizedNaturalFlappingFrequency, normalizedRotorVelocity, normalizedAerodynamicRollVelocity, normalizedAerodynamicPitchVelocity, flappingAngles, swashplateAngles, accelerationVector);

	std::stringstream stringStream;
	stringStream << flappingAngles;

	ASSERTM(stringStream.str(), Utils::equals(Utils::toVec3(0.340701729,12.1890278, 1.7039094), flappingAngles, epsilon));

	tearDown();
}

cute::suite make_suite_RotorHelperFunctionsTest(){
	cute::suite s;

	s.push_back(CUTE(RotorHelperFunctions_computeXForceFactor));
	s.push_back(CUTE(RotorHelperFunctions_computeYForceFactor));
	s.push_back(CUTE(RotorHelperFunctions_computeZForceFactor));
	s.push_back(CUTE(RotorHelperFunctions_computeXMomentFactor));
	s.push_back(CUTE(RotorHelperFunctions_computeYMomentFactor));
	s.push_back(CUTE(RotorHelperFunctions_computeZFlappingMomentFactor));
	s.push_back(CUTE(RotorHelperFunctions_computeZSwashplateMomentFactor));
	s.push_back(CUTE(RotorHelperFunctions_computeZRollPitchMomentFactor));
	s.push_back(CUTE(RotorHelperFunctions_computeZMomentFactor));

	s.push_back(CUTE(RotorHelperFunctions_solveImplicitThrustWashRelation));
	s.push_back(CUTE(RotorHelperFunctions_solveFlappingEquation));

	return s;
}



