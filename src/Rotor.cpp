/*
 * Rotor.cpp
 *
 *  Created on: 14-Jul-2008
 *      Author: malem303
 */

#include <boost/numeric/ublas/io.hpp>

#include "Rotor.h"
#include "Utils.h"
#include "CenterOfGravity.h"


Rotor::Rotor(int numberOfBlades, double radius, double chord, double linearMass, double delta3Angle, HubType hubType, double preconeAngle, double liftCurveSlope, vector<double> dragCoefficientParameters, double horizontalHubOffset, double verticalHubOffset, double hubSpringConstant, ControlRigging collectivePitchRigging, ControlRigging lateralSwashplateRigging, ControlRigging longitudinalSwashplateRigging, double twistAngle, matrix<double> selfInducedWashVelocityModel):
	IndividualElement(selfInducedWashVelocityModel), numberOfBlades(numberOfBlades), radius(radius), chord(chord), linearMass(linearMass),
	delta3Angle(delta3Angle), hubType(hubType), preconeAngle(preconeAngle), liftCurveSlope(liftCurveSlope),
	dragCoefficientParameters(dragCoefficientParameters), horizontalHubOffset(horizontalHubOffset), verticalHubOffset(verticalHubOffset),
	hubSpringConstant(hubSpringConstant), collectivePitch(collectivePitchRigging), twistAngle(twistAngle),
	lateralSwashplateAngle(lateralSwashplateRigging), longitudinalSwashplateAngle(longitudinalSwashplateRigging),
	coningAngle(0.0), longitudinalFlappingAngle(0.0), lateralFlappingAngle(0.0)
{
	// TODO Auto-generated constructor stub

}

Rotor::~Rotor() {
	// TODO Auto-generated destructor stub
}


void Rotor::computeAerodynamicVelocities(double density)
{
	// the rotor is a special case, since it closes the thrust-induced velocity loop
	// so we only want to consider the self induced wash velocity
	vector<double> externalWashVelocity = washVelocityIELA - computeSelfInducedWashVelocity(density);

	// then we substract externalWashVelocity from inertialVelocityIELA
	inertialVelocityIELA -= externalWashVelocity;

	// then we proceed as usual :
	IndividualElement::computeAerodynamicVelocities(density);
}

double Rotor::computeInfluenceFactor(IndividualElement * influencedElement)
{
	vector<double> distanceRotorFromElement = getDistanceIELAFromReferenceAxes() - influencedElement->getDistanceIELAFromReferenceAxes();

	double distanceRadiusRatio = Utils::vectorNorm(distanceRotorFromElement) / radius;
	return (1 + (distanceRadiusRatio / sqrt(1 + pow(distanceRadiusRatio, 2.0))));
}

vector<double> Rotor::computeExternalWashInfluence(IndividualElement * influencedElement, double density)
{
	vector<double> selfInducedWashVelocity = computeSelfInducedWashVelocity(density);

	return Utils::toVec3(0.0, 0.0, computeInfluenceFactor(influencedElement) * selfInducedWashVelocity(2));

}
double Rotor::computeTipLossFactor(double thrustForceFactor) const
{
	double tipLossFactor = 1 - sqrt(2 * fabs(thrustForceFactor))/(float)numberOfBlades; // TODO confirm it is normal thrustForceFactor can be negative


	return (tipLossFactor < 0.9) ? 0.9 : tipLossFactor;
}
double Rotor::computeNormalizedLiftCoefficient(double thrustForceFactor) const
{
	// Dreier p.445
	return 6.0 * thrustForceFactor / computeSolidity();
}

double Rotor::computeNormalizedDragCoefficient(double thrustForceFactor) const
{
	double normalizedLiftCoefficient = computeNormalizedLiftCoefficient(thrustForceFactor);
	return dragCoefficientParameters(0) + normalizedLiftCoefficient * (dragCoefficientParameters(1) + normalizedLiftCoefficient*dragCoefficientParameters(2));
}

double Rotor::computeXForceFactor(double advanceRatio, double aerodynamicInflowRatio, double tipLossFactor, double epsilon0, vector<double> swashplateAngles, vector<double> flappingAngles, double normalizedAerodynamicRollVelocity, double normalizedAerodynamicPitchVelocity) const
{
	return (computeSolidity() * liftCurveSlope / 2.0) *
				(
						-(pow(tipLossFactor, 2.0) * advanceRatio / 2.0) * epsilon0
						+((tipLossFactor * advanceRatio / 2.0) * aerodynamicInflowRatio  - (pow(tipLossFactor, 3.0) / 3) * flappingAngles(1)) * swashplateAngles(0)
						+((pow(tipLossFactor, 2.0) * advanceRatio / 4.0) * aerodynamicInflowRatio - (pow(tipLossFactor, 4.0) / 4.0) * flappingAngles(1)) * swashplateAngles(1)
						+((pow(tipLossFactor, 3.0)/ 6.0) * flappingAngles(0)) * (swashplateAngles(2) + flappingAngles(2))
						+((pow(tipLossFactor, 2.0) / 4.0) * aerodynamicInflowRatio - (pow(tipLossFactor, 2.0) * advanceRatio / 4.0) * flappingAngles(1)) * swashplateAngles(3)
						-(3.0 * pow(tipLossFactor, 2.0) / 4.0) * aerodynamicInflowRatio * flappingAngles(1)
						-(pow(tipLossFactor, 2.0) * advanceRatio / 4.0) * (pow(flappingAngles(0), 2.0) + pow(flappingAngles(1), 2.0))
						+((pow(tipLossFactor, 3.0) / 6.0) * swashplateAngles(0) + (pow(tipLossFactor, 4.0) / 8.0) * swashplateAngles(1) + (pow(tipLossFactor, 2.0) / 2.0) * aerodynamicInflowRatio + (3 * pow(tipLossFactor, 2.0) * advanceRatio / 16.0) * swashplateAngles(3) + (pow(tipLossFactor, 2.0) * advanceRatio / 16.0) * flappingAngles(1) ) * normalizedAerodynamicRollVelocity
						+((pow(tipLossFactor, 3.0) / 6.0) * flappingAngles(0) + (pow(tipLossFactor, 2.0) * advanceRatio / 16.0) * swashplateAngles(2) + (pow(tipLossFactor, 2.0) * advanceRatio / 16.0) * flappingAngles(2) ) * normalizedAerodynamicPitchVelocity
				);
}

double Rotor::computeYForceFactor(double advanceRatio, double aerodynamicInflowRatio, double tipLossFactor, double epsilon0, vector<double> swashplateAngles, vector<double> flappingAngles, double normalizedAerodynamicRollVelocity, double normalizedAerodynamicPitchVelocity) const
{
	return (computeSolidity() * liftCurveSlope / 2.0) *
				(
						((pow(tipLossFactor, 3.0)/ 3.0 + tipLossFactor * pow(advanceRatio, 2.0) / 2.0) * flappingAngles(2) - (3.0 * pow(tipLossFactor, 2.0) * advanceRatio / 4.0) * flappingAngles(0)) * swashplateAngles(0)
						+((pow(tipLossFactor, 4.0)/ 4.0 + pow(tipLossFactor, 2.0) * pow(advanceRatio, 2.0) / 4.0) * flappingAngles(2) - (pow(tipLossFactor, 3.0) * advanceRatio / 2.0) * flappingAngles(0)) * swashplateAngles(1)
						+((pow(tipLossFactor, 2.0)/ 4.0) * aerodynamicInflowRatio + (pow(tipLossFactor, 2.0) * advanceRatio / 4.0) * flappingAngles(1)) * swashplateAngles(2)
						+(-(pow(tipLossFactor, 3.0)/ 6.0 + tipLossFactor * pow(advanceRatio, 2.0) / 2.0) * flappingAngles(0) + (pow(tipLossFactor, 2.0) * advanceRatio / 2.0) * flappingAngles(2)) * swashplateAngles(3)
						+((3.0 * pow(tipLossFactor, 2.0) / 4.0) * flappingAngles(2) - (3.0 * tipLossFactor * advanceRatio / 2.0) * flappingAngles(0)) * aerodynamicInflowRatio
						+((pow(tipLossFactor, 3.0) / 6.0) * flappingAngles(0) + (pow(tipLossFactor, 2.0) * advanceRatio / 4.0) * flappingAngles(2) - tipLossFactor * pow(advanceRatio, 2.0) * flappingAngles(0)) * flappingAngles(1)
						+((pow(tipLossFactor, 2.0) * advanceRatio / 16.0) * swashplateAngles(2) - (pow(tipLossFactor, 3.0) / 6.0) * flappingAngles(0) + (5.0 * pow(tipLossFactor, 2.0) * advanceRatio / 16.0) * flappingAngles(2)) * normalizedAerodynamicRollVelocity
						+((pow(tipLossFactor, 3.0) / 6.0) * swashplateAngles(0) + (pow(tipLossFactor, 4.0) / 8.0) * swashplateAngles(1) + (pow(tipLossFactor, 2.0) / 2.0) * aerodynamicInflowRatio + (pow(tipLossFactor, 2.0) * advanceRatio / 16.0) * swashplateAngles(3) + (7.0 * pow(tipLossFactor, 2.0) * advanceRatio / 16.0) * flappingAngles(1)) * normalizedAerodynamicPitchVelocity
				);
}

double Rotor::computeZForceFactor(double advanceRatio, double aerodynamicInflowRatio, double tipLossFactor, double epsilon0, vector<double> swashplateAngles, double normalizedAerodynamicRollVelocity) const
{
	return -(computeSolidity()*liftCurveSlope / 2.0)* // TODO : verify this sign

				(
						(pow(tipLossFactor, 3)/3.0 + pow(advanceRatio, 2.0) * tipLossFactor/2.0) * swashplateAngles(0)
						+(pow(tipLossFactor, 4)/4.0 + pow(advanceRatio, 2.0) * pow(tipLossFactor, 2)/4.0) * swashplateAngles(1)
						+(pow(tipLossFactor, 2)/2.0 + pow(advanceRatio, 2.0)/4.0)*(1 + epsilon0) * aerodynamicInflowRatio
						+(advanceRatio * pow(tipLossFactor, 2)/2.0 + pow(advanceRatio, 3.0)/8.0) * swashplateAngles(3)
						+(advanceRatio * pow(tipLossFactor, 2)/4.0) * (1 + epsilon0) * normalizedAerodynamicRollVelocity
				);
}

double Rotor::computeXMomentFactor(double advanceRatio, double aerodynamicInflowRatio, double tipLossFactor, double epsilon0, vector<double> swashplateAngles, vector<double> flappingAngles, double normalizedAerodynamicRollVelocity) const
{
	return -(computeSolidity()*liftCurveSlope / 2.0)*
				(
						(pow(tipLossFactor, 3.0) * advanceRatio /3.0) * swashplateAngles(0)
						+(pow(tipLossFactor, 4.0) * advanceRatio / 4.0) * swashplateAngles(1)
						+(pow(tipLossFactor, 2.0) * advanceRatio / 4.0) * aerodynamicInflowRatio
						+(pow(tipLossFactor, 4.0) * advanceRatio / 8.0) * normalizedAerodynamicRollVelocity
						+(pow(tipLossFactor, 4.0)/8.0 + 3 * pow(tipLossFactor, 2.0) * pow(advanceRatio, 2.0) / 16.0) * swashplateAngles(3)
						-(pow(tipLossFactor, 4.0)/8.0 + pow(tipLossFactor, 2.0) * pow(advanceRatio, 2.0) / 16.0) * flappingAngles(1)
				);
}

double Rotor::computeYMomentFactor(double advanceRatio, double aerodynamicInflowRatio, double tipLossFactor, double epsilon0, vector<double> swashplateAngles, vector<double> flappingAngles, double normalizedAerodynamicPitchVelocity) const
{
	return -(computeSolidity()*liftCurveSlope / 2.0)*
				(
						-(pow(tipLossFactor, 3.0) * advanceRatio /3.0) * flappingAngles(0)
						+(pow(tipLossFactor, 4.0) * advanceRatio / 8.0) * normalizedAerodynamicPitchVelocity
						+(pow(tipLossFactor, 4.0)/8.0 + pow(tipLossFactor, 2.0) * pow(advanceRatio, 2.0) / 16.0) * swashplateAngles(2)
						+(pow(tipLossFactor, 4.0)/8.0 + pow(tipLossFactor, 2.0) * pow(advanceRatio, 2.0) / 16.0) * flappingAngles(2)
				);
}

double Rotor::computeZFlappingMomentFactor(double advanceRatio, double aerodynamicInflowRatio, double tipLossFactor, double epsilon0, vector<double> swashplateAngles, vector<double> flappingAngles) const
{
	return (computeSolidity()*liftCurveSlope / 2.0)*
				(
						(pow(tipLossFactor, 4.0) / 4.0 + pow(tipLossFactor, 2.0) * pow(advanceRatio, 2.0) / 4.0) * epsilon0
						-((pow(tipLossFactor, 3.0)/3.0) * swashplateAngles(0) + (pow(tipLossFactor, 4.0)/4.0) * swashplateAngles(1) + (pow(tipLossFactor, 2.0)/2.0) * aerodynamicInflowRatio) * aerodynamicInflowRatio
						-(pow(tipLossFactor, 2.0) * pow(advanceRatio, 2.0) / 4.0) * pow(flappingAngles(0), 2.0)
						-(pow(tipLossFactor, 4.0)/8.0 + 3 * pow(tipLossFactor, 2.0) * pow(advanceRatio, 2.0) / 16.0) * pow(flappingAngles(1), 2.0)
						-(pow(tipLossFactor, 4.0)/8.0 + pow(tipLossFactor, 2.0) * pow(advanceRatio, 2.0) / 16.0) * pow(flappingAngles(2), 2.0)
						+(pow(tipLossFactor, 3.0) * advanceRatio/3.0) * flappingAngles(0) * flappingAngles(2)
						-(pow(tipLossFactor, 2.0) * advanceRatio/2.0) * flappingAngles(1) * aerodynamicInflowRatio
				);
}

double Rotor::computeZSwashplateMomentFactor(double advanceRatio, double aerodynamicInflowRatio, double tipLossFactor, vector<double> swashplateAngles, vector<double> flappingAngles) const
{
	return (computeSolidity()*liftCurveSlope / 2.0)*
				(
						(pow(tipLossFactor, 4.0)/8.0 - pow(tipLossFactor, 2.0) * pow(advanceRatio, 2.0) / 16.0) * swashplateAngles(3) * flappingAngles(1)
						-(pow(tipLossFactor, 2.0) * advanceRatio / 4.0) * swashplateAngles(3) * aerodynamicInflowRatio
						+(pow(tipLossFactor, 3.0) * advanceRatio / 6.0) * swashplateAngles(2) * flappingAngles(0)
						-(pow(tipLossFactor, 4.0) / 8.0 + pow(tipLossFactor, 2.0) * pow(advanceRatio, 2.0) / 16.0) * swashplateAngles(2) * flappingAngles(2)
				);
}


double Rotor::computeZRollPitchMomentFactor(double advanceRatio, double aerodynamicInflowRatio, double tipLossFactor, vector<double> swashplateAngles, vector<double> flappingAngles, double normalizedAerodynamicRollVelocity, double normalizedAerodynamicPitchVelocity) const
{
	return (computeSolidity()*liftCurveSlope / 2.0)*
				(
						(-(pow(tipLossFactor, 4.0) / 8.0) * swashplateAngles(3) - (pow(tipLossFactor, 3.0) * advanceRatio / 6.0) * swashplateAngles(0) -(pow(tipLossFactor, 4.0) * advanceRatio / 8.0) * swashplateAngles(1) -(pow(tipLossFactor, 4.0) / 8.0) * normalizedAerodynamicRollVelocity + (pow(tipLossFactor, 4.0) / 4.0) * flappingAngles(1)) * normalizedAerodynamicRollVelocity
						+(-(pow(tipLossFactor, 4.0) / 8.0) * swashplateAngles(2) + (pow(tipLossFactor, 3.0) * advanceRatio / 3.0) * flappingAngles(0) -(pow(tipLossFactor, 4.0) / 8.0) * normalizedAerodynamicPitchVelocity - (pow(tipLossFactor, 4.0) / 4.0) * flappingAngles(2)) * normalizedAerodynamicPitchVelocity
				);
}

double Rotor::computeZMomentFactor(double advanceRatio, double aerodynamicInflowRatio, double tipLossFactor, double epsilon0, vector<double> swashplateAngles, vector<double> flappingAngles, double normalizedAerodynamicRollVelocity, double normalizedAerodynamicPitchVelocity) const
{
	return 	computeZFlappingMomentFactor(advanceRatio, aerodynamicInflowRatio, tipLossFactor, epsilon0, swashplateAngles, flappingAngles)
			+ computeZSwashplateMomentFactor(advanceRatio, aerodynamicInflowRatio, tipLossFactor, swashplateAngles, flappingAngles)
			+ computeZRollPitchMomentFactor(advanceRatio, aerodynamicInflowRatio, tipLossFactor, swashplateAngles, flappingAngles, normalizedAerodynamicRollVelocity, normalizedAerodynamicPitchVelocity);
}



double Rotor::solveImplicitThrustWashRelation(double & refTipLossFactor, double & refEpsilon0, double normalizedAerodynamicRollVelocity, vector<double> swashplateAngles, double advanceRatio, double inertialInflowRatio, double oldAerodynamicInflowRatio) const
{

	double oldWashInflowRatio = inertialInflowRatio - oldAerodynamicInflowRatio;

	refTipLossFactor = Constants::STANDARD_TIP_LOSS_FACTOR;
	refEpsilon0 = dragCoefficientParameters(0) / liftCurveSlope;

	for(int i = 0; i < 10; i++)
	{
		double inertialZForceFactor = computeZForceFactor(advanceRatio, inertialInflowRatio, refTipLossFactor, refEpsilon0, swashplateAngles, normalizedAerodynamicRollVelocity);


		double newWashInflowRatio = - inertialZForceFactor /
				(
				2 * sqrt(pow(advanceRatio, 2.0) + pow(oldAerodynamicInflowRatio, 2.0))
				+(computeSolidity()*liftCurveSlope / 2.0)*(pow(refTipLossFactor, 2)/2.0 + pow(advanceRatio, 2.0)/4.0)*(1 + refEpsilon0)
				);

		newWashInflowRatio = 0.8 * newWashInflowRatio + 0.2 * oldWashInflowRatio;

		double thrustForceFactor = - computeZForceFactor(advanceRatio, oldAerodynamicInflowRatio, refTipLossFactor, refEpsilon0, swashplateAngles, normalizedAerodynamicRollVelocity);

		refTipLossFactor = computeTipLossFactor(thrustForceFactor);
		refEpsilon0 = computeEpsilon0(thrustForceFactor);

		oldWashInflowRatio = newWashInflowRatio;
		oldAerodynamicInflowRatio = inertialInflowRatio - newWashInflowRatio;



	}

	return oldAerodynamicInflowRatio;
}

double Rotor::computeConingAngle(HubType hubtype, double coningAngle, double precone)
{
	if (hubtype == TEETERING)
		return coningAngle;
	else
		return precone;
}

double Rotor::computeEpsilon0(double thrustForceFactor) const
{
	return computeNormalizedDragCoefficient(thrustForceFactor) / liftCurveSlope;
}

double Rotor::computeInflowRatio(vector<double> velocity, double tangentialVelocity) const
{
	return velocity(2)/ tangentialVelocity;
}

double Rotor::computeLockNumber(double density) const
{
    return (density * chord * liftCurveSlope * pow(radius, 4.0)) / computeBladeInertialMoment();
}

double Rotor::computeSolidity() const
{
    return (this->numberOfBlades * this->chord * M_1_PI) / this->radius ;
}

double Rotor::computeBladeFisrtMassMoment() const
{
    return linearMass * pow(radius, 2.0) / 2.0;;
}

double Rotor::computeBladeInertialMoment() const
{
    return linearMass * pow(radius, 3.0) / 3.0;
}

double Rotor::computeAdvanceRatio(vector<double> velocity, double tangentialVelocity) const
{
	return velocity(0) / tangentialVelocity;
}


double Rotor::computeAerodynamicTangentialVelocity(vector<double> aerodynamicAngularVelocity, double distanceFromRoot) const
{
	return computeAerodynamicRotorVelocity(aerodynamicAngularVelocity) * distanceFromRoot;
}

double Rotor::computeAerodynamicRotorVelocity(vector<double> aerodynamicAngularVelocity) const
{
	return rotorVelocity - aerodynamicAngularVelocity(2);
}

double Rotor::computeInertialRotorVelocity(vector<double> inertialAngularVelocity) const
{
	return rotorVelocity - inertialAngularVelocity(2);
}
void Rotor::computeForcesAndMomentsAtIELA(IndividualElement & centerOfGravity, double density)
{

	matrix<double> transformationIERAFromIELA = Utils::transformationMatrix(rotationAnglesIERAFromIELA);


	vector<double> aerodynamicVelocityIERA = prod(transformationIERAFromIELA, aerodynamicVelocityIELA);
	vector<double> aerodynamicAngularVelocityIERA = prod(transformationIERAFromIELA, aerodynamicAngularVelocityIELA);
	vector<double> washVelocityIERA = prod(transformationIERAFromIELA, washVelocityIELA);
	vector<double> washAngularVelocityIERA = prod(transformationIERAFromIELA, washAngularVelocityIELA);
	vector<double> inertialVelocityIERA = prod(transformationIERAFromIELA, inertialVelocityIELA);
	vector<double> inertialAngularVelocityIERA = prod(transformationIERAFromIELA, inertialAngularVelocityIELA);

	// First we want to rotate to WindMast axes
	double windMastAngle = atan(aerodynamicVelocityIERA(1) / aerodynamicVelocityIERA(0));
	matrix<double> windMastRotationMatrix = Utils::rotationAroundAxisZ(windMastAngle);

	// Then we rotate all velocities, cyclic (swashplate) angles, flapping angles

    vector<double> aerodynamicVelocityWindMast = prod(windMastRotationMatrix, aerodynamicVelocityIERA);
    vector<double> aerodynamicAngularVelocityWindMast = prod(windMastRotationMatrix, aerodynamicAngularVelocityIERA);
    vector<double> washVelocityWindMast = prod(windMastRotationMatrix, washVelocityIERA);
    vector<double> washAngularVelocityWindMast = prod(windMastRotationMatrix, washAngularVelocityIERA);
    vector<double> inertialVelocityWindMast = prod(windMastRotationMatrix, inertialVelocityIERA);
    vector<double> inertialAngularVelocityWindMast = prod(windMastRotationMatrix, inertialAngularVelocityIERA);

    vector<double> tempVec = prod(windMastRotationMatrix, Utils::toVec3(lateralFlappingAngle, longitudinalFlappingAngle, computeConingAngle(hubType, coningAngle, preconeAngle)));
    vector<double> flappingAnglesWindMast = Utils::toVec3(tempVec(2), tempVec(1), tempVec(0));

    tempVec = prod(windMastRotationMatrix, Utils::toVec3(longitudinalSwashplateAngle.getDegreeOfFreedomAngle(), lateralSwashplateAngle.getDegreeOfFreedomAngle(), collectivePitch.getDegreeOfFreedomAngle()));
    double swashplateAnglesWindMastArray[4] = { tempVec(2), twistAngle, tempVec(1), tempVec(0) };
    vector<double> swashplateAnglesWindMast =  Utils::toVector(swashplateAnglesWindMastArray, 4);

    double inertialRotorVelocity = computeInertialRotorVelocity(inertialAngularVelocityWindMast);
    double normalizedAerodynamicRollVelocity = aerodynamicAngularVelocityWindMast(0) / inertialRotorVelocity;
    double normalizedAerodynamicPitchVelocity = aerodynamicAngularVelocityWindMast(1) / inertialRotorVelocity;
    double epsilon0;
    double tipLossFactor;

    double aerodynamicTangentialVelocityWindMast = computeAerodynamicTangentialVelocity(aerodynamicAngularVelocityWindMast, radius);
	double advanceRatio = computeAdvanceRatio(aerodynamicVelocityWindMast, aerodynamicTangentialVelocityWindMast);
	double inertialInflowRatio = computeInflowRatio(inertialVelocityWindMast, aerodynamicTangentialVelocityWindMast);
	double aerodynamicInflowRatio = computeInflowRatio(aerodynamicVelocityWindMast, aerodynamicTangentialVelocityWindMast);

    aerodynamicInflowRatio = solveImplicitThrustWashRelation(tipLossFactor, epsilon0, normalizedAerodynamicRollVelocity, swashplateAnglesWindMast, advanceRatio, inertialInflowRatio, aerodynamicInflowRatio);
    aerodynamicVelocityWindMast(2) = aerodynamicInflowRatio * computeAerodynamicTangentialVelocity(aerodynamicAngularVelocityWindMast, radius);

    CenterOfGravity & refCenterOfGravity = dynamic_cast<CenterOfGravity & >(centerOfGravity);
	vector<double>  inertialAngularVelocityDerivativeWindMast = prod(windMastRotationMatrix, refCenterOfGravity.getInertialAngularVelocityDerivativeCenterOfGravity());

	vector<double>  inertialVelocityDerivativeWindMast = prod(windMastRotationMatrix, refCenterOfGravity.getInertialVelocityDerivativeCenterOfGravity());

	// Solve the flapping equation
	double lockNumber = computeLockNumber(density);
	double aerodynamicRotorVelocity = computeAerodynamicRotorVelocity(aerodynamicAngularVelocityWindMast);
	double normalizedNaturalFlappingFrequency = computeNaturalFlappingFrequency(lockNumber, computeBladeInertialMoment(), inertialRotorVelocity) / inertialRotorVelocity;
	double normalizedRotorVelocity = aerodynamicRotorVelocity / inertialRotorVelocity;
	double normalizedInertialRollVelocity = inertialAngularVelocityWindMast(0) / aerodynamicRotorVelocity;
	double normalizedInertialPitchVelocity = inertialAngularVelocityWindMast(1) / aerodynamicRotorVelocity;

	vector<double> accelerationVector = buildAccelerationVector(normalizedInertialRollVelocity, normalizedInertialPitchVelocity, inertialRotorVelocity, inertialVelocityDerivativeWindMast, inertialAngularVelocityDerivativeWindMast);

	flappingAnglesWindMast = solveFlappingEquation(lockNumber, tipLossFactor, epsilon0, advanceRatio, aerodynamicInflowRatio, normalizedNaturalFlappingFrequency, normalizedRotorVelocity, normalizedAerodynamicRollVelocity, normalizedAerodynamicPitchVelocity, flappingAnglesWindMast, swashplateAnglesWindMast, accelerationVector);

	flappingAnglesWindMast(0) = computeConingAngle(hubType, flappingAnglesWindMast(0), preconeAngle);


	// compute Forces and moments
	double tangentialVelocity = computeAerodynamicTangentialVelocity(aerodynamicAngularVelocityWindMast, radius);
	double forceFactorCoefficient = density * M_PI * pow(radius, 2.0) * pow(tangentialVelocity, 2.0);
	double momentFactorCoefficient = density * M_PI * pow(radius, 3.0) * pow(tangentialVelocity, 2.0);

	vector<double> forcesWindMast = forceFactorCoefficient * Utils::toVec3
			(
			computeXForceFactor(advanceRatio, aerodynamicInflowRatio, tipLossFactor, epsilon0, swashplateAnglesWindMast, flappingAnglesWindMast, normalizedAerodynamicRollVelocity, normalizedAerodynamicPitchVelocity),
			computeYForceFactor(advanceRatio, aerodynamicInflowRatio, tipLossFactor, epsilon0, swashplateAnglesWindMast, flappingAnglesWindMast, normalizedAerodynamicRollVelocity, normalizedAerodynamicPitchVelocity),
			computeZForceFactor(advanceRatio, aerodynamicInflowRatio, tipLossFactor, epsilon0, swashplateAnglesWindMast, normalizedAerodynamicRollVelocity)
			);

	vector<double> momentsWindMast(3);
	switch(hubType)
	{
		case DELTA_3 : 			break; //TODO
		case OFFSET_HINGE : 	break; //TODO

		default :				momentsWindMast(0) = hubSpringConstant * flappingAnglesWindMast(2);
								momentsWindMast(1) = hubSpringConstant * flappingAnglesWindMast(1);
	}

	momentsWindMast(2) = momentFactorCoefficient * computeZMomentFactor(advanceRatio, aerodynamicInflowRatio, tipLossFactor, epsilon0, swashplateAnglesWindMast, flappingAnglesWindMast, normalizedAerodynamicRollVelocity, normalizedAerodynamicPitchVelocity);

	// rotate back to IERA

	vector<double> forcesIERA = prod(trans(windMastRotationMatrix), forcesWindMast);

	vector<double> momentsIERA = prod(trans(windMastRotationMatrix), momentsWindMast);

	aerodynamicVelocityIERA = prod(trans(windMastRotationMatrix), aerodynamicVelocityWindMast);
	//angular velocities are unchanged


	// rotate back to IELA

	aerodynamicVelocityIELA = prod(trans(Utils::transformationMatrix(rotationAnglesIERAFromIELA)), aerodynamicVelocityIERA);

	forcesIELA = prod(trans(Utils::transformationMatrix(rotationAnglesIERAFromIELA)), forcesIERA);
	momentsIELA = prod(trans(Utils::transformationMatrix(rotationAnglesIERAFromIELA)), momentsIERA);

	washVelocityIELA = inertialVelocityIELA - aerodynamicVelocityIELA;
	//angular velocities are unchanged


	// undo the WindMast rotation of the flapping angles
    tempVec = prod(trans(windMastRotationMatrix), Utils::toVec3(flappingAnglesWindMast(2), flappingAnglesWindMast(1), flappingAnglesWindMast(0)));

    coningAngle = tempVec(2);
    longitudinalFlappingAngle = tempVec(1);
    lateralFlappingAngle = tempVec(0);

}

vector<double> Rotor::solveFlappingEquation(double lockNumber, double tipLossFactor, double epsilon0, double advanceRatio, double aerodynamicInflowRatio, double normalizedNaturalFlappingFrequency, double normalizedRotorVelocity, double normalizedAerodynamicRollVelocity, double normalizedAerodynamicPitchVelocity, vector<double> flappingAngles, vector<double> swashplateAngles, vector<double> accelerationVector) const
{
    matrix<double> flappingMatrix = buildFlappingMatrix(advanceRatio, lockNumber, tipLossFactor, epsilon0, normalizedNaturalFlappingFrequency, normalizedRotorVelocity);
    matrix<double> swashplateMatrix = buildSwashplateMatrix(advanceRatio, lockNumber, tipLossFactor, normalizedRotorVelocity);
    matrix<double> ratioMatrix = buildRatioMatrix(advanceRatio, lockNumber, tipLossFactor, epsilon0, normalizedRotorVelocity);
    matrix<double> accelerationMatrix = buildAccelerationMatrix(computeBladeFisrtMassMoment(), computeBladeInertialMoment(), flappingAngles);
    vector<double> ratioVector = Utils::toVec3(aerodynamicInflowRatio, normalizedAerodynamicRollVelocity, normalizedAerodynamicPitchVelocity);


    return prod	(
				Utils::invertMatrix(flappingMatrix),
				prod(swashplateMatrix, swashplateAngles) + prod(ratioMatrix, ratioVector) + prod(accelerationMatrix, accelerationVector)
				);
}

matrix<double> Rotor::buildFlappingMatrix(double advanceRatio, double lockNumber, double tipLossFactor, double epsilon0, double normalizedNaturalFlappingFrequency, double normalizedRotorVelocity) const
{
	matrix<double> flappingMatrix(3,3);
    using namespace std;

//    cout << "adv : " << advanceRatio << " lock :" << lockNumber << " epsilon0 : " << epsilon0 << "norm nat flap : " << normalizedNaturalFlappingFrequency << " normalizedRotorVelocity : " << normalizedRotorVelocity << endl;

	flappingMatrix(0,0) = pow(normalizedNaturalFlappingFrequency, 2.0) / (lockNumber * pow(normalizedRotorVelocity, 2.0) / 2.0);
	flappingMatrix(0,1) = ((advanceRatio * pow(tipLossFactor, 3.0)) / (6 * normalizedRotorVelocity)) * (1 + epsilon0) * (1- normalizedRotorVelocity);
	flappingMatrix(0,2) = 0;

	flappingMatrix(1,0) = 0;
	flappingMatrix(1,1) = ((pow(tipLossFactor, 4.0)/(4.0 * normalizedRotorVelocity)) - (pow(tipLossFactor, 2.0) * pow(advanceRatio,2.0) / 8.0)) * (1 + epsilon0);
	flappingMatrix(1,2) = (1 - pow(normalizedNaturalFlappingFrequency, 2.0)) / (lockNumber * pow(normalizedRotorVelocity, 2.0) / 2.0);

	flappingMatrix(2,0) = (advanceRatio * pow(tipLossFactor, 3.0) / 3.0) * (1 + epsilon0);
	flappingMatrix(2,1) = (1 - pow(normalizedNaturalFlappingFrequency, 2.0)) / (lockNumber * pow(normalizedRotorVelocity, 2.0) / 2.0);
	flappingMatrix(2,2) = -((pow(tipLossFactor, 4.0)/(4.0 * normalizedRotorVelocity)) + (pow(tipLossFactor, 2.0) * pow(advanceRatio,2.0) / 8.0)) * (1 + epsilon0);

	return (lockNumber * pow(normalizedRotorVelocity, 2.0) / 2.0) * flappingMatrix;
}

matrix<double> Rotor::buildSwashplateMatrix(double advanceRatio, double lockNumber, double tipLossFactor, double normalizedRotorVelocity) const
{
	matrix<double> swashplateMatrix(3,4);

	swashplateMatrix(0,0) = (pow(tipLossFactor, 4.0)/4.0) + (pow(tipLossFactor, 2.0) * pow(advanceRatio,2.0) / 4.0);
	swashplateMatrix(0,1) = (pow(tipLossFactor, 5.0)/5.0) + (pow(tipLossFactor, 3.0) * pow(advanceRatio,2.0) / 6.0);
	swashplateMatrix(0,2) = 0;
	swashplateMatrix(0,3) = advanceRatio * pow(tipLossFactor, 3.0) / 3.0;

	swashplateMatrix(1,0) = 2 * pow(tipLossFactor, 3.0) * advanceRatio / 3.0;
	swashplateMatrix(1,1) = pow(tipLossFactor, 4.0) * advanceRatio /2.0;
	swashplateMatrix(1,2) = 0;
	swashplateMatrix(1,3) = (pow(tipLossFactor, 4.0)/4.0) + (3 * pow(tipLossFactor, 2.0) * pow(advanceRatio,2.0) / 8.0);

	swashplateMatrix(2,0) = 0;
	swashplateMatrix(2,1) = 0;
	swashplateMatrix(2,2) = (pow(tipLossFactor, 4.0)/4.0) + (pow(tipLossFactor, 2.0) * pow(advanceRatio,2.0) / 8.0);
	swashplateMatrix(2,3) = 0;

	return (lockNumber * pow(normalizedRotorVelocity, 2.0) / 2.0) * swashplateMatrix;
}

matrix<double> Rotor::buildRatioMatrix(double advanceRatio, double lockNumber, double tipLossFactor, double epsilon0, double normalizedRotorVelocity) const
{
	matrix<double> ratioMatrix(3,3);

	ratioMatrix(0,0) = pow(tipLossFactor, 3.0) / 3;
	ratioMatrix(0,1) = advanceRatio * pow(tipLossFactor, 3.0) / 6;
	ratioMatrix(0,2) = 0;

	ratioMatrix(1,0) = (advanceRatio * pow(tipLossFactor, 2.0) / 2) -( pow(advanceRatio, 3.0) / 8.0 ) ;
	ratioMatrix(1,1) = pow(tipLossFactor, 4.0)/4.0;
	ratioMatrix(1,2) = 0;

	ratioMatrix(2,0) = 0;
	ratioMatrix(2,1) = 0;
	ratioMatrix(2,2) = pow(tipLossFactor, 4.0)/4.0;

	return (lockNumber * pow(normalizedRotorVelocity, 2.0) * (1 + epsilon0) / 2.0) * ratioMatrix;
}

matrix<double> Rotor::buildAccelerationMatrix(double bladeFirstMassMoment, double bladeInertialMassMoment, vector<double> flappingAngles) const
{//TODO change for flappingAngles vector
	matrix<double> accelerationMatrix(3,7);
	accelerationMatrix(0,0) = 0;
	accelerationMatrix(0,1) = 0;
	accelerationMatrix(0,2) = 0;
	accelerationMatrix(0,3) = 0;
	accelerationMatrix(0,4) = -(bladeFirstMassMoment * radius * flappingAngles(1) / (2 * bladeInertialMassMoment));
	accelerationMatrix(0,5) = (bladeFirstMassMoment * radius * flappingAngles(2) / (2 * bladeInertialMassMoment));
	accelerationMatrix(0,6) = (bladeFirstMassMoment * radius) / (bladeInertialMassMoment);

	accelerationMatrix(1,0) = 0;
	accelerationMatrix(1,1) = -2;
	accelerationMatrix(1,2) = 1;
	accelerationMatrix(1,3) = 0;
	accelerationMatrix(1,4) = 0;
	accelerationMatrix(1,5) = -(bladeFirstMassMoment * radius * flappingAngles(0) / (bladeInertialMassMoment));
	accelerationMatrix(1,6) = 0;

	accelerationMatrix(2,0) = 2;
	accelerationMatrix(2,1) = 0;
	accelerationMatrix(2,2) = 0;
	accelerationMatrix(2,3) = 1;
	accelerationMatrix(2,4) = (bladeFirstMassMoment * radius * flappingAngles(0) / (bladeInertialMassMoment));
	accelerationMatrix(2,5) = 0;
	accelerationMatrix(2,6) = 0;


	return accelerationMatrix;
}

double Rotor::computeNaturalFlappingFrequency(double lockNumber, double bladeInertialMoment, double rotorVelocity) const
{
    return sqrt(pow(rotorVelocity, 2.0) * (1.0 + (3.0/2.0) * (horizontalHubOffset / (radius - horizontalHubOffset)) + (hubSpringConstant / (bladeInertialMoment * pow(rotorVelocity, 2.0)) + lockNumber * tan(delta3Angle)) / 8.0));
}

void Rotor::setCyclicAngles(double collectivePitch, double lateralSwashplateAngle, double longitudinalSwashplateAngle)
{
	setCollectivePitch(collectivePitch);
	setLateralSwashplateAngle(lateralSwashplateAngle);
	setLongitudinalSwashplateAngle(longitudinalSwashplateAngle);

}

void Rotor::setCyclicStickPosition(double collectivePitchStickPosition, double lateralSwashplateAngleStickPosition, double longitudinalSwashplateAngleStickPosition)
{
	setCollectivePitchStickPosition(collectivePitchStickPosition);
	setLateralSwashplateStickPosition(lateralSwashplateAngleStickPosition);
	setLongitudinalSwashplateStickPosition(longitudinalSwashplateAngleStickPosition);
}

double Rotor::computeflappingFunction(double azimuth, double distanceFromRoot) const
{
	double normalizedDistanceFromRoot = distanceFromRoot / radius;

	return coningAngle + twistAngle * normalizedDistanceFromRoot - longitudinalFlappingAngle * cos(azimuth) - lateralFlappingAngle * sin(azimuth);
}
vector<double> Rotor::buildAccelerationVector(double normalizedInertialRollVelocity, double normalizedInertialPitchVelocity, double inertialRotorVelocity, vector<double> inertialVelocityDerivative, vector<double> inertialAngularVelocityDerivative) const
{
	vector<double> accelerationVector(7);

	accelerationVector(0) = normalizedInertialRollVelocity;
	accelerationVector(1) = normalizedInertialPitchVelocity;
	accelerationVector(2) = inertialAngularVelocityDerivative(0) / pow(inertialRotorVelocity, 2.0);
	accelerationVector(3) = inertialAngularVelocityDerivative(1) / pow(inertialRotorVelocity, 2.0);
	accelerationVector(4) = inertialVelocityDerivative(0) / (radius * pow(inertialRotorVelocity, 2.0));
	accelerationVector(5) = inertialVelocityDerivative(1) / (radius * pow(inertialRotorVelocity, 2.0));
	accelerationVector(6) = inertialVelocityDerivative(2) / (radius * pow(inertialRotorVelocity, 2.0));

	return accelerationVector;
}
