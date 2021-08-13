/*
 * HalfWing.cpp
 *
 *  Created on: 1-Aug-2008
 *      Author: malem303
 */

#include "HalfWing.h"
#include  "Utils.h"


HalfWing::HalfWing(HalfWingType halfWingType, double halfWingSpan, double chordAtRoot, double chordAtTip, vector<double> rotationAnglesWRAFromIERA, double liftCurveSlope,  vector<double> dragCoefficientParameters, vector<double> momentCoefficientParameters, matrix<double> selfInducedWashVelocityModel, vector<double> distanceIELAFromReferenceAxes, vector<double> rotationAnglesIERAFromIELA) :
	IndividualElement(selfInducedWashVelocityModel, distanceIELAFromReferenceAxes, rotationAnglesIERAFromIELA), halfWingType(halfWingType), halfWingSpan(halfWingSpan), chordAtRoot(chordAtRoot), chordAtTip(chordAtTip),
	rotationAnglesWRAFromIERA(rotationAnglesWRAFromIERA), liftCurveSlope(liftCurveSlope),
	dragCoefficientParameters(dragCoefficientParameters), momentCoefficientParameters(momentCoefficientParameters)
{
	// TODO Auto-generated constructor stub

}

HalfWing::~HalfWing()
{
	// TODO Auto-generated destructor stub
}
double HalfWing::computeTaperRatio() const
{
	return chordAtTip / chordAtRoot;
}
double HalfWing::computeMeanAerodynamicChord() const

{
	double taperRatio = computeTaperRatio();
    return 2.0 * chordAtRoot * ((pow(taperRatio, 2.0) + taperRatio + 1)) / (3 * (taperRatio + 1));

}

double HalfWing::computeCentroid() const
{
	double taperRatio = computeTaperRatio();
    return halfWingSpan * 2.0 * ((2 * taperRatio + 1)) / (6 * (taperRatio + 1));
}

double HalfWing::computeAspectRatio() const
{
	return pow(halfWingSpan * 2.0, 2.0) / (computeHalfWingArea() * 2);
}

double HalfWing::computeLiftCoefficient(double angleOfAttack) const
{
	double aspectRatio = computeAspectRatio();

	return liftCurveSlope * aspectRatio / (aspectRatio + 2 * (aspectRatio + 4)/ (aspectRatio + 2)) * angleOfAttack;
}

double HalfWing::computeDragCoefficient(double angleOfAttack)  const
{
	return dragCoefficientParameters(0) + angleOfAttack * (dragCoefficientParameters(1) + dragCoefficientParameters(2) * angleOfAttack);
}

double HalfWing::computeMomentCoefficient(double angleOfAttack)  const
{
	return momentCoefficientParameters(0) + momentCoefficientParameters(1) * angleOfAttack;
}
/*!
 * The formula is \f$ \alpha=\tan^{-1}\left(\frac{w_{\overline{c}|WRA}}{u_{\overline{c}|WRA}}\right) \f$
 * @warning make sure meanAerodynamicChordVelocityWRA (\f$ \mathbf{v}_{\overline{c}|WRA} \f$) is up to date
 * @return \f$ \alpha \f$ the angle of attack
 */
double HalfWing::computeAngleOfAttack(vector<double> meanAerodynamicChordVelocityWRA)  const
{
	return atan(meanAerodynamicChordVelocityWRA(2) / meanAerodynamicChordVelocityWRA(0));
}

double HalfWing::computeSideSlipAngle(vector<double> meanAerodynamicChordVelocityWRA)  const
{

	return 0.0; //TODO: Fix Me
}
void HalfWing::computeForcesAndMomentsAtIELA(IndividualElement & centerOfGravity, double density)
{
	matrix<double> transformationIERAFromIELA = Utils::transformationMatrix(rotationAnglesIERAFromIELA);

	matrix<double> transformationWRAFromIERA = Utils::transformationMatrix(rotationAnglesWRAFromIERA);

	vector<double> aerodynamicVelocityIERA = prod(transformationIERAFromIELA, aerodynamicVelocityIELA);
	vector<double> aerodynamicAngularVelocityIERA = prod(transformationIERAFromIELA, aerodynamicAngularVelocityIELA);

	/*! \f$ \mathbf{d}_{\overline{c}} \f$ */
	matrix<double> transformationCentroid = prod(trans(Utils::rotationAroundAxisZ(rotationAnglesWRAFromIERA(2))), trans(Utils::rotationAroundAxisX(rotationAnglesWRAFromIERA(0))));

	vector<double> distanceMeanAerodynamicChordFromIERA = prod(transformationCentroid, Utils::toVec3(0.0, (float)halfWingType * computeCentroid(), 0.0));

	vector<double> meanAerodynamicChordVelocityIERA = aerodynamicVelocityIERA + Utils::crossProduct(aerodynamicAngularVelocityIERA, distanceMeanAerodynamicChordFromIERA);
	vector<double> meanAerodynamicChordAngularVelocityIERA = aerodynamicAngularVelocityIERA;

	vector<double> meanAerodynamicChordVelocityWRA = prod(transformationWRAFromIERA, meanAerodynamicChordVelocityIERA);
	vector<double> meanAerodynamicChordAngularVelocityWRA = prod(transformationWRAFromIERA, meanAerodynamicChordAngularVelocityIERA);

	double angleOfAttck = computeAngleOfAttack(meanAerodynamicChordVelocityWRA);

	double dynamicPressure = density * inner_prod(meanAerodynamicChordVelocityWRA, meanAerodynamicChordVelocityWRA) * computeHalfWingArea() / 2.0;

	double lift = dynamicPressure * computeLiftCoefficient(angleOfAttck);
	double drag = dynamicPressure * computeDragCoefficient(angleOfAttck);
	double pitchMoment = dynamicPressure * computeMeanAerodynamicChord() * computeMomentCoefficient(angleOfAttck);

	vector<double> meanAerodynamicChordForces = Utils::toVec3(-drag, 0.0, -lift);
	vector<double> meanAerodynamicChordMoments = Utils::toVec3(0.0, pitchMoment, 0.0);
	matrix<double> transformationAerodynamicFromWRA = Utils::transformationMatrix(computeSideSlipAngle(meanAerodynamicChordVelocityWRA), -angleOfAttck, 0.0);

	vector<double> meanAerodynamicChordForcesWRA = prod(trans(transformationAerodynamicFromWRA), meanAerodynamicChordForces);
	vector<double> meanAerodynamicChordMomentsWRA = prod(trans(transformationAerodynamicFromWRA), meanAerodynamicChordMoments);

	vector<double> meanAerodynamicChordForcesIERA = prod(trans(transformationWRAFromIERA), meanAerodynamicChordForcesWRA);
	vector<double> meanAerodynamicChordMomentsIERA = prod(trans(transformationWRAFromIERA), meanAerodynamicChordMomentsWRA);


	vector<double> forcesIERA = meanAerodynamicChordForcesIERA;
	vector<double> momentsIERA = meanAerodynamicChordMomentsIERA + Utils::crossProduct(distanceMeanAerodynamicChordFromIERA, meanAerodynamicChordForcesIERA);

	forcesIELA = prod(trans(Utils::transformationMatrix(rotationAnglesIERAFromIELA)), forcesIERA);
	momentsIELA = prod(trans(Utils::transformationMatrix(rotationAnglesIERAFromIELA)), momentsIERA);

}

double HalfWing::computeHalfWingArea() const
{
	return halfWingSpan * chordAtRoot * (1 + computeTaperRatio()) / 2.0; // 4.0 --> 2.0  since span = 2 * halfWingSpan
}

