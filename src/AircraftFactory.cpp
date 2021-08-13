/*
 * AircraftFactory.cpp
 *
 *  Created on: 25-Aug-2008
 *      Author: malem303
 */

#include "AircraftFactory.h"
#include "Constants.h"
#include "Utils.h"
#include "ControlRigging.h"

AircraftFactory::AircraftFactory()
{
	// TODO Auto-generated constructor stub

}

AircraftFactory::~AircraftFactory()
{
	// TODO Auto-generated destructor stub
}



Fuselage * AircraftFactory::createFuselage()
{
    Fuselage * fuselage;

    double caracteristicArea = 49.0;
    double caracteristicLenght = 37.3;
	double mutualInfluenceModelForForcesArray[3*3] =
	{
		0.377,		0.0,	0.233,
		-0.035,		1.78,	0.0,
		0.0715,		0.0,	0.755
	};
	double mutualInfluenceModelForMomentsArray[3*3] =
	{
		0.0,		0.0,	0.0,
		0.00822,	0.0,	0.518,
		0.00274,	-0.144,	0.0
	};

	double selfInducedWashVelocityModel[3*3] =
	{
		0.0163,	0.0,	0.0,
		0.0,	0.0183,	0.0,
		0.0,	0.0,	0.0183
	};

	vector<double> distanceIELAFromReferenceAxes = Utils::toVec3(10.0, 0.0, 5.0);
	vector<double> rotationAnglesIERAFromIELA = Constants::ZERO_VEC_3;

	fuselage = new Fuselage	(
						caracteristicArea, caracteristicLenght, Utils::toMatrix(selfInducedWashVelocityModel, 3, 3),
						Utils::toMatrix(mutualInfluenceModelForForcesArray, 3, 3), Utils::toMatrix(mutualInfluenceModelForMomentsArray, 3, 3),
						distanceIELAFromReferenceAxes, rotationAnglesIERAFromIELA
							);

	return fuselage;
}

Wing * AircraftFactory::createHorizontalStabilizer()
{
    double span = 9.33;
    double chordAtRoot = 2.54;
    double chordAtTip = 2.54; // taperRatio = 1.0
    double dihedralAngle = 0.0;
    double incidenceAngle = 0.0;
    double sweepAngle = 0.0;
    double liftCurveSlope = Constants::STANDARD_LIFT_CURVE_SLOPE;
    vector<double> dragCoefficientParameters = Utils::toVec3(0.0105, 0.0, 0.01325);
    vector<double> momentCoefficientParameters(2);
    momentCoefficientParameters(0) = 0.0;
    momentCoefficientParameters(1) = 0.0;

//  double  area(23.6); computed automatically //TODO : 23.6 is very badly rounded, this will incur problem when comparing results

    double selfInfluenceFactor = 4 / (M_PI * pow(span, 2.0));
    matrix<double> selfInducedWashVelocityModel = Constants::IDENTITY3x3 * selfInfluenceFactor;

    Wing * horizontalStabilizer = new Wing(span, chordAtRoot, chordAtTip, dihedralAngle, incidenceAngle, sweepAngle, liftCurveSlope, dragCoefficientParameters, momentCoefficientParameters, selfInducedWashVelocityModel);

	horizontalStabilizer->setDistanceIELAFromReferenceAxes(Utils::toVec3(26.9, 0.0, 5.25));
    horizontalStabilizer->setRotationAnglesIERAFromIELA(Utils::toVec3(0, 0, 0));

    return horizontalStabilizer;

}

HalfWing *  AircraftFactory::createVerticalStabilizer()
{

	double halfWingSpan = 5.51;
	double chordAtRoot = 2.9;
	double chordAtTip = 0.0;
	vector<double> rotationAnglesWRAFromIERA = Utils::toVec3(0.0, 0.0, 0.0);
	double liftCurveSlope = Constants::STANDARD_LIFT_CURVE_SLOPE;
	vector<double> dragCoefficientParameters = Utils::toVec3(0.0105, 0.0, 0.01325);
    vector<double> momentCoefficientParameters(2);
    momentCoefficientParameters(0) = 0.0;
    momentCoefficientParameters(1) = 0.0;

    double selfInfluenceFactor = 4 / (M_PI * pow(halfWingSpan, 2.0));
    matrix<double> selfInducedWashVelocityModel = Constants::IDENTITY3x3 * selfInfluenceFactor;

	vector<double> distanceIELAFromReferenceAxes = Utils::toVec3(34.8, 0.0, 9.06);
	vector<double> rotationAnglesIERAFromIELA = Utils::toVec3(90.0 / 57.295, 0.0, 0.0); //Utils::degToRad(90.0), 0.0, 0.0);

	HalfWing *  verticalStabilizer = new HalfWing(FIN, halfWingSpan, chordAtRoot, chordAtTip, rotationAnglesWRAFromIERA, liftCurveSlope, dragCoefficientParameters, momentCoefficientParameters, selfInducedWashVelocityModel, distanceIELAFromReferenceAxes, rotationAnglesIERAFromIELA);


    return verticalStabilizer;
}

Rotor *  AircraftFactory::createMainRotor()
{
	int numberOfBlades = 2;
	double radius = 22.0;
	double chord = 2.25;
	double linearMass = 0.395;
	double twistAngle = -10.0/ 57.295; //Utils::degToRad(-10);
	double delta3Angle = 0.0;

	HubType hubType = FULLY_ATRICULATED;
	double liftCurveSlope = Constants::STANDARD_LIFT_CURVE_SLOPE;
	vector<double> dragCoefficientParameters = Utils::toVec3(0.0105, 0.0, 0.01325);
	double horizontalHubOffset = 0.0;
	double verticalHubOffset = 0.0;
	double hubSpringConstant = 0.0;
	ControlRigging collectivePitchRigging(Utils::degToRad(10.0),Utils::degToRad(25.0), 0.0, 8.0);
	ControlRigging lateralSwashplateRigging(Utils::degToRad(-10.0),Utils::degToRad(10.0), -4.0, 4.0);
	ControlRigging longitudinalSwashplateRigging(Utils::degToRad(-10.0),Utils::degToRad(10.0), -4.0, 4.0);

	double preconeAngle = 1.875 / 57.295; //Utils::degToRad(1.875);

	double selfInfluenceFactor = 1 / (M_PI * pow(radius, 2.0));

	matrix<double> selfInducedWashVelocityModel = Constants::IDENTITY3x3 * selfInfluenceFactor;

	Rotor * mainRotor = new Rotor 	(
						numberOfBlades, radius, chord, linearMass, delta3Angle, hubType, preconeAngle, liftCurveSlope,
						dragCoefficientParameters, horizontalHubOffset, verticalHubOffset, hubSpringConstant,
						collectivePitchRigging, lateralSwashplateRigging, longitudinalSwashplateRigging, twistAngle, selfInducedWashVelocityModel
							);

    mainRotor->setRotorVelocity(32.5);
    mainRotor->setCyclicAngles(0.2476, 0.0, 0.0);
    mainRotor->setWashVelocityIELA(Utils::toVec3(0.0, 0.0, 10.0));
    mainRotor->setDistanceIELAFromReferenceAxes(Utils::toVec3(10.0, 0.0, 11.6));
    mainRotor->setRotationAnglesIERAFromIELA(Utils::toVec3(0.0, -0.0523, 0.0));

    return mainRotor;
}

Rotor *  AircraftFactory::createTailRotor(double delta3Angle)
{
	int numberOfBlades = 2;
	double radius = 4.25;
	double chord = 0.702;
	double linearMass = 0.08;
	double twistAngle = 0.0;

	HubType hubType = FULLY_ATRICULATED;
	double liftCurveSlope = Constants::STANDARD_LIFT_CURVE_SLOPE;
	vector<double> dragCoefficientParameters = Utils::toVec3(0.0105, 0.0, 0.01325);
	double horizontalHubOffset = 0.0;
	double verticalHubOffset = 0.0;
	double hubSpringConstant = 0.0;
	ControlRigging collectivePitchRigging(Utils::degToRad(-45.0),Utils::degToRad(45.0), -2.0, 2.0); //Utils::degToRad(-12.0),Utils::degToRad(1.0), -2.0, 2.0);
	ControlRigging lateralSwashplateRigging(0.0, 0.0, 0.0, 0.0);
	ControlRigging longitudinalSwashplateRigging(0.0, 0.0, 0.0, 0.0);

	double preconeAngle= 0.0;

	double selfInfluenceFactor = 1 / (M_PI * pow(radius, 2.0));
	matrix<double> selfInducedWashVelocityModel = Constants::IDENTITY3x3 * selfInfluenceFactor;

	Rotor *  tailRotor = new Rotor 	(
						numberOfBlades, radius, chord, linearMass, delta3Angle, hubType, preconeAngle, liftCurveSlope,
						dragCoefficientParameters, horizontalHubOffset, verticalHubOffset, hubSpringConstant,
						collectivePitchRigging, lateralSwashplateRigging, longitudinalSwashplateRigging, twistAngle, selfInducedWashVelocityModel
							);

	tailRotor->setRotorVelocity(172.0);
    tailRotor->setCyclicAngles(5.0/57.295, 0.0, 0.0); //Utils::degToRad(5.0), 0.0, 0.0);
	tailRotor->setWashVelocityIELA(Utils::toVec3(0.0, -10.0, 0.0));
	tailRotor->setDistanceIELAFromReferenceAxes(Utils::toVec3(36.7, -1.2, 10.6));
	tailRotor->setRotationAnglesIERAFromIELA(Utils::toVec3(-1.57078, 0.0, 0.0)); // Utils::toVec3(Utils::degToRad(-90.0), 0.0, 0.0));

	return tailRotor;

}




CenterOfGravity * AircraftFactory::createCenterOfGravity(vector<double> distIELAfromRefCog, vector<double> initialPositionEarthAxes, vector<double> initialAngularPositionEarthAxes, vector<double> initialVelocityEarthAxes, vector<double> initialAngularVelocityEarthAxes)
{

	matrix<double> inertiaTensor(Constants::ZERO3x3);
    inertiaTensor(0, 0) = 4180.0;
    inertiaTensor(1, 1) = 10700.0;
    inertiaTensor(2, 2) = 10300.0;

    return new CenterOfGravity(distIELAfromRefCog, (6700.0 / Constants::GRAVITY), inertiaTensor, initialPositionEarthAxes, initialAngularPositionEarthAxes, initialVelocityEarthAxes, initialAngularVelocityEarthAxes);
}
