/*
 * Common.cpp
 *
 *  Created on: 25-Aug-2008
 *      Author: malem303
 */

#include "Common.h"



CenterOfGravity createCenterOfGravity(vector<double> distIELAfromRefCog, vector<double> initialPositionEarthAxes, vector<double> initialAngularPositionEarthAxes, vector<double> initialVelocityEarthAxes, vector<double> initialAngularVelocityEarthAxes)
{

	matrix<double> inertiaTensor(Constants::ZERO3x3);
    inertiaTensor(0, 0) = 4180.0;
    inertiaTensor(1, 1) = 10700.0;
    inertiaTensor(2, 2) = 10300.0;

    return CenterOfGravity(distIELAfromRefCog, (6700.0 / Constants::GRAVITY), inertiaTensor, initialPositionEarthAxes, initialAngularPositionEarthAxes, initialVelocityEarthAxes, initialAngularVelocityEarthAxes);

}

IndividualElement createIndividualElement(double radius, vector<double> distanceIELAFromReferenceAxes, vector<double> rotationAnglesIERAFromIELA)
{
	double selfInfluenceFactor = 1 / (M_PI * pow(radius, 2.0));

	return IndividualElement(Constants::IDENTITY3x3 * selfInfluenceFactor, distanceIELAFromReferenceAxes, rotationAnglesIERAFromIELA);
}
