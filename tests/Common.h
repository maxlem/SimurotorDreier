/*
 * Common.h
 *
 *  Created on: 25-Aug-2008
 *      Author: malem303
 */

#ifndef COMMON_H_
#define COMMON_H_
#include "IndividualElement.h"
#include "CenterOfGravity.h"
#include "Constants.h"
#include "Utils.h"
#include "AircraftFactory.h"
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <iostream>


using namespace boost::numeric::ublas::detail;

static const double epsilon = 0.0001;


CenterOfGravity createCenterOfGravity(vector<double> distIELAfromRefCog = Constants::ZERO_VEC_3, vector<double> initialPositionEarthAxes = Constants::ZERO_VEC_3, vector<double> initialAngularPositionEarthAxes = Constants::ZERO_VEC_3, vector<double> initialVelocityEarthAxes = Constants::ZERO_VEC_3, vector<double> initialAngularVelocityEarthAxes = Constants::ZERO_VEC_3);

IndividualElement createIndividualElement(double radius = 1.0, vector<double> distanceIELAFromReferenceAxes = Constants::ZERO_VEC_3, vector<double> rotationAnglesIERAFromIELA = Constants::ZERO_VEC_3);

#endif /* COMMON_H_ */
