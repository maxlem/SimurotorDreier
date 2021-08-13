/*
 * Fuselage.h
 *
 *  Created on: 14-Jul-2008
 *      Author: malem303
 */

#ifndef FUSELAGE_H_
#define FUSELAGE_H_

#include "IndividualElement.h"
#include "CenterOfGravity.h"
#include "AtmosphericConditions.h"
#include "Constants.h"
class Fuselage : public IndividualElement {
private :
	double caracteristicArea;
	double caracteristicLenght;

    matrix<double> mutualInfluenceModelForForces;
    matrix<double> mutualInfluenceModelForMoments;

public:
	// inherited doc
    void computeForcesAndMomentsAtIELA(IndividualElement & centerOfGravity, double density);
    Fuselage(double caracteristicArea, double caracteristicLenght, matrix<double> selfInducedWashVelocityModel, matrix<double> mutualInfluenceModelForForces, matrix<double> mutualInfluenceModelForMoments, vector<double> distanceIELAFromReferenceAxes = Constants::ZERO_VEC_3, vector<double> rotationAnglesIERAFromIELA = Constants::ZERO_VEC_3);
    virtual ~Fuselage();

};

#endif /* FUSELAGE_H_ */

