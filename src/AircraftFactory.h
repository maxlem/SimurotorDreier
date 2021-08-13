/*
 * AircraftFactory.h
 *
 *  Created on: 25-Aug-2008
 *      Author: malem303
 */

#ifndef AIRCRAFTFACTORY_H_
#define AIRCRAFTFACTORY_H_
#include "Fuselage.h"
#include "Rotor.h"
#include "Wing.h"
#include "CenterOfGravity.h"
#include "AtmosphericConditions.h"
#include "Utils.h"
/*!
 *  The aircraft factory is a helper class designed to build
 *  TODO make this class abstract
 */
class AircraftFactory
{
public:
	AircraftFactory();
	virtual ~AircraftFactory();

	/*!
	 * Creates a new Fuselage instance, allocated from heap
	 * @return the new instance
	 * @post memory won't be disposed automatically
	 */
	static Fuselage * createFuselage();

	/*!
	 * Creates a new Wing instance, allocated from heap
	 * @return the new instance
	 * @post memory won't be disposed automatically
	 */
    static Wing * createHorizontalStabilizer();

	/*!
	 * Creates a new HalfWing instance, allocated from heap
	 * @return the new instance
	 * @post memory won't be disposed automatically
	 */
    static HalfWing *  createVerticalStabilizer();

	/*!
	 * Creates a new Rotor instance, allocated from heap
	 * @return the new instance
	 * @post memory won't be disposed automatically
	 */
    static Rotor *  createMainRotor();

    /*!
	 * Creates a new Rotor instance, allocated from heap
	 * @return the new instance
	 * @post memory won't be disposed automatically
	 */
    static Rotor *  createTailRotor(double delta3Angle = 45.0 / 57.295); //Utils::degToRad(45.0));

	/*!
	 * Creates a new FusCenterOfGravityelage instance, allocated from heap
	 * @return the new instance
	 * @post memory won't be disposed automatically
	 */
    static CenterOfGravity * createCenterOfGravity(vector<double> distIELAfromRefCog = Constants::ZERO_VEC_3, vector<double> initialPositionEarthAxes = Constants::ZERO_VEC_3, vector<double> initialAngularPositionEarthAxes = Constants::ZERO_VEC_3, vector<double> initialVelocityEarthAxes = Constants::ZERO_VEC_3, vector<double> initialAngularVelocityEarthAxes = Constants::ZERO_VEC_3);
};

#endif /* AIRCRAFTFACTORY_H_ */
