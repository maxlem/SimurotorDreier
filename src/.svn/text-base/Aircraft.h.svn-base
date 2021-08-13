/*
 * Aircraft.h
 *
 *  Created on: 11-Jul-2008
 *      Author: malem303
 */

#ifndef AIRCRAFT_H_
#define AIRCRAFT_H_

#include "Fuselage.h"
#include "Rotor.h"
#include "Wing.h"
#include "CenterOfGravity.h"
#include "AtmosphericConditions.h"

#include <math.h>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>

using namespace boost::numeric::ublas;

/*!
 * This class use a combination of IndividualElement (fuselage, wings, rotors) to compose an aircraft
 */

class Aircraft {

private:
	bool usePhi; //TODO : rename

	AtmosphericConditions * atmosphericConditions;
    Fuselage * fuselage;
    Rotor * mainRotor;
    Rotor * tailRotor;
    Wing * horizontalStabilizer;
    HalfWing * verticalStabilizer;
    CenterOfGravity * centerOfGravity;

    vector<double> buildTrimVector();

    void applyTrimVector(vector<double> trimVector);
    vector<double> buildAccelerationVector();


    /*!
     * The atmospheric condition change according to meteo (pressure, wind, temperature) and altitude
     */
    void updateAtmosphericConditions();

    /*!
     * Center of gravity data comprise of two things :
     * - mass property (center of gravity location, \f$ \mathbf{d}_{IELA-Ref} \f$ inertia tensor, \f$ \mathbf{I}_{n} \f$), they vary according to many factors :
     * 	- fuel burn rate
     * 	- cargo unloading
     * 	- structural changes (tiltrotors, camera movement)
     * 	- etc
     * - inertial velocities (\f$ \mathbf{v}_{i|CoG} \f$, \f$ \mathbf{\omega}_{i|CoG} \f$) :
     * 	- they are set from earth velocities, wich are computed at the simulation loop (we are at the begining)
     */
    void updatePhysics();

    void updateWashVelocities();

    double computeRotorInfluenceFactor(Rotor * rotor, IndividualElement * influencedElement);
    /*!
     * Apply the new control stick positions to the degrees of freedom
     * @param collectivePitchStickPosition \f$ \theta_{collective}^{stick} \f$
     * @param longitudinalSwashplateStickPosition \f$ \theta_{swash_{front}}^{stick} \f$
     * @param lateralSwashplateStickPosition \f$ \theta_{swash_{side}}^{stick} \f$
     * @param tailRotorCollectivePitchStick \f$ \theta_{tail}^{stick} \f$
     */
    void updateControls(double collectivePitchStickPosition, double longitudinalSwashplateStickPosition, double lateralSwashplateStickPosition, double tailRotorCollectivePitchStick);


    void printForces();

    void printWashes();
public:

	/*!
	 * The jacobian trim uses the partial derivatives of linear and angular acceleration with respect to chosen
	 * degrees of freedom of the aircraft (e.g. cyclic angles, tail collective and/or some Euler angles)
	 *
	 * @param initialPositionEarthAxes \f$ \mathbf{p}_{E} \f$
	 * @param initialAngularPositionEarthAxes \f$ \mathbf{\omega}_{E} \f$
	 * @param initialVelocityEarthAxes \f$ \dot{\mathbf{p}_{E}} \f$
	 * @param initialAngularVelocityEarthAxes \f$ \dot{\mathbf{\omega}_{E}} \f$
	 * @return \f$ \left\Vert \begin{array}{c} \dot{\mathbf{v}}_{CoG}\\ \dot{\omega}_{CoG}\end{array}\right\Vert  \f$ the norm of the acceleration vector
	 */
	double jacobianTrim(vector<double> initialPositionEarthAxes, vector<double> initialAngularPositionEarthAxes, vector<double> initialVelocityEarthAxes, vector<double> initialAngularVelocityEarthAxes);



	Aircraft(bool usePhi);
	virtual ~Aircraft();
};

#endif /* AIRCRAFT_H_ */
