/*
 * Wing.h
 *
 *  Created on: 15-Jul-2008
 *      Author: malem303
 */

#ifndef WING_H_
#define WING_H_

#include "IndividualElement.h"
#include "AtmosphericConditions.h"
#include "CenterOfGravity.h"
#include "HalfWing.h"

/*!
 * This class implements the Composite (GoF, p163) design pattern
 */
class Wing: public IndividualElement {

private:
	HalfWing rightWing;
	HalfWing leftWing;


public:
	/*! updatePhysics step 1 */
	void translateInertialVelocitiesToIELA(IndividualElement & centerOfGravity);

	/*! updatePhysics step 2 */
	void computeAerodynamicVelocities(double density);

	/*! updatePhysics step 3 */
	void computeForcesAndMomentsAtIELA(IndividualElement & centerOfGravity, double density);

	/*! updatePhysics step 4 */
	void sumForcesAndMomentsToCenterOfGravity(IndividualElement & centerOfGravity);

	/*! updatePhysics step 5 */
	void updtateWashVelocities(double density, vector<double> externalWashVelocity, vector<double> externalWashAngularVelocity);


	/*!
	 *
	 * @param span \f$ l_{wing} \f$ the full Left and right)
	 * @param chordAtRoot \f$ c_{root} \f$
	 * @param chordAtTip  \f$ c_{tip} \f$
	 * @param dihedralAngle \f$ \Gamma \f$
	 * @param incidenceAngle \f$ \theta \f$
	 * @param sweepAngle \f$ \Lambda \f$
	 * @note 	These three angles are used to build the
	 * 			\f$ \mathbf{T}_{WRA-IERA} \f$ transformation matrix
	 *  		- for a right half wing, the parameters should be \f$ (-\Gamma,\theta,\Lambda) \f$
	 * 			- for a left half wing, they should be \f$ (\Gamma,\theta,-\Lambda) \f$
	 * This is because of the definition of dihedral and sweep angles.
	 *
	 * @param liftCurveSlope \f$ \alpha_{0} \f$, also known as \f$ \dot{C_{l}(\alpha)}_{2D} \f$
	 * @param dragCoefficientParameters the function used in Dreier, 2007, p456 is \f$ C_{d}\left(\alpha\right)=c_{d_{0}}+\alpha\left(c_{d_{1}}+\alpha\cdot c_{d_{2}}\right) \f$
	 * @param momentCoefficientParameters the function used in Dreier, 2007, p456 is \f$ C_{m}\left(\alpha\right)=c_{m_{0}}+c_{m_{\alpha}}\cdot\alpha \f$
	 * @param selfInducedWashVelocityModel the self wash model
	 * @return
	 */
    Wing(double span, double chordAtRoot, double chordAtTip, double dihedralAngle, double incidenceAngle, double sweepAngle, double liftCurveSlope,  vector<double> dragCoefficientParameters, vector<double> momentCoefficientParameters, matrix<double> selfInducedWashVelocityModel);
    ~Wing();
    //inherited
    vector<double> getWashAngularVelocityIELA() const
	{
		return (leftWing.getWashAngularVelocityIELA() + rightWing.getWashAngularVelocityIELA() ) / 2.0;
	}
    //inherited
	void setWashAngularVelocityIELA(vector<double> washAngularVelocityIELA)
	{
		leftWing.setWashAngularVelocityIELA(washAngularVelocityIELA);
		rightWing.setWashAngularVelocityIELA(washAngularVelocityIELA);
	}
    //inherited
	vector<double> getAerodynamicVelocityIELA() const
	{
		return (leftWing.getAerodynamicVelocityIELA() + rightWing.getAerodynamicVelocityIELA() ) / 2.0;
	}
    //inherited
	vector<double> getAerodynamicAngularVelocityIELA() const
	{
		return (leftWing.getAerodynamicAngularVelocityIELA() + rightWing.getAerodynamicAngularVelocityIELA() ) / 2.0;
	}
    //inherited
	vector<double> getForcesIELA() const
	{
		return (leftWing.getForcesIELA() + rightWing.getForcesIELA());
	}
    //inherited
	void setForcesIELA(vector<double> forcesIELA)
	{
		leftWing.setForcesIELA(forcesIELA / 2.0);
		rightWing.setForcesIELA(forcesIELA / 2.0);
	}
    //inherited
	vector<double> getMomentsIELA() const
	{
		return (leftWing.getMomentsIELA() + rightWing.getMomentsIELA());
	}
    //inherited
	void setMomentsIELA(vector<double> momentsIELA)
	{
		leftWing.setMomentsIELA(momentsIELA / 2.0);
		rightWing.setMomentsIELA(momentsIELA / 2.0);
	}
    //inherited
	vector<double> getInertialVelocityIELA() const
	{
		return (leftWing.getInertialVelocityIELA() + rightWing.getInertialVelocityIELA() ) / 2.0;
	}
    //inherited
	void setInertialVelocityIELA(vector<double> inertialVelocityIELA)
	{
		leftWing.setInertialVelocityIELA(inertialVelocityIELA);
		rightWing.setInertialVelocityIELA(inertialVelocityIELA);
	}

	vector<double> getInertialAngularVelocityIELA() const
	{
		return (leftWing.getInertialAngularVelocityIELA() + rightWing.getInertialAngularVelocityIELA() ) / 2.0;
	}
    //inherited
	void setInertialAngularVelocityIELA(vector<double> inertialAngularVelocityIELA)
	{
		leftWing.setInertialAngularVelocityIELA(inertialAngularVelocityIELA);
		rightWing.setInertialAngularVelocityIELA(inertialAngularVelocityIELA);
	}
    //inherited
	vector<double> getWashVelocityIELA() const
	{
		return (leftWing.getWashVelocityIELA() + rightWing.getWashVelocityIELA() ) / 2.0;
	}
    //inherited
	void setWashVelocityIELA(vector<double> washVelocityIELA)
	{
		leftWing.setWashVelocityIELA(washVelocityIELA);
		rightWing.setWashVelocityIELA(washVelocityIELA);
	}
    //inherited
	vector<double> getDistanceIELAFromReferenceAxes() const
	{
		return (leftWing.getDistanceIELAFromReferenceAxes() + rightWing.getDistanceIELAFromReferenceAxes() ) / 2.0;
	}
    //inherited
	void setDistanceIELAFromReferenceAxes(vector<double> distanceIELAFromReferenceAxes)
	{
		leftWing.setDistanceIELAFromReferenceAxes(distanceIELAFromReferenceAxes);
		rightWing.setDistanceIELAFromReferenceAxes(distanceIELAFromReferenceAxes);
	}
    //inherited
	vector<double> getRotationAnglesIERAFromIELA() const
	{
		return (leftWing.getDistanceIELAFromReferenceAxes() + rightWing.getDistanceIELAFromReferenceAxes() ) / 2.0;
	}
    //inherited
	void setRotationAnglesIERAFromIELA(vector<double> rotationAnglesIERAFromIELA)
	{
		leftWing.setRotationAnglesIERAFromIELA(rotationAnglesIERAFromIELA);
		rightWing.setRotationAnglesIERAFromIELA(rotationAnglesIERAFromIELA);
	}

};

#endif /* WING_H_ */
