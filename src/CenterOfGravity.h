/*
 * CenterOfGravity.h
 *
 *  Created on: 17-Jul-2008
 *      Author: malem303
 */

#ifndef CENTEROFGRAVITY_H_
#define CENTEROFGRAVITY_H_

#include "IndividualElement.h"
#include "Constants.h"


class CenterOfGravity: public IndividualElement {

private:
	/*! \f$ \mathbf{p}_{E}\f$ */
	vector<double>	positionEarthAxes;

	/*! \f$ \mathbf{\alpha}_{i|E}\f$ */
	vector<double>	angularPositionEarthAxes;

	/*! \f$ \dot{\mathbf{p}}_{E}\f$ */
	vector<double>	positionDerivativeEarthAxes;

	/*! \f$ \dot{\mathbf{\alpha}}_{i|E}\f$ */
	vector<double>	angularPositionDerivativeEarthAxes;

	/*! \f$ \dot{\mathbf{v}}_{i|CoG}\f$ */
	vector<double>	inertialVelocityDerivativeCenterOfGravity;

	/*! \f$ \dot{\mathbf{\omega}}_{i|CoG}\f$ */
	vector<double>	inertialAngularVelocityDerivativeCenterOfGravity;

	/*! \f$ \mathbf{I}_{n}\f$ */
	matrix<double> interiaTensor;

	double mass;
public:

	/*!
	 * Reset the forces and moments to zero
	 */
	void resetForces();

	/*!
	 * Translate velocities from earth axes to CoG axes using
	 * 	\f$ \begin{array}{c}
		\dot{\mathbf{p}}_{E}\\
		\dot{\alpha}_{E}\end{array}=\begin{array}{c}
		\mathbf{T}_{CoG-E}^{-1}(\phi,\theta,\psi)\mathbf{v}_{CoG}\\
		\mathbf{E}_{CoG-E}^{-1}(\phi,\theta)\omega_{CoG}\end{array} \f$
	 */
	void updateVelocities();

	/*!
	 * Adds the gravity forces to the current forcesIELA
	 * \f$ \mathbf{f}_{g}=\left[\begin{array}{c}
		-\sin(\theta)\\
		\sin(\phi)\cos(\theta)\\
		\cos(\phi)\cos(\theta)\end{array}\right]\cdot m\cdot g \f$

	 */
	void addGravityForces();

	/*!
	 * Updates the derivatives (i.e. the CoG accelerations and earth velocities) using
	 * the forces and moments at the CoG
	 * It uses this set of formulas :
	 * 	\f$ \begin{array}{c}
		\dot{\mathbf{v}}_{CoG}=\mathbf{M}_{CoG}^{-1}\left[\mathbf{f}_{CoG}-\omega_{CoG}\times\left(\mathbf{M}_{CoG}\mathbf{v}_{CoG}\right)\right]\\
		\dot{\omega}_{CoG}=\mathbf{I}_{n}^{-1}\left[\mathbf{f}_{CoG}^{\curvearrowright}-\omega_{CoG}\times\left(I_{n}\omega_{CoG}\right)\right]\end{array}
		\f$
	 */
    void updateDerivatives();

    CenterOfGravity(vector<double> distanceIELAFromReferenceAxes, double mass, matrix<double> intertiaTensor, vector<double> positionEarthAxes = Constants::ZERO_VEC_3, vector<double> angularPositionEarthAxes = Constants::ZERO_VEC_3, vector<double> positionDerivativeEarthAxes = Constants::ZERO_VEC_3, vector<double> angularPositionDerivativeEarthAxes = Constants::ZERO_VEC_3, vector<double> inertialVelocityDerivativeCenterOfGravity = Constants::ZERO_VEC_3, vector<double> inertialAngularVelocityDerivativeCenterOfGravity = Constants::ZERO_VEC_3);


    virtual ~CenterOfGravity();

     /*!
     * @return \f$\mathbf{x|CoG}\f$  the state vector
     */
    vector<double> getState() const;

    /*!
     *
     * @param mass \f$ m \f$
     * @return the mass matrix \f$ \mathbf{M}_{CoG}=\left[\begin{array}{ccc}
									m & 0 & 0\\
									0 & m & 0\\
									0 & 0 & m\end{array}\right]\f$
     */
    matrix<double> buildMassMatrix(double mass) const;

    /*!
     *  Also known as earth axes velocity
     * @return \f$ \dot{\mathbf{p}}_{E} \f$
     */
    vector<double> getPositionDerivativeEarthAxes() const
    {
        return positionDerivativeEarthAxes;
    }

    /*!
     *  Also known as earth axes angular velocity
     * @return \f$ \dot{\mathbf{\omega}}_{E} \f$
     */
    vector<double> getAngularPositionDerivativeEarthAxes() const
    {
        return angularPositionDerivativeEarthAxes;
    }

    /*!
     *  Also known as earth axes position derivative
     * @return \f$ \dot{\mathbf{p}}_{E} \f$
     */
    vector<double> getVelocityEarthAxes() const
    {
        return positionDerivativeEarthAxes;
    }


    /*!
     *
     * @param velocityEarthAxes \f$ \dot{\mathbf{p}}_{E} \f$ a.k.a. position derivative
     */
    void setVelocityEarthAxes(vector<double> velocityEarthAxes)
    {
        this->positionDerivativeEarthAxes = velocityEarthAxes;
    }

    /*!
     * Also known as earth axes angular position derivative
     * @return \f$ \dot{\mathbf{\omega}}_{E} \f$
     */
    vector<double> getAngularVelocityEarthAxes() const
    {
        return angularPositionDerivativeEarthAxes;
    }

    /*!
     *
     * @param angularVelocityEarthAxes \f$ \dot{\mathbf{\omega}}_{E} \f$ a.k.a. angular position derivative
     */
    void setAngularVelocityEarthAxes(vector<double> angularVelocityEarthAxes)
    {
        this->angularPositionDerivativeEarthAxes = angularVelocityEarthAxes;
    }

    /*!
     *
     * @return \f$ \mathbf{p}_{E} = \left(n,e,d\right)\f$ north, east, down coords from reference axes
     */
    vector<double> getPositionEarthAxes() const
    {
        return positionEarthAxes;
    }

    /*!
     *
     * @param positionEarthAxes \f$ \mathbf{p}_{E} = \left(n,e,d\right)\f$ north, east, down coords from origin
     */
    void setPositionEarthAxes(vector<double> positionEarthAxes)
    {
        this->positionEarthAxes = positionEarthAxes;
    }

    /*!
     *
     * @return \f$ \mathbf{\omega}_{E} = \left(\phi,\theta,\psi\right)\f$ roll, pitch, yaw angles from earth axes orientation
     */
    vector<double> getAngularPositionEarthAxes() const
    {
        return angularPositionEarthAxes;
    }

    /*!
     *
     * @param angularPositionEarthAxes \f$ \mathbf{\omega}_{E} = \left(\phi,\theta,\psi\right)\f$ roll, pitch, yaw angles from earth axes orientation
     */
    void setAngularPositionEarthAxes(vector<double> angularPositionEarthAxes)
    {
        this->angularPositionEarthAxes = angularPositionEarthAxes;
    }

    /*!
     *
     * @return \f$ \dot{\mathbf{v}}_{CoG} \f$ a.k.a acceleration in CoG axes
     */
    vector<double> getInertialVelocityDerivativeCenterOfGravity() const
    {
        return inertialVelocityDerivativeCenterOfGravity;
    }

    /*!
     *
     * @return \f$ \dot{\mathbf{\omega}}_{CoG} \f$ a.k.a angular acceleration in CoG axes
     */
    vector<double> getInertialAngularVelocityDerivativeCenterOfGravity() const
    {
        return inertialAngularVelocityDerivativeCenterOfGravity;
    }

    /*!
     * The inertia tensor can be viewed as an expression of how the mass is disposed around the CoG
     * @return \f$ \mathbf{I}_{n}=\left[\begin{array}{ccc}
		I_{xx} & -I_{xy} & -I_{xz}\\
		-I_{xy} & I_{yy} & -I_{yz}\\
		-I_{xz} & -I_{yz} & I_{zz}\end{array}\right] \f$
     */
    matrix<double> getInteriaTensor() const
    {
        return interiaTensor;
    }

    /*!
     *  The inertia tensor can be viewed as an expression of how the mass is disposed around the CoG
     * @param interiaTensor \f$ \mathbf{I}_{n}=\left[\begin{array}{ccc}
		I_{xx} & -I_{xy} & -I_{xz}\\
		-I_{xy} & I_{yy} & -I_{yz}\\
		-I_{xz} & -I_{yz} & I_{zz}\end{array}\right] \f$
	 * The elements are :
	 * \f$ \begin{array}{c}
		\int(xy)dm=I_{xy}\\
		\int(xz)dm=I_{xz}\\
		\int(yz)dm=I_{yz}\\
		\int(y^{2}+z^{2})dm=I_{xx}\\
		\int(x^{2}+z^{2})dm=I_{yy}\\
		\int(x^{2}+y^{2})dmsI_{zz}\\
		\end{array}\f$
     */
    void setInteriaTensor(matrix<double> interiaTensor)
    {
        this->interiaTensor = interiaTensor;
    }

    /*!
     *
     * @return \f$ m \f$ the mass
     */
    double getMass() const
    {
        return mass;
    }

    /*!
     *
     * @param mass \f$ m \f$ the mass
     */
    void setMass(double mass)
    {
        this->mass = mass;
    }

};

#endif /* CENTEROFGRAVITY_H_ */
