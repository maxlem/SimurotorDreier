/*
 * DegreeOfFreedom.h
 *
 *  Created on: 24-Jul-2008
 *      Author: malem303
 */

#ifndef DEGREEOFFREEDOM_H_
#define DEGREEOFFREEDOM_H_
#include "ControlRigging.h"

class DegreeOfFreedom
{
private:
	double degreeOfFreedomAngle;
	ControlRigging controlRigging;

public:
	DegreeOfFreedom(ControlRigging controlRigging);
	virtual ~DegreeOfFreedom();

    /*!
     *
     * @return \f$ \theta_{DoF} \f$
	 * @note Replace DoF by the actual degree of freedom name (e.g. collective)
     */
    double getDegreeOfFreedomAngle() const
    {
        return degreeOfFreedomAngle;
    }

    /*!
     *
     * @param degreeOfFreedomAngle \f$ \theta_{DoF} \f$
	 * @note Replace DoF by the actual degree of freedom name (e.g. collective)
	 * @return false if the angle is out of range
     */
    bool setDegreeOfFreedomAngle(double degreeOfFreedomAngle);

    /*!
     *
	 * @note Replace DoF by the actual degree of freedom name (e.g. collective)
	 * @return  \f$ \theta_{DoF}^{stick} \f$
     */
    double getStickPosition() const;

    /*!
     *
     * @param stickPosition \f$ \theta_{DoF}^{stick} \f$
	 * @note Replace DoF by the actual degree of freedom name (e.g. collective)
	 * @return false if the angle is out of range
     */
    bool setStickPosition(double stickPosition);
};

#endif /* DEGREEOFFREEDOM_H_ */
