/*
 * ControlRigging.h
 *
 *  Created on: 24-Jul-2008
 *      Author: malem303
 */

#ifndef CONTROLRIGGING_H_
#define CONTROLRIGGING_H_
#include <string>

class ControlRigging
{
private:

	/*! \f$ \theta_{DoF}^{min} \f$, the actual degree of freedom minimum angle on the aircraft (ex. \f$ \theta_{tail}^{min} \f$, \f$ \theta_{swash_{front}}^{min} \f$, an aileron, etc) */
	double minDegreeOfFreedomAngle;

	/*! \f$ \theta_{DoF}^{max} \f$, the actual degree of freedom maximum angle on the aircraft (ex. \f$ \theta_{tail}^{max} \f$, \f$ \theta_{swash_{front}}^{max} \f$, an aileron, etc) */
	double maxDegreeOfFreedomAngle;


	/*! \f$ \theta_{DoF}^{stick_{min}} \f$, the actual degree of freedom minimum angle on the aircraft (ex. \f$ \theta_{tail}^{stick_{min}} \f$, \f$ \theta_{swash_{front}}^{stick_{min}} \f$, an aileron, etc) */
	double minStickPosition;

	/*! \f$ \theta_{DoF}^{stick_{max}} \f$, the actual degree of freedom maximum angle on the aircraft (ex. \f$ \theta_{tail}^{stick_{max}} \f$, \f$ \theta_{swash_{front}}^{stick_{max}} \f$, an aileron, etc) */
	double maxStickPosition;

public:
	ControlRigging(double minDegreeOfFreedomAngle, double maxDegreeOfFreedomAngle, double minStickPosition, double maxStickPosition);
	virtual ~ControlRigging();

	/*!
	 * \f$ \theta_{DoF}=\theta_{DoF}^{min}+\left(\theta_{DoF}^{max}-\theta_{DoF}^{min}\right)\left(\frac{\theta_{DoF}^{stick}-\theta_{DoF}^{stick_{min}}}{\theta_{DoF}^{stick_{max}}-\theta_{DoF}^{stick_{min}}}\right) \f$
	 * @param stickPosition \f$ \theta_{DoF}^{stick} \f$ the stick position or angle
	 * @return	\f$ \theta_{DoF} \f$ the corresponding degree of freedom position or angle
	 * @note Replace DoF by the actual degree of freedom name (e.g. collective)
	 */
	double getDegreOfFreedomAngleFor(double stickPosition) const;

	/*!
	 * The inverse of function \c getDegreOfFreedomAngleFor()
	 * @param degreeOfFreedomAngle \f$ \theta_{DoF} \f$ the degree of freedom angle
	 * @return \f$ \theta_{DoF}^{stick_{min}} \f$ the corresponding stick position or angle
	 */
	double getStickPositionFor(double degreeOfFreedomAngle) const;

	/*!
	 *
	 * @return \f$ \theta_{DoF}^{stick_{min}} \f$
	 * @note Replace DoF by the actual degree of freedom name (e.g. collective)
	 */
    double getMinStickPosition() const
    {
        return minStickPosition;
    }

    /*!
     *
     * @return \f$ \theta_{DoF}^{stick_{max}} \f$
	 * @note Replace DoF by the actual degree of freedom name (e.g. collective)
     */
    double getMaxStickPosition() const
    {
        return maxStickPosition;
    }

    /*!
     *
     * @return \f$ \theta_{DoF}^{min} \f$
	 * @note Replace DoF by the actual degree of freedom name (e.g. collective)
     */
    double getMinDegreeOfFreedomAngle() const
    {
        return minDegreeOfFreedomAngle;
    }

    /*!
     *
     * @return \f$ \theta_{DoF}^{max} \f$
	 * @note Replace DoF by the actual degree of freedom name (e.g. collective)
     */
    double getMaxDegreeOfFreedomAngle() const
    {
        return maxDegreeOfFreedomAngle;
    }

};

#endif /* CONTROLRIGGING_H_ */
