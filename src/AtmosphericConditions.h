/*
 * AtmosphericConditions.h
 *
 *  Created on: 25-Jul-2008
 *      Author: malem303
 */

#ifndef ATMOSPHERICCONDITIONS_H_
#define ATMOSPHERICCONDITIONS_H_

/*!
 *  This class is responsible for holding atmospheric data and for computing corrections based on atmospheric data
 */
class AtmosphericConditions
{

private:
	/*! \f$ \rho \f$ */
	double rho;
public:
	AtmosphericConditions();
	virtual ~AtmosphericConditions();

	/*!
	 * @pre
	 * @return \f$ \rho \f$ the current air density
	 */
    double getDensity() const
    {
        return rho;
    }

    /*!
     *
     * @param rho \f$ \rho \f$ the air density
     */
    void setRho(double rho)
    {
        this->rho = rho;
    }

};

#endif /* ATMOSPHERICCONDITIONS_H_ */
