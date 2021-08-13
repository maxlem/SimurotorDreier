/*
 * Constants.h
 *
 *  Created on: 11-Jul-2008
 *      Author: malem303
 */

#ifndef SICONSTANTS_H_
#define SICONSTANTS_H_

#include <boost/numeric/ublas/matrix.hpp>
using namespace boost::numeric::ublas;
/*!
 * Useful constants  TODO : with SI units
 */
class Constants {
public:
	/*!
	 * \f$ g \f$ the gravity constant
	 */
	static const double GRAVITY = 32.174; // slugs/ft^2

	/*!
	 * \f$ \rho_{SSL} \f$ the air density of a normal day at Standard Sea Level
	 */
	static const double RHO_SSL = 0.002378; // slugs/ft

	static const identity_matrix<double> IDENTITY3x3;
	static const zero_matrix<double> ZERO3x3;
	static const zero_vector<double> ZERO_VEC_3;
	static const vector<double> UNIT_VEC_3;

	/*!
	 * \f$ a_{0} \f$
	 */
	static const double STANDARD_LIFT_CURVE_SLOPE = 5.73;

	/*!
	 * \f$ B_{T} \f$
	 */
	static const double STANDARD_TIP_LOSS_FACTOR = 0.97;
	Constants();
	virtual ~Constants();
};

#endif /* SICONSTANTS_H_ */
