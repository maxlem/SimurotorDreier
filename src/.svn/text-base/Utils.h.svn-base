/*
 * Utils.h
 *
 *  Created on: 17-Jul-2008
 *      Author: malem303
 */

#ifndef UTILS_H_
#define UTILS_H_
#include <math.h>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/lu.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <iostream>
#include <iomanip>

using namespace boost::numeric::ublas;

class Utils {
public:

	/*!
	 * Compute the vector's norm (a.k.a length for 2D or 3D vectors)
	 * @param vec the vector
	 * @return	vec's norm
	 */
	static double vectorNorm(vector<double> vec);

	static double degToRad(double angleInDegrees);

	static vector<double> toVec3( double x, double y, double z);

	static vector<double> toVector(double * array, int numberOfElements);

	static matrix<double> toMatrix(double* twoDimensionsArray, unsigned int m, unsigned int n);

	static matrix<double> rotationAroundAxisX(double rotationAngle);

	static matrix<double> rotationAroundAxisY(double rotationAngle);

	static matrix<double> rotationAroundAxisZ(double rotationAngle);

	static matrix<double> transformationMatrix(double thetaX, double thetaY, double thetaZ);

    /*!
     *  Used to build the classic \f$ \mathbf{T}_{CoG-E}(\phi,\theta,\psi)=\mathbf{R}_{x}(\phi)\mathbf{R}_{y}(\theta)\mathbf{R}_{z}(\psi) \f$
     * @return \f$ \mathbf{T}_{CoG-E}(\phi,\theta,\psi) \f$ the transformation matrix for linear rates
     */
	static matrix<double> transformationMatrix(vector<double> thetaXYZ);

	static matrix<double> transformationCenterOfGravityFromReferenceAxes();

    /*!
     *  \f$ \mathbf{E}_{CoG-E}(\phi,\theta)=\left[\begin{array}{ccc}
						1 & 0 & -\sin(\theta)\\
						0 & \cos(\phi) & \sin(\phi)\cos(\theta)\\
						0 & -\sin(\phi) & \cos(\phi)\cos(\theta)\end{array}\right]\f$
     * @return \f$ \mathbf{E}_{CoG-E}(\phi,\theta) \f$ the transformation matrix for angular (Euler) rates
     */
	static matrix<double> buildEulerTransformationMatrix(double roll, double pitch);

	static matrix<double> buildEulerTransformationMatrixInverse(double roll, double pitch);


	/*!
	 *
		\f$ \mathbf{C}(\mathbf{r})=\left[\begin{array}{ccc}
		0 & -z & y\\
		z & 0 & -x\\
		-y & x & 0\\
		\end{array}\right] \f$
		\f$ \mathbf{crossProduct}=\mathbf{leftHandSide}\times\mathbf{rightHandSide}=-\mathbf{rightHandSide}\times\mathbf{leftHandSide}=\mathbf{C}^{T}(\mathbf{rightHandSide})\mathbf{leftHandSide}=-\mathbf{C}(\mathbf{rightHandSide})\mathbf{leftHandSide} \f$

	 * @param leftHandSide the first openrand
	 * @param rightHandSide the second operand
	 * @return the cross product \f$ \mathbf{leftHandSide}\times\mathbf{rightHandSide} \f$
	 */
	static vector<double> crossProduct(vector<double> leftHandSide, vector<double> rightHandSide);

	/*!
	 * Invert a matrix using LU factorisation
	 * @param matrixToInvert the matrix to be inverted
	 * @return  the inverted matrix
	 */
	static matrix<double> invertMatrix(matrix<double> matrixToInvert);

	static bool equals(const matrix<double> lhs, const matrix<double> rhs, double epsilon);

	static bool equals(const vector<double> lhs, const vector<double> rhs, double epsilon);

	static bool equals(double lhs, double rhs, double epsilon);

	Utils();
	virtual ~Utils();
};

#endif /* UTILS_H_ */
