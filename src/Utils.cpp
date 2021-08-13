/*
 * Utils.cpp
 *
 *  Created on: 17-Jul-2008
 *      Author: malem303
 */

#include "Utils.h"

Utils::Utils() {
	// TODO Auto-generated constructor stub

}

Utils::~Utils() {
	// TODO Auto-generated destructor stub
}
double Utils::vectorNorm(vector<double> vec)
{
	return sqrt(inner_prod(vec,vec));
}

double Utils::degToRad(double angleInDegrees)
{
	return angleInDegrees * M_PI / 180.0;
}

vector<double> Utils::toVec3( double x, double y, double z)
{
    vector<double> vec3(3);
    vec3(0) = x;
    vec3(1) = y;
    vec3(2) = z;

    return vec3;
}

vector<double> Utils::toVector(double * array, int numberOfElements)
{
	vector<double> vec(numberOfElements);
	for(int i = 0; i < numberOfElements; i++)
		vec(i) = array[i];

	return vec;
}

matrix<double> Utils::toMatrix(double* twoDimensionsArray, unsigned int m, unsigned int n)
{
	matrix<double> matMN(m,n);
    for (unsigned int i = 0; i < m; i++)
    	for (unsigned int j = 0; j < n; j++)
    		matMN(i, j) = twoDimensionsArray[(m * i) + j];

    return matMN;

}

matrix<double> Utils::rotationAroundAxisX(double rotationAngle)
{
	double rotationArray[3*3] =
	{
			1.0,	0.0,					0.0,
			0.0,	cos(rotationAngle),		sin(rotationAngle),
			0.0,	-sin(rotationAngle),	cos(rotationAngle)
	};

    return toMatrix(rotationArray, 3, 3);
}

matrix<double> Utils::rotationAroundAxisY(double rotationAngle)
{
	double rotationArray[3*3] =
	{
			cos(rotationAngle),		0.0,		-sin(rotationAngle),
			0.0,					1.0,		0.0,
			sin(rotationAngle),		0.0,		cos(rotationAngle)
	};

    return toMatrix(rotationArray, 3, 3);
}

matrix<double> Utils::rotationAroundAxisZ(double rotationAngle)
{
	double rotationArray[3*3] =
	{
			cos(rotationAngle),		sin(rotationAngle),		0.0,
			-sin(rotationAngle),	cos(rotationAngle),		0.0,
			0.0,					0.0,					1.0
	};

    return Utils::toMatrix(rotationArray, 3, 3);
}

matrix<double> Utils::transformationMatrix(double thetaX, double thetaY, double thetaZ)
{
	double sinX = sin(thetaX);
	double sinY = sin(thetaY);
	double sinZ = sin(thetaZ);

	double cosX = cos(thetaX);
	double cosY = cos(thetaY);
	double cosZ = cos(thetaZ);

	double rotationArray[3*3] =
	{
			cosY*cosZ,						cosY*sinZ,						-sinY,
			sinX*sinY*cosZ - cosX*sinZ,		sinX*sinY*sinZ + cosX*cosZ,		sinX*cosY,
			cosX*sinY*cosZ + sinX*sinZ,		cosX*sinY*sinZ - sinX*cosZ,		cosX*cosY
	};

	return toMatrix(rotationArray, 3, 3);

//		matrix<double> xy = prod(rotationAroundAxisX(thetaX), rotationAroundAxisY(thetaY));
//
//		return prod(xy, rotationAroundAxisZ(thetaZ));
}


matrix<double> Utils::buildEulerTransformationMatrix(double roll, double pitch)
{
	double eulerRotationArray[3*3] =
	{
			1.0,		0.0,			-sin(pitch),
			0.0,		cos(roll),		sin(roll)*cos(pitch),
			0.0,		-sin(roll),		cos(roll)*cos(pitch)
	};

	return Utils::toMatrix(eulerRotationArray, 3, 3);
}

matrix<double> Utils::buildEulerTransformationMatrixInverse(double roll, double pitch)
{

	double eulerRotationArray[3*3] =
	{
			1.0,		sin(roll) * tan(pitch),		cos(roll)* tan(pitch),
			0.0,		cos(pitch),					-sin(roll),
			0.0,		sin(roll)/cos(pitch),		cos(roll)/cos(pitch)
	};

	return Utils::toMatrix(eulerRotationArray, 3, 3);
}


matrix<double> Utils::transformationMatrix(vector<double> thetaXYZ)
{
	return transformationMatrix(thetaXYZ(0), thetaXYZ(1), thetaXYZ(2));
}
matrix<double> Utils::transformationCenterOfGravityFromReferenceAxes()
{
	return rotationAroundAxisY(degToRad(180.0));
}

vector<double> Utils::crossProduct(vector<double> leftHandSide, vector<double> rightHandSide)
{
	double x = rightHandSide(0);
	double y = rightHandSide(1);
	double z = rightHandSide(2);

	double crossProductOfRightHandSideArray[3*3] =
	{
			0.0,	-z,		y,
			z,		0.0,	-x,
			-y,		x,		0.0
	};

	return static_cast<vector<double> >
	(
			prod(-Utils::toMatrix(crossProductOfRightHandSideArray, 3, 3), leftHandSide)
	);
}


matrix<double> Utils::invertMatrix(matrix<double> matrixToInvert) {

	// credits : http://www.crystalclearsoftware.com/cgi-bin/boost_wiki/wiki.pl?LU_Matrix_Inversion

	matrix<double> invertedMatrix = identity_matrix<double>(matrixToInvert.size1());

	// create a workingCopy

	permutation_matrix<std::size_t> permutationMatrix(matrixToInvert.size1());


 	// perform LU-factorization
 	double res = lu_factorize(matrixToInvert, permutationMatrix);
        if( res != 0.0 )
        	return invertedMatrix;

 	// backsubstitute to get the inverse
    lu_substitute(matrixToInvert, permutationMatrix, invertedMatrix);

 	return invertedMatrix;
 }

bool Utils::equals(const matrix<double> lhs, const matrix<double> rhs, double epsilon)
{
	matrix<double>::array_type lhsData = lhs.data();
	matrix<double>::array_type rhsData = rhs.data();
	matrix<double>::array_type::iterator rhsIterator = rhsData.begin();

	for (matrix<double>::array_type::iterator lhsIterator = 0; lhsIterator != lhsData.end() && rhsIterator != rhsData.end(); lhsIterator++)
	{
		if (equals(*lhsIterator, *rhsIterator, epsilon))
			return false;

		rhsIterator++;
	}
	return true;
}


bool Utils::equals(const vector<double> lhs, const vector<double> rhs, double epsilon)
{
	if (lhs.size() != rhs.size())
		return false;

	vector<double>::const_iterator rhsIterator = rhs.begin();
	for (vector<double>::const_iterator lhsIterator = lhs.begin(); lhsIterator != lhs.end() && rhsIterator != rhs.end(); lhsIterator++)
	{
		if (!equals(*lhsIterator, *rhsIterator, epsilon))
			return false;

		rhsIterator++;
	}

	return true;
}


bool Utils::equals(double lhs, double rhs, double epsilon)
{
	return fabs(lhs - rhs) < epsilon;
}
