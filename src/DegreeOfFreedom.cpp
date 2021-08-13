/*
 * DegreeOfFreedom.cpp
 *
 *  Created on: 24-Jul-2008
 *      Author: malem303
 */

#include "DegreeOfFreedom.h"

DegreeOfFreedom::DegreeOfFreedom(ControlRigging controlRigging):
	controlRigging(controlRigging)
{
	// TODO Auto-generated constructor stub

}

DegreeOfFreedom::~DegreeOfFreedom()
{
	// TODO Auto-generated destructor stub
}
bool DegreeOfFreedom::setDegreeOfFreedomAngle(double degreeOfFreedomAngle)
{
	bool desiredValue = true;

	if (degreeOfFreedomAngle < controlRigging.getMinDegreeOfFreedomAngle())
	{
		degreeOfFreedomAngle = controlRigging.getMinDegreeOfFreedomAngle();
		desiredValue = false;
	}
	if (degreeOfFreedomAngle > controlRigging.getMaxDegreeOfFreedomAngle())
	{
		degreeOfFreedomAngle = controlRigging.getMaxDegreeOfFreedomAngle();
		desiredValue = false;
	}

    this->degreeOfFreedomAngle = degreeOfFreedomAngle;

    return desiredValue;
}

double DegreeOfFreedom::getStickPosition() const
{
	return controlRigging.getStickPositionFor(degreeOfFreedomAngle);
}

bool DegreeOfFreedom::setStickPosition(double stickPosition)
{
	bool desiredValue = true;

	if (stickPosition < controlRigging.getMinStickPosition())
	{
		stickPosition = controlRigging.getMinStickPosition();
		desiredValue = false;
	}

	if (stickPosition > controlRigging.getMaxStickPosition())
	{
		stickPosition = controlRigging.getMaxStickPosition();
		desiredValue = false;
	}

    this->degreeOfFreedomAngle = controlRigging.getDegreOfFreedomAngleFor(stickPosition);
    return desiredValue;
}
