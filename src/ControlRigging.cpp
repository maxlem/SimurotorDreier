/*
 * ControlRigging.cpp
 *
 *  Created on: 24-Jul-2008
 *      Author: malem303
 */

#include "ControlRigging.h"

ControlRigging::ControlRigging(double minDegreeOfFreedomAngle, double maxDegreeOfFreedomAngle, double minStickPosition, double maxStickPosition) :
	minDegreeOfFreedomAngle(minDegreeOfFreedomAngle), maxDegreeOfFreedomAngle(maxDegreeOfFreedomAngle), minStickPosition(minStickPosition), maxStickPosition(maxStickPosition)
{
	// TODO Auto-generated constructor stub
}

ControlRigging::~ControlRigging()
{
	// TODO Auto-generated destructor stub
}

double ControlRigging::getDegreOfFreedomAngleFor(double stickPosition) const
{
	return minDegreeOfFreedomAngle + (maxDegreeOfFreedomAngle - minDegreeOfFreedomAngle) * ( (stickPosition - minStickPosition) / (maxStickPosition - minStickPosition));
}

double ControlRigging::getStickPositionFor(double degreeOfFreedomAngle) const
{
	return (degreeOfFreedomAngle - minDegreeOfFreedomAngle) / (maxDegreeOfFreedomAngle - minDegreeOfFreedomAngle) * (maxStickPosition - minStickPosition) + minStickPosition;
}
