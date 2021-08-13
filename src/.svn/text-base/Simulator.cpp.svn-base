/*
 * Simulator.cpp
 *
 *  Created on: 31-Jul-2008
 *      Author: malem303
 */

#include "Simulator.h"
#include "Utils.h"


Simulator::Simulator() : aircraft(true)
{
// TODO Auto-generated constructor stub

}

Simulator::~Simulator()
{
	// TODO Auto-generated destructor stub
}

void Simulator::start()
{
	double vinf_kts = 5.0;
	double roc_fps = 0.0;

	vector<double> initialPositionEarthAxes = Utils::toVec3(0.0, 0.0, 0.0);
	vector<double> initialAngularPositionEarthAxes = Utils::toVec3(0.0, 0.0, 0.0);
	vector<double> initialVelocityEarthAxes = Utils::toVec3(vinf_kts * 1.6878, 0.0, -roc_fps/60.0);
	vector<double> initialAngularVelocityEarthAxes = Utils::toVec3(0.0, 0.0, 0.0);

	aircraft.jacobianTrim(initialPositionEarthAxes, initialAngularPositionEarthAxes, initialVelocityEarthAxes, initialAngularVelocityEarthAxes);

}
