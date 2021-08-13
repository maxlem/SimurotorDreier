/*
 * Simulator.h
 *
 *  Created on: 31-Jul-2008
 *      Author: malem303
 */

#ifndef SIMULATOR_H_
#define SIMULATOR_H_

#include "Aircraft.h"


class Simulator
{
private:
	Aircraft aircraft;
public:
	void start();


	Simulator();
	virtual ~Simulator();
};

#endif /* SIMULATOR_H_ */
