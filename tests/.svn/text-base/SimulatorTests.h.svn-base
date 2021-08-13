#include "cute.h"
#include "ide_listener.h"
#include "cute_runner.h"
#include "Simulator.h"
#include "IndividualElementTest.h"
#include "FuselageTest.h"
#include "HalfWingTest.h"
#include "WingTest.h"
#include "RotorHelperFunctionsTest.h"
#include "RotorTest.h"
#include "AircraftTest.h"
#include "Common.h"

void thisIsATest() {
//	Simulator sim;

//	sim.start();

	ASSERTM("TODO Sim tests", false);
}

void runSuite(){
	cute::suite s;
	//TODO add your test here
	s.push_back(CUTE(thisIsATest));
	cute::ide_listener lis;
	cute::makeRunner(lis)(s, "The Suite");
	cute::makeRunner(lis)(make_suite_IndividualElementTest(), "IndividualElementTest");
	cute::makeRunner(lis)(make_suite_FuselageTest(), "FuselageTest");
	cute::makeRunner(lis)(make_suite_HalfWingTest(), "HalfWingTest");
	cute::makeRunner(lis)(make_suite_WingTest(), "WingTest");
	cute::makeRunner(lis)(make_suite_RotorHelperFunctionsTest(), "RotorHelperFunctionsTest");
	cute::makeRunner(lis)(make_suite_RotorTest(), "RotorTest");
	cute::makeRunner(lis)(make_suite_AircraftTest(), "AircraftTest");
}
