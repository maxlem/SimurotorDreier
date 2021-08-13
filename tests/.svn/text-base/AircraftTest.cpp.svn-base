#include "cute.h"
#include "ide_listener.h"
#include "cute_runner.h"
#include "AircraftTest.h"
#include "Aircraft.h"
#include "Common.h"

void Aircraft_jacobianTrim() {
	Aircraft aircraft(true);
	double vinf_kts = 5.0;
	double roc_fps = 0.0;
	double accelerationVectorNorm = aircraft.jacobianTrim(Constants::ZERO_VEC_3, Constants::ZERO_VEC_3, Utils::toVec3(vinf_kts * 1.6878, 0.0, -roc_fps/60.0), Constants::ZERO_VEC_3);

	std::cout << accelerationVectorNorm;
	ASSERT_EQUALM("not zero", 0.0, accelerationVectorNorm);

}

cute::suite make_suite_AircraftTest(){
	cute::suite s;
	s.push_back(CUTE(Aircraft_jacobianTrim));
	return s;
}



