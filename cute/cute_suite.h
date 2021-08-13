#ifndef CUTE_SUITE_H_
#define CUTE_SUITE_H_
#include "cute_test.h"
#include <vector>
namespace cute {
	typedef std::vector<test> suite;
	// convenience operator for appending to suites, might not be right
	// can use boost/assign.hpp instead...
	inline 
	suite &operator+=(suite &left, suite const &right){
		left.insert(left.end(),right.begin(),right.end());
		return left;
	}
	inline
	suite &operator+=(suite &left, test const &right){
		left.push_back(right);
		return left;
	}
}
#endif /*CUTE_SUITE_H_*/
