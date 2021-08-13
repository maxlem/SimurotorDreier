#ifndef CUTE_REPEATED_TEST_H_
#define CUTE_REPEATED_TEST_H_
#include "cute_test.h"
namespace cute{
	struct repeated_test {
		repeated_test(test const &t,unsigned int n):theTest(t),repetitions(n){}
		void operator()(){
			for (unsigned int i=0;i<repetitions;++i){
				theTest();
			}
		}
		test theTest;
		const unsigned int repetitions;
	};
}
#define CUTE_REPEAT(aTestFunction,n) cute::test(cute::repeated_test((aTestFunction),(n)),#aTestFunction " " #n " times repeated")
#define CUTE_REPEAT_TEST(aTestObject,n) cute::test(cute::repeated_test((aTestObject),(n)),aTestObject.name()+" " #n " times repeated")
#endif /*CUTE_REPEATED_TEST_H_*/
