#ifndef CUTE_EXPECT_H_
#define CUTE_EXPECT_H_
#include "cute_base.h"
#include "cute_test.h"
namespace cute{
	template <typename ExpectedException>
	struct cute_expect{
		test theTest;
		std::string filename;
		int  lineno;
		cute_expect(test const &t,char const *file,int line)
		:theTest(t), filename(file), lineno(line){}
		void operator()(){
			try{
				theTest();
				throw test_failure(what(),filename.c_str(),lineno);
			} catch(ExpectedException &) {
			}
		}
		std::string what() const{
			return theTest.name() + " expecting " 
			       + test::demangle(typeid(ExpectedException).name());
		}
	};
}
#define CUTE_EXPECT(tt,exc) cute::test(cute::cute_expect<exc>(tt,__FILE__,__LINE__),tt.name())

#define ASSERT_THROWSM(msg,code,exc) \
	do { \
		try { \
			(code) ; \
			FAILM(#msg); \
		} catch(exc &e){} \
	} while(0) 
#define ASSERT_THROWS(code,exc) ASSERT_THROWSM(" expecting " #code " to throw " #exc,code,exc)

#endif /*CUTE_EXPECT_H_*/
