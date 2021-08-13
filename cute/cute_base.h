#ifndef CUTE_BASE_H_
#define CUTE_BASE_H_
#include <string>
namespace cute{
	struct test_failure {
		std::string reason;
		std::string filename;
		int lineno;
	
		test_failure(std::string const &r,char const *f, int line)
		:reason(r),filename(f),lineno(line)
		{ 	}
		char const * what() const { return reason.c_str(); }
	};
}
#define ASSERTM(msg,cond) if (!(cond)) throw cute::test_failure((msg),__FILE__,__LINE__)
#define ASSERT(cond) ASSERTM(#cond,cond)
#define FAIL() ASSERTM("FAIL()",false)
#define FAILM(msg) ASSERTM(msg,false)

#endif /*CUTE_BASE_H_*/
