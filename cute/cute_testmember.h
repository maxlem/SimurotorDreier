#ifndef CUTE_TESTMEMBER_H_
#define CUTE_TESTMEMBER_H_
#include "cute_test.h"
#include <boost/bind.hpp>
namespace cute {
	template <typename TestClass>
	test makeMemberFunctionTest(TestClass &t,void (TestClass::*fun)(),char const *name){
		return test(boost::bind(fun,boost::ref(t)),test::demangle(typeid(TestClass).name())+"::"+name);
	}
	template <typename TestClass>
	test makeMemberFunctionTest(TestClass const &t,void (TestClass::*fun)()const,char const *name){
		return test(boost::bind(fun,boost::cref(t)),test::demangle(typeid(TestClass).name())+"::"+name);
	}
	template <typename TestClass,typename MemFun>
	struct incarnate_for_member_function {
		MemFun memfun;
		incarnate_for_member_function(MemFun f):memfun(f){}
		void operator()(){
			TestClass t;
			(t.*memfun)();
		}
	};
	template <typename TestClass, typename MemFun>
	test makeSimpleMemberFunctionTest(MemFun fun,char const *name){
		return test(incarnate_for_member_function<TestClass,MemFun>(fun),test::demangle(typeid(TestClass).name())+"::"+name);
	}
	template <typename TestClass,typename MemFun, typename Context>
	struct incarnate_for_member_function_with_context_object {
		MemFun memfun;
		Context context;
		incarnate_for_member_function_with_context_object(MemFun f,Context c)
		:memfun(f),context(c){}
		void operator()(){
			TestClass t(context);
			(t.*memfun)();
		}
	};
	template <typename TestClass, typename MemFun, typename Context>
	test makeMemberFunctionTestWithContext(Context c,MemFun fun,char const *name){
		return test(incarnate_for_member_function_with_context_object<TestClass,MemFun,Context>(fun,c),test::demangle(typeid(TestClass).name())+"::"+name);
	}
}
#define CUTE_MEMFUN(testobject,TestClass,MemberFunctionName) \
	cute::makeMemberFunctionTest(testobject,\
		&TestClass::MemberFunctionName,\
		#MemberFunctionName)
#define CUTE_SMEMFUN(TestClass,MemberFunctionName) \
	cute::makeSimpleMemberFunctionTest<TestClass>(\
		&TestClass::MemberFunctionName,\
		#MemberFunctionName)
#define CUTE_CONTEXT_MEMFUN(context_object,TestClass,MemberFunctionName) \
	cute::makeMemberFunctionTestWithContext<TestClass>(\
		context_object,\
		&TestClass::MemberFunctionName,\
		#MemberFunctionName)

#endif /*CUTE_TESTMEMBER_H_*/
