#ifndef CUTE_TEST_INCARNATE_H_
#define CUTE_TEST_INCARNATE_H_
#include "cute_test.h"
// idea blatantly stolen from Aeryn
namespace cute {
	template <typename TestFunctor>
	struct test_incarnate {
		void operator()(){
			TestFunctor()();
		}
	};
	// TODO: check if there are problems with references.
	template <typename TestFunctor, typename ContextObject>
	struct test_incarnate_with_context {
		test_incarnate_with_context(ContextObject context):theContext(context)
		{}
		void operator()(){
			TestFunctor t(theContext);// wouldn't create temporary to call with ()()
			t();
		}
		ContextObject theContext;
	};
	template <typename TestFunctor,typename ContextObject>
	test make_incarnate_with_context(ContextObject obj){
		return test(test_incarnate_with_context<TestFunctor,ContextObject>(obj),test::demangle(typeid(TestFunctor).name()));
	}
}
#define CUTE_INCARNATE(TestFunctor) cute::test(cute::test_incarnate<TestFunctor>(),cute::test::demangle(typeid(TestFunctor).name()))
#define CUTE_INCARNATE_WITH_CONTEXT(TestFunctor,contextObject) cute::make_incarnate_with_context<TestFunctor>(contextObject)
#endif /*CUTE_TEST_INCARNATE_H_*/
