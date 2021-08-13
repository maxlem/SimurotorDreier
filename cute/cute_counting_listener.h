#ifndef CUTE_COUNTING_LISTENER_H_
#define CUTE_COUNTING_LISTENER_H_
#include "cute_listener.h"
namespace cute{
	template <typename Listener=null_listener>
	struct counting_listener:Listener{
		counting_listener()
		:Listener()
		,numberOfTests(0),successfulTests(0),failedTests(0),errors(0),numberOfSuites(0){}
	
		counting_listener(Listener const &s)
		:Listener(s)
		,numberOfTests(0),successfulTests(0),failedTests(0),errors(0),numberOfSuites(0){}

		void begin(suite const &s, char const *info){
			++numberOfSuites;
			Listener::begin(s,info);
		}
		void start(test const &t){
			++numberOfTests;
			Listener::start(t);
		}
		void success(test const &t,char const *msg){
			++successfulTests;
			Listener::success(t,msg);
		}
		void failure(test const &t,test_failure const &e){
			++failedTests;
			Listener::failure(t,e);
		}
		void error(test const &t,char const *what){
			++errors;
			Listener::error(t,what);
		}
		int numberOfTests;
		int successfulTests;
		int failedTests;
		int errors;
		int numberOfSuites;
	};
}
#endif /*CUTE_COUNTING_LISTENER_H_*/
