#ifndef ECLIPSE_LISTENER_H_
#define ECLIPSE_LISTENER_H_
#include "ostream_listener.h"
#include <iostream>
#include <iterator>
#include <algorithm>
namespace cute {

	class eclipse_listener 
	{
	protected:
		struct blankToUnderscore{
            char operator()(char in){
			if (in == ' ') return '_';
			return in;
		}
        };
		std::string maskBlanks(const std::string &in) {
			std::string result;
			std::transform(in.begin(),in.end(),std::back_inserter(result),blankToUnderscore());
			return result;
		}
	public:
		eclipse_listener() {}
		void start(test const &t){
			std::cout << "#starting " <<t.name()<< std::endl;
		}
		
		void begin(suite const &t,char const *info){
			std::cout << "#beginning " << info << " " << t.size() << std::endl;
		}
		void end(suite const &t, char const *info){
			std::cout << "#ending " << info << std::endl;
		}
		void success(test const &t, char const *msg){
			std::cout << "#success " <<  maskBlanks(t.name()) <<" " << msg<< std::endl;
		}
		void failure(test const &t,test_failure const &e){
			std::cout << "#failure " << maskBlanks(t.name()) << " " << e.filename << ":" << e.lineno << " " <<e.reason << std::endl;
		}
		void error(test const &t, char const *what){
			std::cout << "#error " << maskBlanks(t.name()) << " " << what << std::endl;
		}
	};
}
#endif /*ECLIPSE_LISTENER_H_*/
