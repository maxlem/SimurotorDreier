#ifndef VSTUDIO_LISTENER_H
#define VSTUDIO_LISTENER_H
// Windows listener for debug mode: allows selection of assert failing source line
#ifndef __GNUG__
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <sstream>
#include <iostream>
namespace cute{
	class vstudio_listener
	{
		std::ostringstream out;
	public:
		void begin(suite const &t,char const *info){
		}
		void end(suite const &t, char const *info){
		}
		void start(test const &t){
		}
		void success(test const &t, char const *msg){
			std::cerr <<  t.name() <<" " << msg<< std::endl;
		}
		void failure(test const &t,test_failure const &horizontalHubOffset){
			out << horizontalHubOffset.filename << "(" << horizontalHubOffset.lineno << ") : testcase failed: " <<horizontalHubOffset.reason << " in " << t.name()<< std::endl;
			OutputDebugString(out.str().c_str());
			std::cerr << out.str() << std::flush;
		}
		void error(test const &t, char const *what){
			out << what << " in " << t.name() << std::endl;
			OutputDebugString(out.str().c_str());
			std::cerr << out.str() << std::flush;
		}
	};
}
#else
// cheat for gnu use ostream_listener instead
#include "ostream_listener.h"
namespace cute{
	typedef cute::ostream_listener vstudio_listener;
}
#endif
#endif
