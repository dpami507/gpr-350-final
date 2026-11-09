#include "Debug.h"

namespace engine
{
	Debug* Debug::mpsInstance = nullptr;

	//Create static instance
	Debug* Debug::createInstance()
	{
		mpsInstance = new Debug();
		return mpsInstance;
	}
	//Get static instance
	Debug* Debug::instnace()
	{
		if (mpsInstance == nullptr)
		{
			createInstance();
		}

		return mpsInstance;
	}
	//Remove and cleanup static instance
	void Debug::removeInstance()
	{
		delete mpsInstance;
		mpsInstance = nullptr;
	}

	/*
	Logs to the console
	@param DebugColor: color for text to be
	@param prefix: prefix at begining of log default is "[LOG]"
	*/
	DebugStream Debug::log(DebugColor color, std::string prefix)
	{
		return DebugStream(prefix, color);
	}
	/*
	Logs a success to the console in yellow
	*/
	DebugStream Debug::success()
	{
		return DebugStream("[SUCC]", DBG_GREEN);
	}
	/*
	Logs a warning to the console in yellow
	*/
	DebugStream Debug::warn()
	{
		return DebugStream("[WARN]", DBG_YELLOW);
	}
	/*
	Logs an error to the console in red
	*/
	DebugStream Debug::error()
	{
		return DebugStream("[ERR]", DBG_RED);
	}

	//Print when destroyed
	DebugStream::~DebugStream()
	{
		//Print prefix then message
		std::cout 
			<< "\033[" 
			<< indexToColor(mColor)
		    << mPrefix << " " << mBuffer.str() 
			<< "\033[0m"
			<< "\n";
	}
}