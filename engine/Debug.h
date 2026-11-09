#pragma once
#include <iostream>
#include <string>
#include <sstream>
#undef ERROR
namespace engine
{
	enum DebugColor
	{
		DBG_BLACK = 0,
		DBG_RED = 1,
		DBG_GREEN = 2,
		DBG_YELLOW = 3,
		DBG_BLUE = 4,
		DBG_PURPLE = 5,
		DBG_LIGHT_BLUE = 6,
		DBG_WHITE = 7,
	};
	const DebugColor DEFAULT_COLOR = DBG_WHITE;

	class DebugStream
	{
	public:
		DebugStream(std::string prefix, DebugColor color)
			: mPrefix(prefix), mColor(color), mMoved(false) {
		}
		DebugStream(DebugStream&& other) noexcept
			: mBuffer(std::move(other.mBuffer)),
			mColor(other.mColor),
			mPrefix(std::move(other.mPrefix)),
			mMoved(false) {
			other.mMoved = true;
		}

		std::string indexToColor(const unsigned int& index)
		{
			switch (index)
			{
			case 0:
				return "30m";
			case 1:
				return "31m";
			case 2:
				return "32m";
			case 3:
				return "33m";
			case 4:
				return "34m";
			case 5:
				return "35m";
			case 6:
				return "36m";
			case 7:
				return "37m";
			default:
				return "37m";
			}
		}

		DebugStream(const DebugStream&) = delete;
		DebugStream& operator=(const DebugStream&) = delete;

		//Add things to the buffer
		template<typename T>
		DebugStream& operator<<(const T& val)
		{
			mBuffer << val;
			return *this;
		}

		~DebugStream();

	private:
		std::ostringstream mBuffer;
		DebugColor mColor;
		std::string mPrefix;
		bool mMoved;
	};

	class Debug
	{
	public:

		static Debug* createInstance();
		static Debug* instnace();
		static void removeInstance();

		static DebugStream log(DebugColor color = DBG_WHITE, std::string prefix = "[LOG]");
		static DebugStream success();
		static DebugStream warn();
		static DebugStream error();

	private:
		Debug() = default;
		~Debug() = default;

		static Debug* mpsInstance;
	};
}