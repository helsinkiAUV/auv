#include "timeString.h"

std::string getTimeString()
{
	const int strSize = 15;
	char timeStr[strSize];
	tm* timeinfo;
	time_t rawtime;
	time(&rawtime);
	timeinfo = gmtime(&rawtime);

	strftime(timeStr, strSize, "%Y%m%d%H%M%S", timeinfo);

	return std::string(timeStr);
}

