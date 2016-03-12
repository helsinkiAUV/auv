#include <iostream>
#include "navigation.h"

int main ()
{
	Coord current(70, 24.9);
    Coord target(1, 25.0);
	Coord start(0, 25.0);

//	int nh;
//	double cte, dist;
//	bool inForbZone;
//
//	newBearing(start, target, current, 1E4, 1E5, nh, dist, cte, inForbZone);
//
//	std::cout << nh << std::endl;
//	std::cout << dist/1000 << std::endl;
//	std::cout << cte/1000 << std::endl;
//	std::cout << inForbZone << std::endl;

//	double val = angleBetweenBearings(10*M_PI/180, 358*M_PI/180) * 180 / M_PI;
//	std::cout << val << std::endl;

	Coord dest = current.destination(1, 100E3);
	std::cout << dest.latd << std::endl;
	std::cout << dest.lond << std::endl;

	return 0;
}


