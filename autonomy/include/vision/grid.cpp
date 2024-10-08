/*
  Support code for navigation grid. 
*/
#include "grid.hpp"
#include "../aurora/robot_base.h"
#include <cmath>

grid_square::grid_square()
{
  clear();
}
void grid_square::clear() 
{
	count=0;
	max=-10000;
	min=10000;
	sum=0;
	sumSquares=0;
	flags=0;
}

void grid_square::addPoint(float z)
{
	++count;
	sum+=z;
	sumSquares+=z*z;
	if(z<min)
	{
		min=z;
	}
	if(max<z)
	{
		max=z;
	}
}
float grid_square::getMean() const
{
	return sum/count;
}
float grid_square::getTrimmedMean() const
{
	if (count>2) {
		return (sum-max-min)/(count-2);
	}
	else {
		return getMean();
	}
}
float grid_square::getVariance() const
{
	return sumSquares/count-getMean()*getMean();
}

int compare(grid_square a, grid_square b)
{
	if(std::abs(a.getMean() - b.getMean())>5)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}




#include <vector>


