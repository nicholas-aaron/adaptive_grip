#include "SurfaceMap.h"


int
SurfaceMap::convex_hull()
{
	int n = coordinates.size();
	int k = 0;

///std::vector<Coordinate> H(n*2);
	hull.resize(n*2);

	// Sort points lexicographically - works because we defined "_Coordinate::operator <"
	sort(coordinates.begin(), coordinates.end());

	// Build lower hull
	for (int i = 0; i < n; ++i)
	{
		while (k >= 2 && cross(hull[k-2], hull[k-1], coordinates[i]) <= 0) k--;
		hull[k++] = coordinates[i];
	}

	// Build upper hull
	for (int i = (n - 2), t = (k+1); i >= 0; i--) {
		while (k >= t && cross(hull[k-2], hull[k-1], coordinates[i]) <= 0) k--;
		hull[k++] = coordinates[i];
	}

///hull.resize(k-1);
///return hull;

	return hull.size();
}

float 
SurfaceMap::cross(const Coordinate &O, const Coordinate &A, const Coordinate &B)
{
	return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
}

int 
SurfaceMap::initialize(PointCloud::Ptr surface, const Point & centroid, const Calibration & cal)
{
	typedef PointCloud::const_iterator		Iterator;
	Point 											difference;
	float												x_diff;
	float												y_diff;

	for (Iterator i = surface->begin(); i != surface->end(); ++i)
	{
		PCLUtils::subtractXYZ(*i, centroid, difference);

		x_diff = PCLUtils::dot_double_normalize(difference, cal.x_vector);
		y_diff = PCLUtils::dot_double_normalize(difference, cal.y_vector);

		coordinates.push_back(Coordinate(x_diff, y_diff));
	}

	return SUCCESS;
}


