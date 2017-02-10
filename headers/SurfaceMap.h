#include "common_defs.h"
#include "PCLUtils.h"
#include "Calibration.h"
#include <algorithm>
#include <vector>

class SurfaceMap {

public:

	typedef struct _Coordinate {
		float x, y;
		bool operator <(const _Coordinate &p) const {
			return x < p.x || (x == p.x && y < p.y);
		}
		_Coordinate(float _x, float _y) : x(_x), y(_y) {}

		_Coordinate(const _Coordinate & copy) :
			x(copy.x), y(copy.y) {}

		_Coordinate() :
			x(0.0), y(0.0) {}

	} Coordinate;

	// 2D cross product of OA and OB vectors.
	float cross(const Coordinate &O, const Coordinate &A, const Coordinate &B);

	// Returns a vector of points forming a convex hull.
	int	convex_hull();

	std::vector<Coordinate> coordinates;
	std::vector<Coordinate> hull;
	

	typedef pcl::PointXYZRGBA 									Point;
	typedef pcl::PointCloud<Point>							PointCloud;

	// Build a surface map from a Kinect scan.
	int initialize(PointCloud::Ptr surface, const Point & centroid, const Calibration & cal);

};
