#ifndef CCLOUD__H
#define CCLOUD__H

#include "Mutex.h"
#include "ClusterFinder.h"

#include <pcl/point_cloud.h>

class CCloud {
public:

      typedef pcl::PointXYZRGBA        Point;
      typedef pcl::PointCloud<Point>   PointCloud;

      PointCloud::Ptr      cloud;
      bool                 initialized;
      PointCloud::Ptr      clusters;

      CCloud() {
         cloud.reset(new PointCloud());
         initialized    = false;
      }

      CCloud(PointCloud::Ptr _cloud) {
         cloud          = _cloud;
         initialized    = false;
      }

      int size() {
         return initialized ? clusters->size() : -1;
      }

      inline std::vector<Point>::iterator begin() // convenience
      {
         return clusters->begin();
      }

      inline std::vector<Point>::iterator end() // convenience
      {
         return clusters->end();
      }

      void find_clusters()
      {
         Mutex m; // TODO un-mutex this code
         ClusterFinder<Point> cf(cloud, &m);
         cf.find_clusters();
         clusters = cf.get_clusters();
      }
};

#endif // CCloud.h
