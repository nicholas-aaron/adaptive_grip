#ifndef PLANEFILTER__H
#define PLANEFILTER__H

/*
   Name        :  PlaneFilter.h
   Author      :  Aaron
   Purpose     :  A class used to "filter" all points in a given plane out of 
                  a point cloud.
                  For example, this class can be used to remove all points 
                  representing the floor from a point cloud captured from
                  the Gantry Robot's Kinect.

                  The typename PointT specified the type of points in the point
                  cloud that a PlaneFilter object is working with.
                  
                  Note that this is a templated class so functions have to be 
                  implemented where they are declared.
*/

#include <iostream>
#include <pcl/common/common_headers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/plane_clipper3D.h> // Clip plane
#include "common_defs.h"

// Points 0.05 (5cm) above the floor and closer are removed, by default.
#define DEFAULT_FLOOR_HEIGHT 0.05

template <typename PointT> class PlaneFilter {

public:

   // Convenience
   typedef typename pcl::PointCloud<PointT>           Cloud;
   typedef typename pcl::PointCloud<PointT>::Ptr      CloudPtr;
   typedef typename pcl::PointCloud<PointT>::ConstPtr CloudConstPtr;

   // Coefficients of the plane that was detected.
   pcl::ModelCoefficients::Ptr      m_plane;

/*
   Function       : filter_plane()
   Description    : Tries to detect a plane in m_cloud. If successfull, removes
                    the points within `_floor_extract_height` of that plane.
                    `_floor_extract_height` is set to DEFAULT_FLOOR_HEIGHT 
                    by default.
   Returns        : SUCCESS, if a plane was detected and its corresponding 
                    points were removed.
                    FAILURE, if a plane was not detected.
*/
   int filter_plane() {

      if (_downsample_factor > 0) {
         // TODO optionally downsample...
      }

      _seg.segment(*_inliers, *m_plane); 

      // TODO this check could be better.. maybe check if 15% of the points
      // in the point cloud were removed?
      if (_inliers->indices.size() == 0) {
         std::cerr << "PlaneFilter::filter_plane() - no plane inliers!" << std::endl;
         return FAILURE;
      } 


      else {
         // Acquire the cloud's mutex while we're in this scope
         Mutex::ScopedLock(*_cloud_mutex);

         _compute_separator();
         _filter_cloud();

         return SUCCESS;
      }


   }

/*
   Function       : get_inliers()
   Description    : Tries to detect a plane in m_cloud. If successfull, removes
                    the points within `_floor_extract_height` of that plane.
                    `_floor_extract_height` is set to DEFAULT_FLOOR_HEIGHT 
                    by default.
   Returns        : A reference to the std::vector<int> of indices that were
                    removed from the last point cloud.
*/
   const std::vector<int> & get_inliers() const {
      return _inliers->indices;
   }

   PlaneFilter(CloudPtr cloud, Mutex * mutex) :
      _cloud(cloud),
      _cloud_mutex(mutex),
      m_plane(new pcl::ModelCoefficients()),
      _floor_extract_height(DEFAULT_FLOOR_HEIGHT),
      _clipper(_plane_vector),
      _inliers(new pcl::PointIndices())
   {
      _downsample_factor = -1;

      // default settings..
      _seg.setOptimizeCoefficients(true);
      _seg.setModelType(pcl::SACMODEL_PLANE);
      _seg.setMethodType(pcl::SAC_RANSAC);
      _seg.setMaxIterations(1000);
      _seg.setDistanceThreshold(0.01);
      _seg.setInputCloud(_cloud);

      // extractor settings..
//    _extract.setIndices(_inliers->indices);
      _extract.setNegative(true);
   } 

/*
   Function       : get_plane_coefficients()
   Description    : Returns a Vector4f (basically double[4]) representing the 
                    coefficients of the detected plane.
   Returns        : See description
*/
   Eigen::Vector4f                        get_plane_coefficients()
   {
      return _plane_vector;
   }

protected:

   CloudPtr                               _cloud;
   int                                    _downsample_factor;
   Mutex *                                _cloud_mutex;
   pcl::SACSegmentation<PointT>           _seg;
   pcl::PointIndices::Ptr                 _inliers;
   pcl::ExtractIndices<PointT>            _extract;
   pcl::PlaneClipper3D<PointT>            _clipper;
   Eigen::Vector4f                        _plane_vector;
   double                                 _floor_extract_height;


   virtual void _compute_separator()
   {
      _plane_vector << m_plane->values[0], m_plane->values[1], m_plane->values[2], 
                   (m_plane->values[3] + DEFAULT_FLOOR_HEIGHT);
   }

   virtual void _filter_cloud()
   {
      _clipper.setPlaneParameters(_plane_vector);
      _inliers->indices.clear();
      _clipper.clipPointCloud3D(*_cloud, _inliers->indices);
      _extract.setIndices(_inliers);
      _extract.filterDirectly(_cloud);
   }
};

#endif
