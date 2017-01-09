#ifndef TWOPLANEFILTER__H
#define TWOPLANEFILTER__H 

/*
   Name        :  TwoPlaneFilter.h
   Author      :  Aaron
   Purpose     :  An extension of PlaneFilter.h that filters out all points
                  above one plane, and below a second.
                  The two planes are parallel and are separated by the distance
                  contained in `m_plane_separation`
*/

#include "PlaneFilter.h"

// Empirically tested
#define DEFAULT_PLANE_SEPARATION 0.1

template <typename PointT> class TwoPlaneFilter : public PlaneFilter<PointT> {

   using PlaneFilter<PointT>::m_plane;
   using PlaneFilter<PointT>::_inliers;
   using PlaneFilter<PointT>::_clipper;
   using PlaneFilter<PointT>::_cloud;

   public:

      typedef typename pcl::PointCloud<PointT>           Cloud;
      typedef typename pcl::PointCloud<PointT>::Ptr      CloudPtr;
      typedef typename pcl::PointCloud<PointT>::ConstPtr CloudConstPtr;

      double                           m_plane_separation;
      pcl::ModelCoefficients::Ptr      m_plane_two;


      TwoPlaneFilter(CloudPtr cloud, Mutex * mutex, double plane_separation = DEFAULT_PLANE_SEPARATION) :
         PlaneFilter<PointT>(cloud, mutex),
         m_plane_separation(plane_separation),
         m_plane_two(new pcl::ModelCoefficients())
      {
         _extract_two.setNegative(false);
      }


   private:

      pcl::ExtractIndices<PointT>      _extract_two;
      Eigen::Vector4f                  _plane_two_vector;

      virtual void _compute_separator()
      {
         PlaneFilter<PointT>::_compute_separator();

         _plane_two_vector << m_plane->values[0], m_plane->values[1], m_plane->values[2], 
                      (m_plane->values[3] + DEFAULT_FLOOR_HEIGHT + m_plane_separation);
      }

      virtual void _filter_cloud()
      {
         PlaneFilter<PointT>::_filter_cloud();

         _clipper.setPlaneParameters(_plane_two_vector);
         _inliers->indices.clear();
         _clipper.clipPointCloud3D(*_cloud, _inliers->indices);

         _extract_two.setIndices(_inliers);
         _extract_two.filterDirectly(_cloud);

      }

};

#endif // TWOPLANEFILTER__H
