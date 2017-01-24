#ifndef CLUSTERFINDER__H
#define CLUSTERFINDER__H

/*
   Name        :  ClusterFinder.h
   Author      :  Aaron
   Purpose     :  A class used to find Euclidean clusters within a point cloud.
                  The typename PointT specified the type of points in the point
                  cloud that a PlaneFilter object is working with.
                  
                  Note that this is a templated class so functions have to be 
                  implemented where they are declared.
*/

#include <iostream>
#include "common_defs.h"
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>

#define CF_DEFAULT_VOXELSIZE 0.03f, 0.03f, 0.03f
#define CF_DEFAULT_TOLERANCE 0.20 // 20cm
#define CF_DEFAULT_MIN_CLUSTERSIZE 100
#define CF_DEFAULT_MAX_CLUSTERSIZE 25000

template <typename PointT> class ClusterFinder {
   public:

   typedef typename pcl::PointCloud<PointT>           Cloud;
   typedef typename pcl::PointCloud<PointT>::Ptr      CloudPtr;
   typedef typename pcl::PointCloud<PointT>::ConstPtr CloudConstPtr;

   // Constructor
   ClusterFinder(CloudPtr cloud, Mutex * mutex) :
      _input_cloud(cloud),
      _cloud_mutex(mutex),
      _tree(new pcl::search::KdTree<PointT>),
      _ds_cloud(CloudPtr(new Cloud))
   {
//    _ds_filter.setInputCloud(_input_cloud);
//    _ds_filter.setLeafSize(CF_DEFAULT_VOXELSIZE);

      // Euclidean cluster extraction settings
      _ece.setClusterTolerance(CF_DEFAULT_TOLERANCE);
      _ece.setMinClusterSize(CF_DEFAULT_MIN_CLUSTERSIZE);
      _ece.setMaxClusterSize(CF_DEFAULT_MAX_CLUSTERSIZE);
      _ece.setSearchMethod(_tree);
      _ece.setInputCloud(_input_cloud);
   }


   // A vector of vectors.
   // Each element of m_clusters is a pcl::PointIndices with all the indices of m_cloud
   // corresponding to a specific cluster.
   std::vector<pcl::PointIndices>                     m_clusters;

   
/*
   Function       : find_clusters()
   Description    : Try to identify clusters within a point clud.
   Returns        : The number of clusters identified, if successful.
                    A negative nubmer, if unsuccessful.
*/
   int find_clusters()
   {
      std::vector<int> nan_indices;
      pcl::removeNaNFromPointCloud(*_input_cloud, *_input_cloud, nan_indices);
      // Downsample
      _downsample();

      // Construct the 3D-Tree
      _tree->setInputCloud(_input_cloud);

      // Extract
      _ece.extract(m_clusters);
      return m_clusters.size();
   }

   private:

   int _downsample()
   {
      // TODO
      return SUCCESS;
   }

   CloudPtr                                           _input_cloud;
   CloudPtr                                           _ds_cloud; // The downsampled point-cloud
   int                                                _downsample_factor;
   Mutex *                                            _cloud_mutex;
// pcl::VoxelGrid<Cloud>   _ds_filter;

   pcl::EuclideanClusterExtraction<PointT>            _ece;
   typename pcl::search::KdTree<PointT>::Ptr          _tree;


};

#endif // CLUSTERFINDER__H
