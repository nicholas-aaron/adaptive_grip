#ifndef CAMERAFILE__H
#define CAMERAFILE__H

/*
   Name        :  CameraFile.h
   Author      :  Aaron
   Purpose     :  An extension of CameraBase used to retrive point clouds from
                  a file, instead of a Kinect. 
                  The typename PointT determines what `kind` of points should be 
                  retrieved from file (can be pcl::PointXYZ, 
                  pcl::PointXYZRGB, pcl::PointXYZRGBA, etc..)

                  Note that this is a templated class so functions have to be 
                  implemented where they are declared.
*/

#include <iostream>
#include "Camera.h"
#include <pcl/io/pcd_io.h>
#include <string>

template <typename PointT> class CameraFile: public CameraBase<PointT> {

   typedef typename pcl::PointCloud<PointT>              Cloud;
   typedef typename pcl::PointCloud<PointT>::Ptr         CloudPtr;
   typedef typename pcl::PointCloud<PointT>::ConstPtr   CloudConstPtr;

   public:

   // Constructor.
   // Takes in a Mutex along with a point cloud, to make sure the cloud isn't 
   // being modified or accessed elsewhere while it's being overwritten.
   CameraFile(CloudPtr cloud, Mutex * mutex) :
      CameraBase<PointT>(cloud),
      _cloud_mutex(mutex)
   {}

   /*
      Function       : retrieve()
      Description    : Updates m_cloud from file.
      Returns        : Returns SUCCCESS if read from file was successful,
                       FAILURE otherwise.
   */
   int retrieve()
   {
      _cloud_mutex->lock();

      // TODO get this filename from the preprocessor!
      const std::string filename = "/Users/amnicholas/Documents/ELEC490/adaptive_grip_recent/data/saved_kinect_pcd.pcd";

      if (pcl::io::loadPCDFile<PointT>(filename, *(this->m_cloud)) == -1) {
         std::cerr << "Could not open: <" << filename << ">!" << std::endl;
         _cloud_mutex->unlock();
         return FAILURE;
      }

      for (typename pcl::PointCloud<PointT>::iterator it = this->m_cloud->begin();
           it != this->m_cloud->end(); 
           ++it)
      {
         (*it).r = 255;
         (*it).g = 255;
         (*it).b = 255;
      }

      _cloud_mutex->unlock();
      return SUCCESS;
   }

   private:

   // Pointer to cloud mutex
   Mutex * _cloud_mutex;
};
#endif
