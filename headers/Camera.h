#ifndef CAMERA__H
#define CAMERA__H

#include <boost/thread/thread.hpp>
#include <pcl/io/openni_grabber.h> 
#include <pcl/common/common_headers.h>
#include "common_defs.h"
#include "Mutex.h"
#include "CameraBase.h"


template <typename PointT> class Camera : public CameraBase<PointT> { 

   typedef typename pcl::PointCloud<PointT>              Cloud;
   typedef typename pcl::PointCloud<PointT>::Ptr         CloudPtr;
   typedef typename pcl::PointCloud<PointT>::ConstPtr   CloudConstPtr;

public:

// CloudPtr    m_cloud;

   // Constructor
   Camera(CloudPtr cloud, Mutex * mutex) :
      CameraBase<PointT>(cloud),
      _cloud_mutex(mutex)
   {

      // Note to self - you need to pass in an address to a function for boost::bind
      // boost::bind(__function_name__ ...) will segfault and not tell you why
      // boost::bind(&__function_name__ ...) is the correct call
      boost::function<void (const typename pcl::PointCloud<PointT>::ConstPtr &)>   callback_function =
         boost::bind(&Camera::_update_cloud_callback, this, _1); 
      
      // Instantiate the interface and register the callback.
      _interface = new pcl::OpenNIGrabber();
      _interface->registerCallback(callback_function);
   }

   // 0 if good, non-zero if bad
   int   retrieve()
   {
      // Acquire the cloud mutex before we modify the cloud.
      _cloud_mutex->lock();
      _interface->start();

      // Wait until the Kinect has picked up a new cloud.
      while (!_retrieved_cloud) { 
         boost::this_thread::sleep(boost::posix_time::microseconds(100));
      }

      // Modify the cloud, and release the Mutex.
      *(this->m_cloud) = *_camera_cloud; // quirk with template inheritance, need to access superclass members with this->
      _interface->stop();
      _cloud_mutex->unlock();

      _retrieved_cloud = false;
      
      return SUCCESS;
   }



private:

   void                             _update_cloud_callback(const typename pcl::PointCloud<PointT>::ConstPtr &cloud) 
   {
      _cloud_mutex->lock();
      _camera_cloud = cloud; // Is this wrong? A: nope
      _retrieved_cloud = true;
      _cloud_mutex->unlock();
   }


   CloudConstPtr                    _camera_cloud;
   Mutex *                          _cloud_mutex;
   bool                             _retrieved_cloud;
   pcl::Grabber *                   _interface;

};

#endif // CAMERA__H
