#ifndef ENGINE3__H
#define ENGINE3__H

#include "Engine2.h"
#include "Mutex.h"
#include <pcl/io/openni_grabber.h>

class Engine3 : public Engine2 {

   using Engine2::Point;
   Mutex * mutex;

public:
   pcl::visualization::PCLVisualizer::Ptr vis_live;

   Engine3(
         pcl::visualization::PCLVisualizer::Ptr vis   = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("viewer")),
         Logger * logger                              = new Logger(),
         pcl::visualization::PCLVisualizer::Ptr _vis_live = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("live viewer"))):
      Engine2(vis, logger),
      vis_live(_vis_live)
   {} // use Engine2 constructor

   void moveTo(RobotPosition & position)
   {
      std::cout << "Entering Engine3::moveTo()" << std::endl;

      // In the main thread: register the Camera with a callback to update vis_live

      boost::function<void (const pcl::PointCloud<Point>::ConstPtr &)> callback_function
         = boost::bind(&Engine3::_live_cam_callback, this, _1);

//    pcl::PointCloud<Point>::Ptr live_cloud_ptr (new pcl::PointCloud<Point>);
//    vis_live->addPointCloud<Point>(live_cloud_ptr, "live_cloud");

      // Instantiate the interface and register the callback
      pcl::Grabber * interface = new pcl::OpenNIGrabber();
      interface->registerCallback(callback_function);
      interface->start();

      // Interesting, can't put &Engine2::move_to in a boost::bind
      boost::thread     move_to_thread(boost::bind(&Engine3::superclass_move_to, this, position));

//    while (!vis_live->wasStopped())  // This will freeze
//    {
//       mutex->lock();
//       vis_live->spinOnce(20);
//       mutex->unlock();
//    }

      move_to_thread.join();

      interface->stop();
   }
   
   void superclass_move_to(RobotPosition & pos)
   {
      Engine2::moveTo(pos);
   }

private:

   void _live_cam_callback(const pcl::PointCloud<Point>::ConstPtr &cloud)
   {
//    pcl::PointCloud<Point>::Ptr   live_cloud_ptr(new pcl::PointCloud<Point>::Ptr);
      mutex->lock();
      vis_live->updatePointCloud(cloud, "live_cloud");
      mutex->unlock();
   }
};

#endif // ENGINE3__H
