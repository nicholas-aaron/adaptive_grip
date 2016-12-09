#include <iostream>
#include <boost/thread/thread.hpp>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/openni_grabber.h>

extern "C" {
#include <stdio.h>
#include "readline/readline.h"
#include "readline/history.h"
}

#include "stringmanip.h"
#include "mutex.h"

// namespace stuff to reduce typing
using                                                    std::string;
using                                                    std::cout;
using                                                    std::endl;
using                                                    std::cerr;
using                                                    stringmanip::trim;
using                                                    stringmanip::split;
using                                                    stringmanip::arg_list;
typedef  pcl::PointXYZRGBA                               point_t;

// Forward Function Declarations
void                                                     prompt(const string);
int                                                      open_file(arg_list args);
int                                                      segment(arg_list args);
int                                                      remove_segment(arg_list args);
int                                                      capture_from_camera(arg_list input);
int                                                      adjust_seg_threshold(arg_list input);

int                                                      segment2(arg_list args);

// Program variables and objects
pcl::PointCloud<point_t>::Ptr                            m_cloud;                      
pcl::PointCloud<point_t>::ConstPtr                       m_cam_cloud;
bool                                                     quit = false;
boost::shared_ptr<pcl::visualization::PCLVisualizer>     viewer;

bool                                                     received_input = false;
char *                                                   cmdline_input;
string                                                   input;
bool                                                     point_cloud_loaded = false;
bool                                                     got_cam_cloud = false;

// Visualization Mutex
Mutex                                                    * upd_vis_mutex;
Mutex                                                    * input_mutex;
Mutex                                                    * cloud_mutex;

pcl::PointIndices::Ptr                                   plane_inliers (new pcl::PointIndices());

pcl::Grabber *                                           interface;



void get_input()
{
   cmdline_input = readline("SHELL > ");
   input = string(cmdline_input);
   input_mutex->lock();
   received_input = true;
   input_mutex->unlock();
}

void _update_cloud_callback(const pcl::PointCloud<point_t>::ConstPtr &cloud)
{
   cloud_mutex->lock();
   m_cam_cloud = cloud; // Is this wrong?
   got_cam_cloud = true;
   cloud_mutex->unlock();
}

int main(int argc, char ** argv)
{
   // Initialize program Variables and Objects

   // Mutexes
   upd_vis_mutex = new Mutex();
   input_mutex   = new Mutex();
   cloud_mutex   = new Mutex();

#define NOT_CONNECTING_INTERFACE
#ifndef NOT_CONNECTING_INTERFACE
   // Initialize grabber/Kinect interface. Register callback.
   interface = new pcl::OpenNIGrabber();

   // Note to self - you need to pass in an address to a function for boost::bind
   // boost::bind(__function_name__ ...) causes a mystery segfault
   // boost::bind(&__function_name__ ...) works.
   boost::function<void (const pcl::PointCloud<point_t>::ConstPtr &)>   callback_function =
      boost::bind(&_update_cloud_callback, _1); 
   
   // Register the callback.
   interface->registerCallback(callback_function);
#endif

   // "Shell is still running"
   bool quit = false;

   // Point Cloud that'll be used to visualize everything
   m_cloud = pcl::PointCloud<point_t>::Ptr(new pcl::PointCloud<point_t>);

   // Viewer
   viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("3D Viewer"));
   viewer->setBackgroundColor(0, 0, 0);

   while (!quit) {


      // ********************************************************************************
      //       Let user interact with the point cloud visualizer while the program
      //       waits for user input.
      // ********************************************************************************

      boost::thread  input_thread(get_input);

      // IMPORTANT: ANYTHING INVOLVING THE VIEWER NEEDS TO BE IN THE MAIN THREAD.
      // __ANYTHING__ ELSE CAN GO IN A SEPARATE THREAD.
      // DO NOT DO ANYTHING LIKE THIS OUTSIDE OF THE MAIN THREAD!
      while (!viewer->wasStopped())
      {
         viewer->spinOnce(100);

         boost::this_thread::sleep(boost::posix_time::microseconds(100000));

         input_mutex->lock();
         if (received_input) {
            input_mutex->unlock(); // :)
            break;
         }
         input_mutex->unlock();
      }

      input_thread.join(); // just to be safe..
      received_input = false;

      // ********************************************************************************
      //       Process Input. Do stuff to point cloud
      // ********************************************************************************
      
      if (trim(input).length() != 0)
      {

         arg_list args = split(input);
         string   cmd_name = args.front();
         args.pop_front();


         if (cmd_name == "load") {
            if (open_file(args) == 0) {
               viewer->addPointCloud<point_t>(m_cloud, "sample cloud");
               viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

               viewer->addCoordinateSystem();
            }
         } else if (cmd_name == "quit") {

            quit = true;  // will end the program

         } else if (cmd_name == "segment") {

            segment(args);

         } else if (cmd_name == "segment2") {

            segment2(args);

         } else if (cmd_name == "remove") {

            remove_segment(args);

         } else if (cmd_name == "camera") {

            capture_from_camera(args);

         } else {

            cout << "Command not recognized." << endl;;
         }


      }

      free(cmdline_input); // free the pointer
   }
   return 0;
}


int segment_camera(arg_list input) {
   return 0;
}


int segment(arg_list input)
{
   if (!point_cloud_loaded) {
      cerr << "No point cloud has been loaded! Not segmenting.." << endl;
      return -1;
   }

   pcl::ModelCoefficients::Ptr         coefficients (new pcl::ModelCoefficients());
   pcl::SACSegmentation<point_t>       seg;
   
   // Optional? TODO investigate
   seg.setOptimizeCoefficients(true);

   // Configure segmentation object.
   seg.setModelType(pcl::SACMODEL_PLANE);
   seg.setMethodType (pcl::SAC_RANSAC);
   seg.setMaxIterations (1000);
   seg.setDistanceThreshold (0.01);
 
   // Perform Segmentation
   seg.setInputCloud(m_cloud);
   seg.segment(*plane_inliers, *coefficients);

   std::cout << "Point Cloud representing plane: " << plane_inliers->indices.size() << " points, out of " << m_cloud->points.size() << " points." << endl;
   if (plane_inliers->indices.size() == 0) {
      return -1;
   } else {

      // Colour points in the plane.
      for (std::vector<int>::const_iterator it = plane_inliers->indices.begin(); it != plane_inliers->indices.end(); ++it)
      {
         m_cloud->points[*it].r = 255;
         m_cloud->points[*it].g = 0;
         m_cloud->points[*it].b = 0;
      }

      // Update the cloud, we've changed colors.
      viewer->updatePointCloud(m_cloud, "sample cloud");
   }

   return 0;
}

// 1. Segment.
// 2. Draw Normal to Plane if found.
int segment2(arg_list input)
{
   if (!point_cloud_loaded) {
      cerr << "No point cloud has been loaded! Not segmenting.." << endl;
      return -1;
   }

   pcl::ModelCoefficients::Ptr      floor_coef   (new pcl::ModelCoefficients());
   pcl::SACSegmentation<point_t>    seg;

   // Configure segmentation object
   seg.setOptimizeCoefficients(true);
   seg.setModelType(pcl::SACMODEL_PLANE);
   seg.setMethodType(pcl::SAC_RANSAC);
   seg.setMaxIterations(1000);
   seg.setDistanceThreshold(0.01);

   seg.setInputCloud(m_cloud);
   seg.segment(*plane_inliers, *floor_coef);

   std::cout << "Point Cloud representing plane: " << plane_inliers->indices.size() << " points, out of " << m_cloud->points.size() << " points." << endl;

   // TODO write this operator somewhere
   cout << "Plane representation: (x, y, z, d) = (" << floor_coef->values[0] << ", " << floor_coef->values[1] << ", " << floor_coef->values[2] << ", " << floor_coef->values[3] << ")" << endl;

   // Draw normal vector?
   //
   // Add first plane?

   viewer->addPlane(*floor_coef, "plane_one");

   // Add a second plane slightly above the first plane
   //
   // Add a third plane slightly below the first plane


   return -1;

}

int open_file(arg_list input)
{
// string filename = input.front();
// const string filename = "/Users/amnicholas/Documents/ELEC490/adaptive_grip_recent/vision/pcl_segmentation/table_scene_lms400.pcd"; // TODO FOR NOW
   const string filename = "/Users/amnicholas/Documents/ELEC490/adaptive_grip_recent/data/saved_kinect_pcd.pcd"; // TODO FOR NOW
   cout << "Opening: " << filename << endl;

   if (pcl::io::loadPCDFile<point_t>(filename, *m_cloud) == -1) {
      cout << "Could not open: <" << filename << ">!" << endl;
      return -1;
   } else {

      // Go through the entire cloud and make evertyhing white..
      cerr << "Warning: colouring everything white.." << endl;
      for (pcl::PointCloud<point_t>::iterator it = m_cloud->begin(); it != m_cloud->end(); ++it)
      {
         (*it).r = 255;
         (*it).g = 255;
         (*it).b = 255;
      }
      
      // If we got the point cloud, set point_cloud_loaded = true.
      point_cloud_loaded = true;

      return 0;
   }
}

int remove_segment(arg_list input)
{
   pcl::ExtractIndices<point_t>     extract;

   extract.setIndices(plane_inliers);
   extract.setNegative(true);

   extract.filterDirectly(m_cloud);

   viewer->updatePointCloud(m_cloud, "sample cloud");

   return 1;
}


// Capture a point cloud from the camera and display it
int capture_from_camera(arg_list input) {
   
   cloud_mutex->lock();
   got_cam_cloud = false;
   cloud_mutex->unlock();

   bool last = false;

   interface->start();

   while (!last) 
   {
      cloud_mutex->lock();
      if (got_cam_cloud == true) {
         last = got_cam_cloud;
      }
      cloud_mutex->unlock();
   }

   cout << "Got a point cloud from the camera." << endl;

   *m_cloud = *m_cam_cloud;

// viewer->removePointCloud("sample cloud");
// viewer->addPointCloud(m_cloud, "sample cloud");
   viewer->updatePointCloud(m_cloud, "sample cloud");

   point_cloud_loaded = true;

   interface->stop();

   return 0;
}

int adjust_seg_threshold(arg_list input) {
   return 0;
}




