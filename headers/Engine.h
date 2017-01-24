#ifndef ENGINE__H
#define ENGINE__H

/*
   Name        :  Engine.h
   Author      :  Aaron
   Purpose     :  This class implements an "Engine" that facilitates the operation of the robot.
                  It co-ordinates subclasses that process point clouds, and user input,
                  to tell the Gantry Robot where to go and what to do.
*/

#include "Logger.h"
#include "CameraFile.h"
#include "ClusterFinder.h"
#include "TwoPlaneFilter.h"

class Engine {

   // Public Typedefs
public:

   // Enumerate the different "states" the engine can be in.
   typedef enum {
      GOT_NOTHING, // We don't even have a point cloud
      GOT_CLOUD,   // We have a point cloud
      GOT_PLANE,   // We have a point cloud and know where the floor is
      GOT_CLUSTERS // We have located objects on the floor
   } EEngineState;

   typedef pcl::PointXYZRGBA Point;
   typedef pcl::PointCloud<Point> PointCloud;

private:

   PointCloud::Ptr      _cloud;
   Mutex *              _mutex;

public:

   // Camera object
   CameraBase<Point> *         m_camera;

   // Plane Filter object
   PlaneFilter<Point> *    m_plane_filter;

   // Cluster location
   ClusterFinder<Point>    m_cluster_finder;

   // Logging
   Logger                  m_logger;
   
   // "State" Enumeration
   EEngineState            m_state;

   // Base Constructor
   Engine(PointCloud::Ptr, Mutex *);

   // Visualization
   PointCloud::Ptr         m_vis_cloud;

   // Coefficients representing the floor's equation in 3-D space
   Eigen::Vector4f         m_floor_plane;

   /* OBJECTS TODO
    *
    * - Line projecting the claw on to the floor
    * - Points showing actual data (leave white)
    * - Larger points showing centroids of object surfaces
    *
    */

   /*
      Function       : get_image()
      Description    : Update the internal point cloud from the camera.
      Arguments      : None
      Returns        : void
   */
   void  get_image();

   /*
      Function       : get_floor()
      Description    : Attempt to identify the floor in 3-D space.
      Arguments      : None
      Returns        : void
   */
   void  get_floor();

   /*
      Function       : get_clusters()
      Description    : Attempt to identify the floor in 3-D space.
      Arguments      : None
      Returns        : void
   */
   void  get_clusters();

   /*
      Function       : get_movement()
      Description    : Incomplete
      Arguments      : None
      Returns        : void
   */
   void  get_movement();


};

#endif // ENGINE__H
