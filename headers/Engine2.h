#include "PCLUtils.h"
#include "RobotExt.h"
#include "TwoPlaneFilter.h"
#include "Mutex.h"
#include "Camera.h"
#include "common_defs.h"
#include "Cluster.h"
#include "ClusterFinder.h"
#include "Logger.h"
#include <iostream>
#include <sstream>

// For cal type
#include <fstream>



using std::stringstream;
using pcl::visualization::PointCloudColorHandlerCustom;
using PCLUtils::addXYZ;
using PCLUtils::subtractXYZ;

class Engine2 {
public: 

   typedef pcl::PointXYZRGBA        Point;
   typedef pcl::PointCloud<Point>   PointCloud;

   private:

   PointCloud::Ptr   cam_cloud_;
   Mutex *           dummy_mutex_;

   // VIEWPORTS
   int      vp_calibration_clouds;
   int      vp_calibration_axes;
   int      vp_navigation;
   int      vp_workspace;

   Point                            origin_;
   Point                            origin_proj_;

   pcl::visualization::PCLVisualizer::Ptr viewer;

public:

   // TODO move
   typedef struct _WS_Object {
      float          x_position;
      float          y_position;
      float          observation_distance;
      int            id;
      float          distance_to(_WS_Object & o, Eigen::Vector4f & plane);

         static float fudge;
         static float distance_fudge;
         static int   current_id;


      _WS_Object(float x, float y, float obs_dist) :
         x_position(x), y_position(y), observation_distance(obs_dist)
      {
         id = current_id++;
      }

      // Copy constructor
      _WS_Object(const _WS_Object & copy) {
         x_position = copy.x_position;
         y_position = copy.y_position;
         observation_distance = copy.observation_distance;
         id         = copy.id;
      }

   } WSObject;


   std::vector<WSObject>   m_objects;

   bool add_object(float x, float y, const RobotPosition & observed_position) {
      
      // Go through m_objects..
      typedef std::vector<WSObject>::iterator Iterator;
      int count = 0;

      const float x_delta = x - observed_position.x;
      const float y_delta = y - observed_position.y;
      const float observation_distance = sqrt(x_delta*x_delta + y_delta*y_delta);

      if (observation_distance > WSObject::distance_fudge) {
         std::stringstream msg;
         msg << "add_object() : Ignoring the add at (" << x << ", " << y << "). Distance (" << observation_distance 
            << ") above fudge factor (" << WSObject::distance_fudge << ")";
         m_logger->log(msg);
      }

      for (Iterator i = m_objects.begin(); i != m_objects.end(); i++) {
         
         float x_diff = (*i).x_position - x;
         float y_diff = (*i).y_position - y;
         float distance = sqrt((x_diff*x_diff) + (y_diff*y_diff));
         std::stringstream msg;
         msg << "add_object() : (" << count++ << "): Distance = " << distance;
         m_logger->log(msg);

         if ( distance < WSObject::fudge ) {
            std::stringstream ss;
            ss << "add_object(" << x << ", " << y << ") failed - there is an object at (" << (*i).x_position << "," << (*i).y_position << ")." << endl;
            ss << "distance = " << distance << ", fudge = " << WSObject::fudge << "." << endl;              
            m_logger->log(ss);
            return false;
         }
      }

      std::stringstream ss;
      WSObject obj(x, y, observation_distance);
      m_objects.push_back(obj);
      ss << "Added object at (" << x << ", " << y << ")." << endl;
      m_logger->log(ss);
      return true;
      
   }


   bool remove_object(float x, float y) {
      m_logger->log("remove_object not implemented yet.");
      return false;
   }

   typedef struct {
      bool x_axis_calibrated;
      bool y_axis_calibrated;
//    bool robot_homed;
   } state_t;

   typedef union {
      state_t  state;
      char     mask;
   }  ustate_t;

   RobotExt *                       m_robot;
   bool                             m_got_floor_plane;
   Eigen::Vector4f                  m_floor_plane;
   ustate_t                         m_state;
   std::vector<Point>               m_clusters;
   Camera<Point>                    m_camera;
   Logger *                         m_logger;


   // TODO move to its own class/file
   typedef struct 
   {
      // Calibration
      float                            x_rpos_amt;
      float                            y_rpos_amt;
      Point                            x_vector;
      Point                            y_vector;
      
      
      // Could actually just guess this
//    float m_vis_x_max;
//    float m_vis_y_min;
//    float m_vis_y_max;
//    float m_vis_x_min;

      bool toFile(const std::string& filename)
      {
         using namespace std;
         ofstream outfile(filename.c_str());
         if (outfile) {
            outfile << x_rpos_amt << " " << y_rpos_amt << " "
               << x_vector.x << " " << x_vector.y << " " << x_vector.z << " "
               << y_vector.x << " " << y_vector.y << " " << y_vector.z << endl;
            return true;
         } else return false;
      }

      bool fromFile(const std::string& filename)
      {
         using namespace std;
         ifstream infile(filename.c_str());
         float x_vec_x, x_vec_y, x_vec_z;
         float y_vec_x, y_vec_y, y_vec_z;
         if (infile) {
            infile >> x_rpos_amt >> y_rpos_amt
               >> x_vec_x >> x_vec_y >> x_vec_z 
               >> y_vec_x >> y_vec_y >> y_vec_z;
            x_vector.x = x_vec_x;
            x_vector.y = x_vec_y;
            x_vector.z = x_vec_z;
            y_vector.x = y_vec_x;
            y_vector.y = y_vec_y;
            y_vector.z = y_vec_z;

            return true;
         } else return false;
      }

      std::string toString()
      {
         std::stringstream ss;
         ss << x_rpos_amt << " " << y_rpos_amt << " "
            << x_vector.x << " " << x_vector.y << " "<< x_vector.z << " "
            << y_vector.x << " " << y_vector.y << " "<< y_vector.z << endl;
         return ss.str();
      }

   } cal_t;

   cal_t                            m_cal;

   // TODO move to its own files
   typedef struct CCloud {
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

   } ClusterCloud ;

   // TODO Move to its own file
   bool validate_limits(const RobotPosition & pos) 
   {
      std::cout << "Implement validateLimits!!" << std::endl;
      return true;
   }

   bool calculate_robot_position(const Point & position, RobotPosition & new_pos)
   {
      Point vector_along_floor;
      PCLUtils::subtractXYZ(position, origin_proj_, vector_along_floor);

      float x_amt = PCLUtils::dot_double_normalize(vector_along_floor, m_cal.x_vector);
      float y_amt = PCLUtils::dot_double_normalize(vector_along_floor, m_cal.y_vector);

      new_pos.x += (-1.0 * x_amt * m_cal.x_rpos_amt);
      new_pos.y += (-1.0 * y_amt * m_cal.y_rpos_amt);

      stringstream ss;
      ss << "calculate_robot_position(): New robot position:" << endl;
      ss << new_pos;
      m_logger->log(ss);

      return validate_limits(new_pos);
   }

   void calibrate()
   {
      // Calibrate the X-axis, if necessary
      if (!m_state.state.x_axis_calibrated) {
         m_state.state.x_axis_calibrated = calibrate_axis(true, m_cal.x_rpos_amt, m_cal.x_vector);
         if (m_state.state.x_axis_calibrated) {
            m_logger->log("Successfully calibrated the x-axis.");
         }
      } else m_logger->log("The x-axis was already calibrated.");


      // Calibrate the Y-axis, if necessary
      if (!m_state.state.y_axis_calibrated) {
         m_state.state.y_axis_calibrated = calibrate_axis(false, m_cal.y_rpos_amt, m_cal.y_vector);
         if (m_state.state.y_axis_calibrated) {
            m_logger->log("Successfully calibrated the y-axis.");
         }
      } else m_logger->log("The y-axis was already calibrated.");


      // If both axes are calibrated, set up the viewer
      if (m_state.state.x_axis_calibrated && m_state.state.y_axis_calibrated)
      {
         m_logger->log("Displaying calibration vectors.");
         axis_view_setup(vp_calibration_axes);
      }
   }

   void load()
   {
      const std::string calFile = "/Users/amnicholas/Documents/ELEC490/adaptive_grip_recent/data/calibration.dat";
      if (m_cal.fromFile(calFile)) {
         cout << "Succesfully loaded file." << endl;
         cout << m_cal.toString() << endl;
         m_state.state.x_axis_calibrated = true;
         m_state.state.y_axis_calibrated = true;
         axis_view_setup(vp_calibration_axes);
      } else {
         cerr << "There was a problem loading the calibration file." << endl;
      }
   }

   bool calibrate_axis(bool x_yb, float & robot_pos_amt, Point & vector)
   {

      robot_pos_amt = 100.0;

      ClusterCloud   source, target;

      RobotPosition new_pos = m_robot->currentPos();
      if (x_yb) {
         new_pos.x = new_pos.x + robot_pos_amt;
      } else {
         new_pos.y = new_pos.y + robot_pos_amt;
      }

      if (!validate_limits(new_pos)) {
         stringstream ss("");
         ss << "calibrate_axis() : Illegal robot position while trying to calibrate.";
         m_logger->log(ss);
         return false;
      }

      // Load source cloud
      load_and_filter(source);
      add_cloud_to_viewer(source.cloud, "TargetCloud", vp_calibration_clouds, 255, 0, 0);


      // Move to position
      moveTo(new_pos);

      // Load target cloud
      load_and_filter(target);
      add_cloud_to_viewer(target.cloud, "SourceCloud", vp_calibration_clouds, 0, 0, 255);

      // ClusterDifference
      if (!clusterDifference(source, target, vector)) {
         stringstream ss;
         ss << "calibrate_axis() : clusterDifference() failed. Could not calibrate ";
         ss << (x_yb ? "x" : "y") << " axis." << endl;
         m_logger->log(ss);
         return false;
      }

      return true;
   }


   // This is only used for calibration, maybe worth moving?
   bool clusterDifference(ClusterCloud & source, ClusterCloud & target, Point & vector)
   {
      source.find_clusters();
      target.find_clusters();

      if (source.size() != 1) {
         stringstream ss;
         ss << "clusterDifference() : Source cloud had " << source.size() << " clusters." << endl;
         ss << "Returning false." << std::endl;
         m_logger->log(ss);
         return false;
      } else if (target.size() != 1) {
         stringstream ss;
         ss << "clusterDifference() : Target cloud had " << target.size() << " clusters." << endl;
         ss << "Returning false." << std::endl;
         m_logger->log(ss);
         return false;
      }

      Point source_cluster = source.clusters->points[0];
      Point target_cluster = target.clusters->points[0];
      PCLUtils::subtractXYZ(target_cluster, source_cluster, vector);

      return true;
   }


   int   locate(bool observe_only) 
   {
      // find clusters
      // rank them based on distance to origin
      // move to the closest one
      ClusterCloud current_view;

      typedef std::vector<Point>::iterator Iterator;
      Iterator        closest_cluster;

      // TODO CHECK THAT WE ARE CALIBRATED!
      if (!m_state.state.x_axis_calibrated || !m_state.state.y_axis_calibrated) {
         m_logger->log("locate() - Not calibrated. Aborting.");
         return -1;
      }

      load_raw(current_view);

      add_cloud_to_viewer(current_view.cloud, "CurrentCloud", vp_navigation);

      filter_floor(current_view);

      current_view.find_clusters();

      if (current_view.size() == 0) {
         m_logger->log("locate() - No clusters found. Not moving.");
         return -1;
      }

      else if (current_view.size() == 1) {
         m_logger->log("One cluster found. Moving..");
         closest_cluster = current_view.begin();

         RobotPosition     pos = m_robot->currentPos();
         calculate_robot_position(*closest_cluster, pos);
         add_object(pos.x, pos.y, m_robot->currentPos());
      }

      else {
         RobotPosition currentPos = m_robot->currentPos();

         stringstream ss;
         ss << "A total of " << current_view.size() << " clusters were found." << endl;
         m_logger->log(ss);

         float shortest_distance = -1.0;
         
         for (Iterator i = current_view.begin();
               i != current_view.end(); ++i)
         {

            Cluster<Point>    current_point_cluster(*i);
            Point   floorPoint  = current_point_cluster.get_plane_projection(m_floor_plane);

            // Get the norm distance between floorPoint and originProjection
            float distance = PCLUtils::distance(floorPoint, origin_proj_);

            RobotPosition pos = currentPos;
            calculate_robot_position(*i, pos);
            add_object(pos.x, pos.y, currentPos);


            if (shortest_distance < 0) {
               shortest_distance = distance;
               closest_cluster = i;
            } else {
               if (distance < shortest_distance) {
                  shortest_distance = distance;
                  closest_cluster = i;
               }
            }
         }
      }

      // This pretty much forces you to use PointXYZRGBA

      // Color everything, except the closest cluster, green
      for (Iterator i = current_view.begin(); i != current_view.end(); ++i)
      {
         if (i == closest_cluster) {
            (*i).r = 0;
            (*i).g = 255;
            (*i).b = 255;
         } else {
            (*i).r = 0;
            (*i).g = 255;
            (*i).b = 0;
         }
      }

      add_cluster_to_viewer(current_view.clusters, "CurrentCloud_Clusters", vp_navigation);

      if (observe_only) {
         m_logger->log("locate() returning - only observing.");
         return 0;
      }
 
      RobotPosition new_position = m_robot->currentPos();
      if (!calculate_robot_position(*closest_cluster, new_position))
      {
         m_logger->log("The calcualted RobotPosition is outside of robot limits! Not moving.");
         return -1;
      } else {
         moveTo(new_position);
         return 0;
     }

   }

   void load_raw(ClusterCloud & cc) {
      cc.cloud.reset(new PointCloud);
      m_camera.retrieve();
      *cc.cloud = *cam_cloud_;

      // Don't filter it
      cc.initialized = true;
   }

   void moveTo(RobotPosition & position)
   {
      m_robot->moveTo(position);
   }

   void filter_floor(ClusterCloud & cc) 
   {
      Mutex dumb_mutex; // TODO un-mutex this
      TwoPlaneFilter<Point>   tp(cc.cloud, &dumb_mutex);
      tp.filter_plane();

      if (!m_got_floor_plane) {
         m_logger->log("load_and_filter() - Updating floor coefficients.");

         m_floor_plane = tp.get_plane_coefficients();
         m_got_floor_plane = true;

         // TODO refactor
         Cluster<Point> originClust(origin_);
         origin_proj_ = originClust.get_plane_projection(m_floor_plane);
      }
   }


   inline void  load_and_filter(ClusterCloud & cc) 
   {
      load_raw(cc);
      filter_floor(cc);
   }

   void add_cloud_to_viewer(PointCloud::Ptr cloud, std::string name, int viewport)
   {
      viewer->removePointCloud(name);
      viewer->addPointCloud(cloud, name, viewport);
   }

   void add_cloud_to_viewer(PointCloud::Ptr cloud, std::string name, int viewport, int r, int g, int b)
   {
      PointCloudColorHandlerCustom<Point>    handler(cloud, r, g, b);
      viewer->removePointCloud(name);
      viewer->addPointCloud(cloud, handler, name, viewport);
   }

   void add_cluster_to_viewer(PointCloud::Ptr clusters, std::string name, int viewport, int r=255, int g=255, int b=255)
   {
      viewer->removePointCloud(name);
//    PointCloudColorHandlerCustom<Point>  handler(clusters, r, g, b);
      viewer->addPointCloud(clusters, name, viewport);
      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, name);

   }

   void axis_view_setup(int viewport)
   {

      viewer->removeShape("originLine", viewport);
      viewer->addLine(origin_, origin_proj_, "originLine", viewport);

      Point xDifference;
      addXYZ(origin_proj_, m_cal.x_vector, xDifference);
      viewer->removeShape("xRegAxis", viewport);
      viewer->addLine(origin_proj_, xDifference, 214, 0, 244, "xRegAxis", viewport);

      Point yDifference;
      addXYZ(origin_proj_, m_cal.y_vector, yDifference);
      viewer->removeShape("yRegAxis", viewport);
      viewer->addLine(origin_proj_, yDifference, 255, 165, 0, "yRegAxis", viewport);
   }


   /* Constructor */
   Engine2() :
      cam_cloud_(new PointCloud()),
      dummy_mutex_(new Mutex()),
      m_camera(cam_cloud_, dummy_mutex_),
      m_robot(new RobotExt("/dev/cu.usbserial")),
      m_logger(new Logger())
   {
      m_got_floor_plane = false;

      m_state.state.x_axis_calibrated = false;
      m_state.state.y_axis_calibrated = false;

      // Origin
      origin_.x = 0; origin_.y = 0; origin_.z = 0;

      // TODO read in configuration from file

      // Setup viewer
      viewer.reset(new pcl::visualization::PCLVisualizer("viewer"));
      viewer->createViewPort(0.0, 0.0, 0.5, 0.5, vp_calibration_clouds);
      viewer->createViewPort(0.5, 0.0, 1.0, 0.5, vp_calibration_axes);
      viewer->createViewPort(0.0, 0.5, 0.5, 1.0, vp_navigation);
      viewer->createViewPort(0.5, 0.5, 1.0, 1.0, vp_workspace);
   }

public:
   // Just in case..
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   
};

// Static initiations
int Engine2::WSObject::current_id = 0;
float Engine2::WSObject::fudge = 60.0; // TODO FIND A WAY TO LOWER THIS
float Engine2::WSObject::distance_fudge = 9999;
