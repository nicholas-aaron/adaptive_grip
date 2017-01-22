
#include "Shell.h"
#include "Mutex.h"
#include "TwoPlaneFilter.h"
#include "Camera.h"
#include "common_defs.h"
#include "Cluster.h"
#include <iostream>

#include "Engine.h"

#include "RobotExt.h"

#define RESOURCES

using std::string;
using pcl::visualization::PointCloudColorHandlerCustom;

class TestVectorShell : public Shell {

public :

   typedef pcl::PointXYZRGBA           Point;
   typedef pcl::PointCloud<Point>      PointCloud;

   Mutex *              mutex;
   PointCloud::Ptr      camcloud;
   Logger *             logger;

   RobotExt             robot;

   Camera<Point>     camera;


   Point xRegVector, yRegVector;
   Point planeNormal;



   PointCloud::Ptr      rcloud_src;
   PointCloud::Ptr      rcloud_tgt;

   int vp_left, vp_right;
   int vp_botright, vp_botleft, vp_topleft, vp_topright;

   PointCloud::Ptr      clusters;
   
   Eigen::Matrix4f      transform_mtx;

   // Just in case someone dynamically declares..
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW


   // Constructor
   TestVectorShell() :
      Shell(),
      mutex(new Mutex()),
      camcloud(new PointCloud()),
      logger(new Logger()),
//    engine(new Engine(camcloud, mutex, logger)),
      robot("/dev/cu.usbserial"),
      camera(camcloud, mutex)
   {
      bind_functions();
      viewer->createViewPort(0.0, 0.0, 0.5, 0.5, vp_topleft);
      viewer->createViewPort(0.5, 0.0, 1.0, 0.5, vp_topright);
      viewer->createViewPort(0.0, 0.5, 0.5, 1.0, vp_botleft);
      viewer->createViewPort(0.5, 0.5, 1.0, 1.0, vp_botright);


      clusters.reset(new PointCloud());
   }

   void bind_functions()
   {

      Shell::callback pos = boost::bind(&TestVectorShell::pos, this, _1);
      register_function(pos, "pos", "pos");

      Shell::callback home = boost::bind(&TestVectorShell::home, this, _1);
      register_function(home, "home", "home");

      Shell::callback move = boost::bind(&TestVectorShell::move, this, _1);
      register_function(move, "move", "move");

      Shell::callback regX = boost::bind(&TestVectorShell::regX, this, _1);
      register_function(regX, "regX", "regX");

      Shell::callback regY = boost::bind(&TestVectorShell::regY, this, _1);
      register_function(regY, "regY", "regY");

      Shell::callback analyze = boost::bind(&TestVectorShell::analyze, this, _1);
      register_function(analyze, "ana", "ana");
   }

   void pos(arg_list args)
   {
      // get current position.
      const    RobotPosition current = robot.currentPos();
      const    RobotPosition max     = robot.currentLimits().max();
      const    RobotPosition min     = robot.currentLimits().min();
      
      // cout << sprintf("") << endl;


      printf("%-10s | %-10s | %-10s | %-10s\n", "AXIS", "MIN", "CURRENT", "MAX");
      printf("%-10s | %-+10.2f | %-+10.2f | %-+10.2f\n", "X", min.x, current.x, max.x);
      printf("%-10s | %-+10.2f | %-+10.2f | %-+10.2f\n", "Y", min.y, current.y, max.y);
      printf("%-10s | %-+10.2f | %-+10.2f | %-+10.2f\n", "Z", min.z, current.z, max.z);
      printf("%-10s | %-+10.2f | %-+10.2f | %-+10.2f\n", "j4", min.j4, current.j4, max.j4);
      printf("%-10s | %-+10.2f | %-+10.2f | %-+10.2f\n", "j5", min.j5, current.j5, max.j5);
      printf("%-10s | %-+10.2f | %-+10.2f | %-+10.2f\n", "j6", min.j6, current.j6, max.j6);
      
   }



   void home(arg_list args)
   {
      robot.home(false);
   }

   void move(arg_list args)
   {
      RobotPosition new_pos = robot.currentPos();

      while (!args.empty()) {
         string axis = args.front();
         float value;
         args.pop_front();
         if (args.empty()) {
            cout << "Illegal command." << endl;
            return;
         } else {
            value = atof(args.front().c_str());
            args.pop_front();
         }

         if      (axis == "x")      new_pos.x = value;
         else if (axis == "y")      new_pos.y = value;
         else if (axis == "z")      new_pos.z = value;
         else {
            cout << "Unrecognized argument: <" << axis << ">" << endl;
            return;
         }
      }

      // Output the position to the console
      cout << "NEW POSITION:" << endl;
      cout << new_pos << endl;

      robot.moveTo(new_pos);

      // Poll the buffer
      string reply;
      robot.controller >> reply;

      cout << "REPLY:" << endl << reply << endl;

   }



   // TODO move to utils
   void subtractXYZ(const Point & _this_, const Point & _minus_this_, Point & _equals_this_)
   {
      _equals_this_.x = _this_.x - _minus_this_.x;
      _equals_this_.y = _this_.y - _minus_this_.y;
      _equals_this_.z = _this_.z - _minus_this_.z;
   }
   // TODO move to utils
   void addXYZ(const Point & _this_, const Point & _plus_this_, Point & _equals_this_)
   {
      _equals_this_.x = _this_.x + _plus_this_.x;
      _equals_this_.y = _this_.y + _plus_this_.y;
      _equals_this_.z = _this_.z + _plus_this_.z;
   }

   void load1(arg_list args)
   {
      viewer->removePointCloud("RCloudOneUnfiltered");
      rcloud_src.reset(new PointCloud);
      camera.retrieve();
      *rcloud_src = *camcloud;
         PointCloudColorHandlerCustom<Point> handler_red_1(rcloud_src, 255, 0, 0);
         viewer->addPointCloud(rcloud_src, handler_red_1, "RCloudOneUnfiltered", vp_topleft);
   }


   void load2(arg_list args)
   {
      viewer->removePointCloud("RCloudTwoUnfiltered");
      rcloud_tgt.reset(new PointCloud);
      camera.retrieve();
      *rcloud_tgt = *camcloud;
         PointCloudColorHandlerCustom<Point> handler_blue_1(rcloud_tgt, 0, 0, 255);
         viewer->addPointCloud(rcloud_tgt, handler_blue_1, "RCloudTwoUnfiltered", vp_topleft);
   }

   

   void scene_setup(int viewport, Eigen::Vector4f & plane_eigen, PointCloud::Ptr clusters)
   {
      Point origin; origin.x = 0.0, origin.y = 0.0, origin.z = 0.0;
      Cluster<Point> originCluster(origin);

      Point originProjection = originCluster.get_plane_projection(plane_eigen);

      viewer->removeShape("originLine", viewport);
      viewer->addLine(origin, originProjection, "originLine", viewport);

      viewer->removePointCloud("clusterLocations", viewport);
      viewer->addPointCloud(clusters, "Clusters", viewport);
   }

      // NO REGISTRATION - only using clusters
   bool clusterDifference(Point & difference,
      PointCloud::Ptr source_cloud,
      PointCloud::Ptr target_cloud,
      Eigen::Vector4f & floor_plane,
      PointCloud::Ptr clusterCloud)
   {
      Mutex m;

      // Remove Plane (floor) from point clouds.
      {
         TwoPlaneFilter<Point> tpOne(source_cloud, &m);
         tpOne.filter_plane();
         TwoPlaneFilter<Point> tpTwo(target_cloud, &m);
         tpTwo.filter_plane();

         floor_plane = tpTwo.get_plane_coefficients();
      }

      // Locate clusterCloud.
      ClusterFinder<Point> cfOne(source_cloud, &m);
      cfOne.find_clusters();
      PointCloud::Ptr      source_clusters = cfOne.get_clusters();
      std::cout << "rlcoud_src: Found " << source_clusters->size() << " clusterCloud." << std::endl;

      ClusterFinder<Point> cfTwo(target_cloud, &m);
      cfTwo.find_clusters();
      PointCloud::Ptr      target_clusters = cfTwo.get_clusters();
      std::cout << "target_cloud: Found " << target_clusters->size() << " clusterCloud." << std::endl;

      if (source_clusters->size() != 1 || target_clusters->size() != 1)
      {
         std::cerr << "clusterDifference() - located more than one cluster in one of the point clouds." << std::endl;
         std::cerr << "Returning false." << std::endl;
         return false;
      }

      Point source_cluster = source_clusters->points[0];
      Point target_cluster = target_clusters->points[0];

      source_cluster.r = 255; source_cluster.g = 0; source_cluster.b = 0;
      target_cluster.r = 0; target_cluster.g = 0; target_cluster.b = 255;

      clusterCloud->push_back(source_cluster);
      clusterCloud->push_back(target_cluster);

      subtractXYZ(target_cluster, source_cluster, difference);

      return true;
   }

   void regX(arg_list args)
   {
      load1(args); // Load first point cloud and display.
      move(args);
      load2(args); // Load second point cloud and display.

//    Point difference;
      Eigen::Vector4f floor_plane;

      PointCloud::Ptr clusters(new PointCloud());

      bool result = clusterDifference(xRegVector, rcloud_src, rcloud_tgt, floor_plane, clusters);

      planeNormal.x = floor_plane(0);
      planeNormal.y = floor_plane(1);
      planeNormal.z = floor_plane(2);

      // Visualize the clusters. Source = red, Target = blue.

      Point difference_fp; // Differnce, on floor plane

      Cluster<Point> originCluster(0.0, 0.0, 0.0);
      Point originProjection = originCluster.get_plane_projection(floor_plane);

      addXYZ(originProjection, xRegVector, difference_fp);
      viewer->addLine(originProjection, difference_fp, 214, 0, 255, "XRegLine", vp_botright);

      scene_setup(vp_botright, floor_plane, clusters);
   }

   void regY(arg_list args)
   {
      load1(args);
      move(args);
      load2(args);

      Point difference;
      Eigen::Vector4f floor_plane;
      
      PointCloud::Ptr clusters(new PointCloud());
      bool result = clusterDifference(yRegVector, rcloud_src, rcloud_tgt, floor_plane, clusters);

      Point difference_fp; // Difference, on floor plane
      Cluster<Point> originCluster(0.0, 0.0, 0.0);
      Point originProjection = originCluster.get_plane_projection(floor_plane);

      addXYZ(originProjection, yRegVector, difference_fp);
      viewer->addLine(originProjection, difference_fp, 255, 165, 0, "YRegLine", vp_botright);

      viewer->updatePointCloud(clusters, "Clusters");
   }

   void analyze(arg_list args)
   {
      float xy_angle_rads = PCLUtils::angleBetween(xRegVector, yRegVector);
      float xy_angle_degrees = acos(xy_angle_rads) * 180.0 / 3.14159;

      std::cout << "xy_angle_degrees = " << xy_angle_degrees << std::endl;

      float xz_angle_rads = PCLUtils::angleBetween(xRegVector, planeNormal);
      float xz_angle_degrees = acos(xz_angle_degrees) * 180.0 / 3.14159;

      std::cout << "xz_angle_degrees = " << xz_angle_degrees << std::endl;

      float yz_angle_rads = PCLUtils::angleBetween(yRegVector, planeNormal);
      float yz_angle_degrees = acos(yz_angle_rads) * 180.0 / 3.14159;

      std::cout << "yz_angle_degrees = " << yz_angle_degrees << std::endl;
   }

   void reg3(arg_list args)
   {
      PointCloud::Ptr      reg_output(new PointCloud);
      Mutex                m;
      Eigen::Matrix4f      transform;
      /* Capture rcloud_src and rcloud_tgt */
//    viewer->removePointCloud("RCloudOne");
//    viewer->removePointCloud("RCloudTwo");

//    rcloud_src.reset(new PointCloud);
//    Camera<Point> cam (cam_buffer, &m);
// // engine->get_image();
//    cam.retrieve();
//    *rcloud_src = *cam_buffer;

//    move(args);

//    rcloud_tgt.reset(new PointCloud);
//    engine->get_image();
//    cam.retrieve();
//    *rcloud_tgt = *cam_buffer;

      
      
      /* Display the unfilitered point clouds on the top right.. */
      std::cout <<  "Before filter: rcloud_src->size() = " << rcloud_src->size() << std::endl;
      std::cout <<  "Before filter: rcloud_tgt->size() = " << rcloud_tgt->size() << std::endl;

      /* Filter the Point Clouds */
      TwoPlaneFilter<Point> tp2(rcloud_tgt, &m);
      tp2.filter_plane();
      TwoPlaneFilter<Point> tp1(rcloud_src, &m);
//    tp2.set_input_cloud(rcloud_src);
      tp1.filter_plane();
      // TODO there's an error with using the same TwoPlaneFilter with two different point clouds..


//    return;

      /* Display original point clouds on bottom right. */
     
      PointCloudColorHandlerCustom<Point> handler_red_2(rcloud_src, 255, 0, 0);
      viewer->addPointCloud(rcloud_src, handler_red_2, "RCloudOne", vp_botright);
      PointCloudColorHandlerCustom<Point> handler_blue_2(rcloud_tgt, 0, 0, 255);
      viewer->addPointCloud(rcloud_tgt, handler_blue_2, "RCloudTwo", vp_botright);
      

      std::cout <<  "After filter: rcloud_src->size() = " << rcloud_src->size() << std::endl;
      std::cout <<  "After filter: rcloud_tgt->size() = " << rcloud_tgt->size() << std::endl;


      /* Then, try to register the point clouds */
      PCLUtils::pairAlign<Point>(rcloud_src, rcloud_tgt, reg_output, transform, logger);

      // Scratch:
      // by applying the "transform" to cf2 (target), you get back the points in cf1 (source).
      // the "transform" is the target-to-source transform. Can invert it (Matrix.inverse()) to get
      // source-to-target.
      //
      /* Display the source centroids (blue),
       * Target cloud centroids (red),
       * and estimate-target centroids (Green),
       *
       * LIMIT to one centroid */
      ClusterFinder<Point>    cf(rcloud_src, &m);
      cf.find_clusters();
      PointCloud::Ptr clusters_src = cf.get_clusters();
      std::cout << "clusters_src.size() = " << clusters_src->size() << std::endl;

      ClusterFinder<Point>    cf2(rcloud_tgt, &m);
    //cf.setInputCloud(rcloud_tgt);
      cf2.find_clusters();
      PointCloud::Ptr clusters_tgt = cf2.get_clusters();
      std::cout << "clusters_tgt.size() = " << clusters_tgt->size() << std::endl;
      // TODO same thing with the clusterFinder. use two of them.

      /* Calculate the estimate of the source */
      Eigen::Affine3f transform_affine;
      transform_affine.matrix() = transform.matrix();
      PointCloud::Ptr   src_centroids_est(new PointCloud);
      pcl::transformPointCloud(*clusters_src, *src_centroids_est, transform_affine);

      Eigen::Vector4f tmp_plane = tp2.get_plane_coefficients();
      int lineNum = 0;
      std::cout << "From clusters_tgt..." << std::endl;
      for (std::vector<Point>::iterator i = clusters_tgt->begin();
            i != clusters_tgt->end(); i++)
      {
         Cluster<Point> c (*i);
         Point    projection = c.get_plane_projection(tmp_plane);
         std::stringstream lineName;
         lineName << "Line" << (lineNum++);
         viewer->addLine(c.m_location, projection, lineName.str().c_str(), vp_botleft);
         // ALSO, OUTPUT THE POINT...
         std::cout << "CLUSTER " << lineNum << ": x = " << (*i).x <<
            ", y = " << (*i).y << ", z = " << (*i).z << std::endl;
      }

      std::cout << "From clusters_src..." << std::endl;
      for (std::vector<Point>::iterator i = clusters_src->begin();
            i != clusters_src->end(); i++)
      {
         Cluster<Point> c (*i);
         Point    projection = c.get_plane_projection(tmp_plane);
         std::stringstream lineName;
         lineName << "Line" << (lineNum++);
         viewer->addLine(c.m_location, projection, lineName.str().c_str(), vp_botleft);
         std::cout << "CLUSTER " << lineNum << ": x = " << (*i).x <<
            ", y = " << (*i).y << ", z = " << (*i).z << std::endl;
      }

//    viewer->addPlane(*(tp2.m_plane), "tp2.m_plane");

      Point origin; origin.x = 0.0, origin.y = 0.0, origin.z = 0.0;
      origin.r = 255; origin.g = 255; origin.b = 255;
      Cluster<Point> originCluster(origin);
      Point originProjection = originCluster.get_plane_projection(tmp_plane);
      viewer->addLine(origin, originProjection, "originLine", vp_botleft);
      

      /* Now Show the Estimate Centroids (Green), Source Centroids (Blue), Target Centroids (Red) */
      {
         PointCloudColorHandlerCustom<Point> handler_red(clusters_src, 255, 0, 0);
         viewer->addPointCloud(clusters_src, handler_red, "ClustersSrc", vp_botleft);

         PointCloudColorHandlerCustom<Point> handler_blue(clusters_tgt, 0, 0, 255);
         viewer->addPointCloud(clusters_tgt, handler_blue, "ClustersTgt", vp_botleft);

         

         PointCloudColorHandlerCustom<Point> handler_green(src_centroids_est, 0, 255, 0);
         viewer->addPointCloud(src_centroids_est, handler_green, "ClustersSrcEst", vp_botleft);

         viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "ClustersSrc");
         viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "ClustersTgt");
         viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "ClustersSrcEst");
      }
      

      if (clusters_src->size() != 1 || clusters_tgt->size() != 1)
      {
         std::cerr << "Error - more than one cluster in source/target cluster clouds. " << std::endl;
         std::cerr << "When attempting to get vector from source clusters to target cluster" << std::endl;
         return;
      }

      pcl::PointXYZ tmp_originProjection; tmp_originProjection.x = originProjection.x;
      tmp_originProjection.y = originProjection.y;
      tmp_originProjection.z = originProjection.z;


      pcl::PointXYZ source_to_target;
      source_to_target.x = clusters_tgt->points[0].x - clusters_src->points[0].x;
      source_to_target.y = clusters_tgt->points[0].y - clusters_src->points[0].y;
      source_to_target.z = clusters_tgt->points[0].z - clusters_src->points[0].z;
      
      // The same vector, but starting from the projection of the origin.
      pcl::PointXYZ source_to_target_op;
      source_to_target_op.x = tmp_originProjection.x + source_to_target.x;
      source_to_target_op.y = tmp_originProjection.y + source_to_target.y;
      source_to_target_op.z = tmp_originProjection.z + source_to_target.z;
      viewer->addLine(tmp_originProjection, source_to_target_op, 255, 0, 0, "vec", vp_botleft);


   }

};


int main (int argc, char ** argv)
{
   TestVectorShell   ts;
   ts.run_shell();
   return SUCCESS;
};
