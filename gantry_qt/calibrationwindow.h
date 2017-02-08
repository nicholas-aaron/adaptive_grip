#ifndef CALIBRATIONWINDOW_H
#define CALIBRATIONWINDOW_H

//#define ROBOT

#include <QMainWindow>

#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <vtkRenderWindow.h>

#include "Mutex.h"
#include "common_defs.h"
#include "Camera.h"
#include "CameraFile.h" // TODO Remove. used to debug
#include "QLogger.h"
#include "TwoPlaneFilter.h"

#include "RobotExt.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <Eigen/Dense>

// Include Robot.moveTo()..

#define CHECKED_CONNECT(source, signal, receiver, slot) \
   if (!connect(source, signal, receiver, slot)) \
      qt_assert_x(Q_FUNC_INFO, "CHECKED_CONNECT failed", __FILE__, __LINE__);

namespace Ui {
class CalibrationWindow;
}

// struct GantryVector {
//    RobotPosition     robotVector;
//    Eigen::Vector4f   cloudVector;
// }
//

// Define a new point representation for < x, y, z, curvature >
//class MyPointRepresentation : public pcl::PointRepresentation <pcl::PointNormal>
//{
//  using pcl::PointRepresentation<pcl::PointNormal>::nr_dimensions_;
//public:
//  MyPointRepresentation ()
//  {
//    // Define the number of dimensions
//    nr_dimensions_ = 4;
//  }
//
//  // Override the copyToFloatArray method to define our feature std::vector
//  virtual void copyToFloatArray (const pcl::PointNormal &p, float * out) const
//  {
//    // < x, y, z, curvature >
//    out[0] = p.x;
//    out[1] = p.y;
//    out[2] = p.z;
//    out[3] = p.curvature;
//  }
//};

// TODO temporarily global
    typedef    pcl::PointXYZ              PointT;
    typedef    pcl::PointCloud<PointT>    PointCloud;
    typedef pcl::PointNormal              PointNormalT;
    typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

//     void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample);

class CalibrationWindow : public QMainWindow
{
    Q_OBJECT

public:

//  typedef    pcl::PointXYZ              PointT;
//  typedef    pcl::PointCloud<PointT>    PointCloud;
//  typedef pcl::PointNormal              PointNormalT;
//  typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;


    explicit CalibrationWindow(QWidget *parent = 0);
    ~CalibrationWindow();

    // Two reference cloud members for point cloud registration
    Mutex *                                     mutex;
    PointCloud::Ptr                             cam_buffer;
    PointCloud::Ptr                             cloud_src;
    PointCloud::Ptr                             cloud_tgt;
    pcl::visualization::PCLVisualizer::Ptr      viewer;
    CameraBase<PointT> *                        camera;
    Logger *                                    logger;

    PointCloudWithNormals::Ptr                  normals_src;
    PointCloudWithNormals::Ptr                  normals_tgt;

#ifdef ROBOT
    RobotExt                                    robot;
#endif

    int vp_before;
    int vp_after;

    int num_iterations;

private:
    Ui::CalibrationWindow *ui;

   void _getNormalEstimation(PointCloudWithNormals::Ptr   normals, PointCloud::Ptr   cloud);

   void cloud_align( const PointCloud::Ptr source, 
                     const PointCloud::Ptr target,
                        PointCloud::Ptr            output,
                        Eigen::Matrix4f &          final_transform);

     void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample);
public slots:
      
   void captureSource();
   void captureTarget();
   void registerVector();

   void calibrate();

   void moveX(float amt);
   void moveY(float amt);


};

#endif // CALIBRATIONWINDOW_H
