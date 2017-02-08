#include "calibrationwindow.h"
#include "ui_calibrationwindow.h"

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

/*
 * Constructor
 */



CalibrationWindow::CalibrationWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::CalibrationWindow),
    cam_buffer(new PointCloud()),
    cloud_src(new PointCloud()),
    cloud_tgt(new PointCloud()),
#ifdef ROBOT
    robot("/dev/cu.usbserial"),
#endif
    mutex(new Mutex())
//  logger(new Logger())
//  robot("/dev/cu.usbserial")
{
    ui->setupUi(this);


    // Initialize the logger, attach it to the txt display
    logger = new QLogger(ui->textBrowser);

    // INITIALIZE THE VIEWER TODO
    viewer.reset(new pcl::visualization::PCLVisualizer("Viewer", false)); // "false" - no interactor

    ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow());
    viewer->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());
    ui->qvtkWidget->update();

    // Create Viewports
    viewer->createViewPort(0.0, 0.0, 1.0, 0.5, vp_before);
    viewer->createViewPort(0.0, 0.5, 1.0, 1.0, vp_after);
   
    // 
//  camera = new CameraFile<PointT>(cam_buffer, mutex);
    camera = new Camera<PointT>(cam_buffer, mutex);

    // Try creating the robot object..
    

    CHECKED_CONNECT(ui->captureSrcButton, SIGNAL(pressed()), this, SLOT(captureSource()))
    CHECKED_CONNECT(ui->captureTgtButton, SIGNAL(pressed()), this, SLOT(captureTarget()))

    CHECKED_CONNECT(ui->registerButton,   SIGNAL(pressed()), this, SLOT(registerVector()))
    
    CHECKED_CONNECT(ui->calibrateButton, SIGNAL(pressed()), this, SLOT(calibrate()))

    num_iterations = 30;
}

/*
 * Destructor
 */
CalibrationWindow::~CalibrationWindow()
{
    delete ui;
}

void
CalibrationWindow::captureSource()
{
   camera->retrieve();
   *cloud_src = *cam_buffer;
   cam_buffer->clear();

   viewer->removePointCloud("source");
   //
   // Remove NaN
   std::vector<int> indices;
   pcl::removeNaNFromPointCloud(*cloud_src, *cloud_src, indices);

   // Filter floor/claw points out
   Mutex * tmp_mutex = new Mutex();
   PlaneFilter<PointT> * plane_filter = new TwoPlaneFilter<PointT>(cloud_src, tmp_mutex);
   plane_filter->filter_plane();

   // Normal Estimation
// normals_src.reset(new PointCloudWithNormals());
// _getNormalEstimation(normals_src, cloud_src);
// 
// PointCloudColorHandlerGenericField<PointNormalT>   color_handler(normals_src, "curvature");
// if (!color_handler.isCapable()) {
//    PCL_WARN ("Cannot create curvature color handler!");
// }

   PointCloudColorHandlerCustom<PointT> handler(cloud_src, 0, 255, 0);
   viewer->addPointCloud(cloud_src, handler, "source", vp_before);

   logger->log("Captured Source");
}

void
CalibrationWindow::captureTarget()
{
   camera->retrieve();
   *cloud_tgt = *cam_buffer;
   cam_buffer->clear();

   viewer->removePointCloud("target");

   // Remove NaN
   std::vector<int> indices;
   pcl::removeNaNFromPointCloud(*cloud_tgt, *cloud_tgt, indices);

   // Filter floor/claw points out
   Mutex * tmp_mutex = new Mutex();
   PlaneFilter<PointT> * plane_filter = new TwoPlaneFilter<PointT>(cloud_tgt, tmp_mutex);
   plane_filter->filter_plane();

   PointCloudColorHandlerCustom<PointT> handler(cloud_tgt, 255, 0, 0);
   viewer->addPointCloud(cloud_tgt, handler, "target", vp_before);

   logger->log("Captured Target");

}

void CalibrationWindow::cloud_align( const PointCloud::Ptr source,
                  const PointCloud::Ptr target,
                        PointCloud::Ptr            output,
                        Eigen::Matrix4f &          final_transform)
{
   

   // Get Normals.
   PointCloudWithNormals::Ptr source_normals(new PointCloudWithNormals());
   PointCloudWithNormals::Ptr target_normals(new PointCloudWithNormals());
// _getNormalEstimation(target_normals, target);
// _getNormalEstimation(source_normals, source);

  pcl::NormalEstimation<PointT, PointNormalT> norm_est;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  norm_est.setSearchMethod (tree);
  norm_est.setKSearch (30);
  
  norm_est.setInputCloud (source);
  norm_est.compute (*source_normals);
  pcl::copyPointCloud (*source, *source_normals);

  norm_est.setInputCloud (target);
  norm_est.compute (*target_normals);
  pcl::copyPointCloud (*target, *target_normals);

   {
      std::stringstream s;
      s << "Source Normals - size = " << source_normals->size() << std::endl;
      s << "Target Normals - size = " << target_normals->size();
      logger->log(s);
   }

   // Instantiate the custom point representation
   MyPointRepresentation      point_representation;
   // weight the 'curvature' dimension so it is balanced against x, y, z
   float alpha[4] = {1.0, 1.0, 1.0, 1.0};
   point_representation.setRescaleValues(alpha);
   
   // Alignment
   pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT>   reg;
   reg.setTransformationEpsilon(1e-6); // KNOB
   reg.setMaxCorrespondenceDistance(0.1);
// reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

   // Set Input/Output clouds of ICP object.
   reg.setInputSource(source_normals);
   reg.setInputTarget(target_normals);

   // The optimization loop. Visualize the reults.
   Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity();

   PointCloudWithNormals::Ptr    reg_result = source_normals;

   reg.setMaximumIterations(2);

   std::stringstream s("");
   s << "Optimization loop: will run " << num_iterations << " times." << std::endl;
   logger->log(s);

   PointCloudWithNormals::Ptr 	tmp_output(new PointCloudWithNormals);

   for (int i = 0; i < num_iterations ; ++i)
   {
      if (i % 10 == 0) {
         std::stringstream tmp("");
         tmp << "Iteration number " << i << "." << std::endl;
         logger->log(s);
      }

      std::cout << "i = " << i << std::endl;

//    // Save cloud for visualization purpose
      source_normals = reg_result;

      // Estimate
      reg.setInputSource(source_normals);
      reg.align(*tmp_output, Ti);

      // Accumulate transformation between each itearation
      Ti = reg.getFinalTransformation() * Ti;

   }

   // Get the transformation from target to source
   Eigen::Matrix4f targetToSource = Ti.inverse();

   // Transform the target back in source frame
   pcl::transformPointCloud(*target, *output, targetToSource);

   // Add point clouds back to the viewer
   PointCloudColorHandlerCustom<PointT>  cloud_tgt_h(output, 0, 255, 0);
   PointCloudColorHandlerCustom<PointT>  cloud_src_h(source, 255, 0, 0);

   viewer->addPointCloud(output, cloud_tgt_h, "output", vp_after);
   viewer->addPointCloud(source, cloud_src_h, "source", vp_after);
}

void
CalibrationWindow::registerVector()
{
   PointCloud::Ptr      register_output(new PointCloud());
   Eigen::Matrix4f      transform;
// cloud_align(cloud_src, cloud_tgt, register_output, transform);
   pairAlign(cloud_src, cloud_tgt, register_output, transform, false);
   logger->log("Finished regsterVector()");
}

  
void
CalibrationWindow::_getNormalEstimation(PointCloudWithNormals::Ptr   normals, PointCloud::Ptr   cloud)
{
   pcl::NormalEstimation<PointT, PointNormalT>     norm_est;
   pcl::search::KdTree<PointT>::Ptr                tree (new pcl::search::KdTree<PointT> ());
   
   norm_est.setSearchMethod (tree);
   norm_est.setKSearch(30);

   norm_est.setInputCloud(cloud);
   norm_est.compute(*normals);
   pcl::copyPointCloud(*cloud, *normals);
}

void CalibrationWindow::moveX(float amt)
{
}

void CalibrationWindow::moveY(float amt)
{
}

void CalibrationWindow::calibrate()
{

#ifdef ROBOT
   robot.home(false);
#endif
}

void CalibrationWindow::pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
{
  //
  // Downsample for consistency and speed
  // \note enable this for large datasets
  PointCloud::Ptr src (new PointCloud);
  PointCloud::Ptr tgt (new PointCloud);
  pcl::VoxelGrid<PointT> grid;
  if (downsample)
  {
    grid.setLeafSize (0.05, 0.05, 0.05);
    grid.setInputCloud (cloud_src);
    grid.filter (*src);

    grid.setInputCloud (cloud_tgt);
    grid.filter (*tgt);
  }
  else
  {
    src = cloud_src;
    tgt = cloud_tgt;
  }


  // Try a removeNanFromPointCloud
  // NTP You have to do this or else things get real fucky!!
  std::vector<int> indices1, indices2;
  pcl::removeNaNFromPointCloud(*src, *src, indices1);
  pcl::removeNaNFromPointCloud(*tgt, *tgt, indices2);

  // Compute surface normals and curvature
  PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
  PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

  pcl::NormalEstimation<PointT, PointNormalT> norm_est;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  norm_est.setSearchMethod (tree);
  norm_est.setKSearch (30);
  
  norm_est.setInputCloud (src);
  norm_est.compute (*points_with_normals_src);
  pcl::copyPointCloud (*src, *points_with_normals_src);

  norm_est.setInputCloud (tgt);
  norm_est.compute (*points_with_normals_tgt);
  pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

  //
  // Instantiate our custom point representation (defined above) ...
  MyPointRepresentation point_representation;
  // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
  point_representation.setRescaleValues (alpha);

  //
  // Align
  pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
  reg.setTransformationEpsilon (1e-6);
  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
  // Note: adjust this based on the size of your datasets
  reg.setMaxCorrespondenceDistance (0.1);  
  // Set the point representation
  reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

  reg.setInputSource (points_with_normals_src);
  reg.setInputTarget (points_with_normals_tgt);



  //
  // Run the same optimization in a loop and visualize the results
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
  PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
  reg.setMaximumIterations (2);
  for (int i = 0; i < 60; ++i)
  {
    PCL_INFO ("Iteration Nr. %d.\n", i);

    {
       std::stringstream ss;
       ss << "Iteration Nr. " << i;
       logger->log(ss);
    }

    // save cloud for visualization purpose
    points_with_normals_src = reg_result;

    // Estimate
    reg.setInputSource (points_with_normals_src);
    reg.align (*reg_result);

		//accumulate transformation between each Iteration
    Ti = reg.getFinalTransformation () * Ti;

		//if the difference between this transformation and the previous one
		//is smaller than the threshold, refine the process by reducing
		//the maximal correspondence distance
    if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
      reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);
    
    prev = reg.getLastIncrementalTransformation ();

    // visualize current state
//  showCloudsRight(points_with_normals_tgt, points_with_normals_src);
  }

	//
  // Get the transformation from target to source
  targetToSource = Ti.inverse();

  //
  // Transform target back in source frame
  pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);

  viewer->removePointCloud ("source_after");
  viewer->removePointCloud ("target_after");

  PointCloudColorHandlerCustom<PointT> cloud_tgt_h (output, 0, 255, 0);
  PointCloudColorHandlerCustom<PointT> cloud_src_h (cloud_src, 255, 0, 0);
  viewer->addPointCloud (output, cloud_tgt_h, "target_after", vp_after);
  viewer->addPointCloud (cloud_src, cloud_src_h, "source_after", vp_after);

   PCL_INFO ("Press q to continue the registration.\n");
//viewer->spin ();

//viewer->removePointCloud ("source"); 
//viewer->removePointCloud ("target");

  //add the source to the transformed target
  *output += *cloud_src;
  
  final_transform = targetToSource;

  // Now take apart final_transform
  //
  {
     std::stringstream ss;
     ss << final_transform; // Let's see..
     logger->log("Transform Matrix: ");
     logger->log(ss);
  }

  Eigen::Affine3f tmp;
          tmp.matrix() = final_transform.matrix();

          Eigen::MatrixXf 	translation = tmp.translation();
          Eigen::MatrixXf 	rotation	= tmp.rotation();
//Eigen::Matrix4f	scaling		= tmp.scaling().matrix();

  {
      std::stringstream ss;
      ss << "Translation Matrix: " << std::endl;
      ss << translation << std::endl;

      ss << "Rotation Matrix: " << std::endl;
      ss << rotation << std::endl;

      logger->log(ss);
  }

  
 }
