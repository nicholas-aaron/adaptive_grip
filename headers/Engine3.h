#include "Engine2.h"
#include <Eigen/Dense>
#include <pcl/visualization/pcl_plotter.h>

#define PI 3.14159265

class Engine3 : public Engine2 {

public: 

   pcl::visualization::PCLPlotter * plotter;

   Engine3(
         pcl::visualization::PCLVisualizer::Ptr _vis,
         Logger * _logger) : Engine2(_vis, _logger) {
      plotter = NULL;   }

   PointCloud::Ptr closest_cluster;
   double          claw_angle;

   int scan()
   {
      Engine2::scan();

      closest_cluster.reset(new PointCloud());
      
      // TODO this will break any of the "center" or "move to" methods
      // Colour the closest cluster. Will be at the same index as the 'centroid'
      const std::vector<WSObject>::iterator closest = get_closest_object();

      // TODO THIS WILL ONLY BE TRUE FOR THE FIRST SCAN.
      int cluster_index = std::distance(m_objects->begin(), closest);
      {
         stringstream msg;
         msg << "Engine3(): current_view.cf->m_clusters.size() = " << current_view.cf->m_clusters.size() << endl;
         msg << "closest_index = " << cluster_index << endl;
         msg << "Engine3()::scan - size of closest cluster = " << current_view.cf->m_clusters[cluster_index].indices.size();
         std::cout  << msg.str() << endl;
         m_logger->log(msg);
      }


      typedef std::vector<int>::const_iterator IndicesIt;
      for (IndicesIt it = current_view.cf->m_clusters[cluster_index].indices.begin();
            it != current_view.cf->m_clusters[cluster_index].indices.end(); ++it)
      {
         // copy these indices of current_view
         closest_cluster->push_back((*current_view.cloud)[*it]);
      }

      add_cloud_to_viewer(closest_cluster, "ClosestCluster", vp_calibration_axes, 255, 0, 0);

      // Now, we have the entire cluster, and its 
      
   // create_surface_map(closest_cluster, (*current_view.clusters)[cluster_index]);

   }

   int create_surface_map(PointCloud::Ptr surface, const Point & centroid)
   {
      typedef PointCloud::const_iterator Iterator;
      using PCLUtils::dot_double_normalize;
      
  //  std::vector<float> x_values;
  //  std::vector<float> y_values;
      std::vector<std::pair<double, double> > coordinates;
      Point difference;

      float x_diff, y_diff;

      for (Iterator i = surface->begin(); i != surface->end(); ++i)
      {
         // Get the difference
         PCLUtils::subtractXYZ(*i, centroid, difference);

         // Dot the difference with the x vector
         x_diff = dot_double_normalize(difference, m_cal.x_vector);

         // Dot the difference with the y vector
         y_diff = dot_double_normalize(difference, m_cal.y_vector);

         // x/y is backwards in one of the Coordinate.h class or
         // this..
         coordinates.push_back(std::pair<double, double> (y_diff, x_diff));
         
      }

      float slope_a, intercept_b;
      {
         //http://math.stackexchange.com/questions/204020/what-is-the-equation-used-to-calculate-a-linear-trendline
         // CALCULATE SLOPE
         //
         typedef std::vector<std::pair<double, double> >::iterator Iterator;

         float sum_xy   = 0.0;
         float sum_x    = 0.0;
         float sum_y    = 0.0;
         float sum_x2   = 0.0;
         float sum_y2   = 0.0;
         float n        = (float) coordinates.size();

         for (Iterator i = coordinates.begin(); i != coordinates.end();
               i++)
         {
            sum_xy   += i->first * i->second;
            sum_x    += i->first;
            sum_y    += i->second;
            sum_x2   += (i->first * i->first);
            sum_y2   += (i->second * i->second);
         }

         slope_a = (n * sum_xy) - (sum_x * sum_y);
         slope_a = slope_a / (n * sum_x2 - (sum_x*sum_x));
         intercept_b = (sum_y - slope_a * sum_x) / n;
      }

      std::vector<double> trendline(2, 0);
      trendline[1] = slope_a;
      trendline[2] = intercept_b;

      claw_angle = atan(slope_a) * 180 / PI;
      {
         stringstream msg;
         msg << "Calculated slope = " << claw_angle << std::endl;
         m_logger->log(msg);
      }

  //  if (plotter != NULL) {
  //     delete plotter;
  //  }
  //  plotter = new pcl::visualization::PCLPlotter();
  //  plotter->addPlotData(trendline, -4, 4, "trendline");
  //  plotter->addPlotData(coordinates, "coordinates", vtkChart::POINTS);
  //  plotter->plot();
   }

   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};
