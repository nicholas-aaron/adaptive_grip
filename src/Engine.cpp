#include "Engine.h"

Engine::Engine(PointCloud::Ptr      cloud,
               Mutex *              mutex) :
   _cloud(cloud),
   _mutex(mutex),
   m_camera(new CameraFile<Point>(_cloud, _mutex)),
   m_plane_filter(new PlaneFilter<Point>(_cloud, _mutex)),
   m_cluster_finder(_cloud, _mutex)
{
   m_vis_cloud.reset(new PointCloud());
   m_state = GOT_NOTHING;
}

void
Engine::get_image()
{
   if (m_camera->retrieve() == SUCCESS) {
      m_state = GOT_CLOUD;
   } else {
      m_logger.log("Could not retrieve an image!");
      m_state = GOT_NOTHING;
   };

   // Copy the cloud
   // VISUALIZATION
   m_vis_cloud.reset(new PointCloud());
   *m_vis_cloud = *_cloud;
}

void
Engine::get_floor()
{
// m_logger.log("Engine::get_floor() not yet implemented!");
   
   m_plane_filter->filter_plane();

   // FYI : http://www.cprogramming.com/tutorial/references.html
   const std::vector<int> & indices = m_plane_filter->get_inliers();

   // Colour them
   for (std::vector<int>::const_iterator it = indices.begin();
         it != indices.end();
         ++it)
   {
      m_vis_cloud->points[*it].r = 0xFF;
      m_vis_cloud->points[*it].g = 0xB6;
      m_vis_cloud->points[*it].b = 0xC1;
   }
   
}

void
Engine::get_clusters()
{
// m_logger.log("Engine::get_clusters() not yet implemented!");

   const int clusters = m_cluster_finder.find_clusters();

   std::stringstream s("");
   s << "Found " << clusters << " clusters.";
   m_logger.log(s);


   // VISUALIZATION -- create points corresponding to the clusters
   for(std::vector<pcl::PointIndices>::const_iterator it = m_cluster_finder.m_clusters.begin();
         it != m_cluster_finder.m_clusters.end();
         ++it)
   {
//    const std::vector<int> & indices = (*it).
//    for (std::vector<int>::const_iterator it = m_clusters
   }

   
}

void
Engine::get_movement()
{
   m_logger.log("Engine::get_movement() not yet implemented!");
}

