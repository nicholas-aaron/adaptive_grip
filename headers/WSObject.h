#ifndef WSOBJECT__H
#define WSOBJECT__H

#include <pcl/point_types.h>

class WSObject {

   public:

   float          x_position;
   float          y_position;

   // Don't store any z-axis information or "surface" information in 
   // WSObject. Handle it in Engine2.
// float          z_height;

   float          observation_distance;

   pcl::PointXYZRGBA   point;

   int            id;

   static float   fudge;
   static float   distance_fudge;
   static int     current_id;

	float x_obs_position;
	float y_obs_position;
	float r_display,b_display,g_display;


   WSObject(float x, float y, float obs_dist);

   // Copy constructor
   WSObject(const WSObject & copy); 

	bool operator== (const WSObject& ws) const;

};

#endif 
