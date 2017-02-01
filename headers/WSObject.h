#ifndef WSOBJECT__H
#define WSOBJECT__H

// #include <Eigen/Dense>

class WSObject {

   public:

   float          x_position;
   float          y_position;

   float          observation_distance;

   float          x_obs_position;
   float          y_obs_position;
   float          r_display, g_display, b_display;

   float          x_img_position;
   float          y_img_position;

   int            id;
// float          distance_to(WSObject & o, Eigen::Vector4f & plane);

   static float   fudge;
   static float   distance_fudge;
   static int     current_id;


   WSObject(float x, float y, float obs_dist);

   // Copy constructor
   WSObject(const WSObject & copy); 

   bool operator== (const WSObject& ws) const;

};

#endif 
