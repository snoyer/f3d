
.. _program_listing_file_public_types.h:

Program Listing for File types.h
================================

|exhale_lsh| :ref:`Return to documentation for file <file_public_types.h>` (``public/types.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef f3d_types_h
   #define f3d_types_h
   
   #include "export.h"
   
   #include <array>
   #include <string>
   #include <vector>
   
   namespace f3d
   {
   struct F3D_EXPORT point3_t : std::array<double, 3>
   {
     template<typename... Args>
     point3_t(Args&&... args)
       : array({ double(std::forward<Args>(args))... })
     {
     }
   };
   
   struct F3D_EXPORT vector3_t : std::array<double, 3>
   {
     template<typename... Args>
     vector3_t(Args&&... args)
       : array({ double(std::forward<Args>(args))... })
     {
     }
   };
   
   using angle_deg_t = double;
   
   struct mesh_t
   {
     std::vector<float> points;
     std::vector<float> normals;
     std::vector<float> texture_coordinates;
     std::vector<unsigned int> face_sides;
     std::vector<unsigned int> face_indices;
   
     F3D_EXPORT std::pair<bool, std::string> isValid() const;
   };
   }
   
   #endif
