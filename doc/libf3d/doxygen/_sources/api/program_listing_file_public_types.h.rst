
.. _program_listing_file_public_types.h:

Program Listing for File types.h
================================

|exhale_lsh| :ref:`Return to documentation for file <file_public_types.h>` (``public/types.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef f3d_types_h
   #define f3d_types_h
   
   #include "exception.h"
   #include "export.h"
   
   #include <algorithm>
   #include <array>
   #include <iostream>
   #include <string>
   #include <vector>
   
   namespace f3d
   {
   struct type_construction_exception : public exception
   {
     explicit type_construction_exception(const std::string& what = "")
       : exception(what) {};
   };
   
   struct type_access_exception : public exception
   {
     explicit type_access_exception(const std::string& what = "")
       : exception(what) {};
   };
   
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
   
   class F3D_EXPORT ratio_t
   {
   public:
     ratio_t() = default;
     ratio_t(double val)
       : Value(val)
     {
     }
     operator double() const
     {
       return this->Value;
     }
   
   private:
     double Value;
   };
   
   class F3D_EXPORT color_t
   {
   public:
     color_t() = default;
     color_t(const std::vector<double>& vec)
     {
       if (vec.size() != 3)
       {
         throw f3d::type_construction_exception("Provided vector does not have the right size");
       }
       std::copy_n(vec.begin(), 3, this->RGB.begin());
     }
     color_t(double red, double green, double blue)
     {
       this->RGB[0] = red;
       this->RGB[1] = green;
       this->RGB[2] = blue;
     }
     color_t(const std::initializer_list<double>& list)
     {
       if (list.size() != 3)
       {
         throw f3d::type_construction_exception("Provided list does not have the right size");
       }
       std::copy_n(list.begin(), 3, this->RGB.begin());
     }
     operator std::vector<double>() const
     {
       return std::vector<double>(this->RGB.begin(), this->RGB.end());
     }
     bool operator==(const color_t& other) const
     {
       return this->RGB == other.RGB;
     }
     bool operator!=(const color_t& other) const
     {
       return this->RGB != other.RGB;
     }
     double operator[](size_t i) const
     {
       if (i >= 3)
       {
         throw f3d::type_access_exception("Incorrect index");
       }
       return this->RGB[i];
     }
     double& operator[](size_t i)
     {
       if (i >= 3)
       {
         throw f3d::type_access_exception("Incorrect index");
       }
       return this->RGB[i];
     }
     const double* data() const
     {
       return this->RGB.data();
     }
     double r() const
     {
       return this->RGB[0];
     }
     double g() const
     {
       return this->RGB[1];
     }
     double b() const
     {
       return this->RGB[2];
     }
   
   private:
     std::array<double, 3> RGB;
   };
   
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
