
.. _program_listing_file_public_image.h:

Program Listing for File image.h
================================

|exhale_lsh| :ref:`Return to documentation for file <file_public_image.h>` (``public/image.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef f3d_image_h
   #define f3d_image_h
   
   #include "exception.h"
   #include "export.h"
   
   #include <string>
   
   namespace f3d
   {
   class F3D_EXPORT image
   {
   public:
     explicit image(const std::string& path);
   
   
     image();
     ~image();
     image(const image& img);
     image& operator=(const image& img) noexcept;
     image(image&& img) noexcept;
     image& operator=(image&& img) noexcept;
   
   
     bool operator==(const image& reference) const;
     bool operator!=(const image& reference) const;
   
   
     unsigned int getWidth() const;
     unsigned int getHeight() const;
     image& setResolution(unsigned int width, unsigned int height);
   
   
     unsigned int getChannelCount() const;
     image& setChannelCount(unsigned int dim);
   
   
     image& setData(unsigned char* buffer);
     unsigned char* getData() const;
   
     bool compare(const image& reference, double threshold, image& diff, double& error) const;
   
     void save(const std::string& path) const;
   
     struct write_exception : public exception
     {
       explicit write_exception(const std::string& what = "");
     };
   
     struct read_exception : public exception
     {
       explicit read_exception(const std::string& what = "");
     };
   
   private:
     class internals;
     internals* Internals;
   };
   }
   
   #endif
