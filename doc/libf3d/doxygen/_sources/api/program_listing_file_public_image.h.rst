
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
   #include <vector>
   
   namespace f3d
   {
   class F3D_EXPORT image
   {
   public:
     enum class SaveFormat : unsigned char
     {
       PNG,
       JPG,
       TIF,
       BMP
     };
   
     enum class ChannelType : unsigned char
     {
       BYTE,
       SHORT,
       FLOAT
     };
   
     explicit image(const std::string& path);
   
     image(unsigned int width, unsigned int height, unsigned int channelCount,
       ChannelType type = ChannelType::BYTE);
   
   
     image();
     ~image();
     image(const image& img);
     image& operator=(const image& img) noexcept;
     image(image&& img) noexcept;
     image& operator=(image&& img) noexcept;
   
   
     bool operator==(const image& reference) const;
     bool operator!=(const image& reference) const;
   
     static std::vector<std::string> getSupportedFormats();
   
   
     unsigned int getWidth() const;
     unsigned int getHeight() const;
   #ifndef F3D_NO_DEPRECATED
     F3D_DEPRECATED image& setResolution(unsigned int width, unsigned int height);
   #endif
   
   
     unsigned int getChannelCount() const;
   #ifndef F3D_NO_DEPRECATED
     F3D_DEPRECATED image& setChannelCount(unsigned int dim);
   #endif
   
     ChannelType getChannelType() const;
   
     unsigned int getChannelTypeSize() const;
   
   
     image& setContent(void* buffer);
     void* getContent() const;
   #ifndef F3D_NO_DEPRECATED
     F3D_DEPRECATED image& setData(unsigned char* buffer);
     F3D_DEPRECATED unsigned char* getData() const;
   #endif
   
     bool compare(const image& reference, double threshold, image& diff, double& error) const;
   
     void save(const std::string& path, SaveFormat format = SaveFormat::PNG) const;
   
     std::vector<unsigned char> saveBuffer(SaveFormat format = SaveFormat::PNG) const;
   
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
