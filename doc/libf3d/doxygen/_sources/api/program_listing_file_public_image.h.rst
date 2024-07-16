
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
   
     std::vector<double> getNormalizedPixel(const std::pair<int, int>& xy) const;
   
     static std::vector<std::string> getSupportedFormats();
   
   
     unsigned int getWidth() const;
     unsigned int getHeight() const;
   
   
     unsigned int getChannelCount() const;
   
     ChannelType getChannelType() const;
   
     unsigned int getChannelTypeSize() const;
   
   
     image& setContent(void* buffer);
     void* getContent() const;
   
     bool compare(const image& reference, double threshold, image& diff, double& error) const;
   
     void save(const std::string& path, SaveFormat format = SaveFormat::PNG) const;
   
     std::vector<unsigned char> saveBuffer(SaveFormat format = SaveFormat::PNG) const;
   
     const f3d::image& toTerminalText(std::ostream& stream) const;
   
     std::string toTerminalText() const;
   
     f3d::image& setMetadata(const std::string& key, const std::string& value);
   
     std::string getMetadata(const std::string& key) const;
   
     std::vector<std::string> allMetadata() const;
   
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
