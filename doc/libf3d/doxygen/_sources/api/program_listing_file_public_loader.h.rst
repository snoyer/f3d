
.. _program_listing_file_public_loader.h:

Program Listing for File loader.h
=================================

|exhale_lsh| :ref:`Return to documentation for file <file_public_loader.h>` (``public/loader.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef f3d_loader_h
   #define f3d_loader_h
   
   #include "exception.h"
   #include "export.h"
   
   #include <string>
   #include <vector>
   
   namespace f3d
   {
   class F3D_EXPORT loader
   {
   public:
     struct load_failure_exception : public exception
     {
       load_failure_exception(const std::string& what = "")
         : exception(what){};
     };
   
     virtual bool hasGeometryReader(const std::string& filePath) = 0;
   
     virtual loader& loadGeometry(const std::string& filePath, bool reset = false) = 0;
   
     virtual bool hasSceneReader(const std::string& filePath) = 0;
   
     virtual loader& loadScene(const std::string& filePath) = 0;
   
   protected:
     loader() = default;
     virtual ~loader() = default;
     loader(const loader& opt) = delete;
     loader(loader&& opt) = delete;
     loader& operator=(const loader& opt) = delete;
     loader& operator=(loader&& opt) = delete;
   };
   }
   
   #endif
