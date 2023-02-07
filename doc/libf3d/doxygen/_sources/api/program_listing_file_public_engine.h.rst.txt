
.. _program_listing_file_public_engine.h:

Program Listing for File engine.h
=================================

|exhale_lsh| :ref:`Return to documentation for file <file_public_engine.h>` (``public/engine.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef f3d_engine_h
   #define f3d_engine_h
   
   #include "exception.h"
   #include "export.h"
   #include "interactor.h"
   #include "loader.h"
   #include "options.h"
   #include "window.h"
   
   #include <map>
   #include <string>
   #include <vector>
   
   namespace f3d
   {
   class F3D_EXPORT engine
   {
   public:
     explicit engine(window::Type windowType = window::Type::NATIVE);
   
     ~engine();
   
     void setCachePath(const std::string& cachePath);
   
     engine& setOptions(const options& opt);
   
     engine& setOptions(options&& opt);
   
     options& getOptions();
   
     window& getWindow();
   
     loader& getLoader();
   
     interactor& getInteractor();
   
     static void loadPlugin(const std::string& path);
   
     static void autoloadPlugins();
   
     struct libInformation
     {
       std::string Version;
       std::string BuildDate;
       std::string BuildSystem;
       std::string Compiler;
       std::string RaytracingModule;
       std::string ExternalRenderingModule;
       std::string VTKVersion;
       std::string PreviousCopyright;
       std::string Copyright;
       std::string License;
       std::string Authors;
     };
   
     static libInformation getLibInfo();
   
     struct readerInformation
     {
       std::string Name;
       std::string Description;
       std::vector<std::string> Extensions;
       std::vector<std::string> MimeTypes;
       std::string PluginName;
     };
   
     static std::vector<readerInformation> getReadersInfo();
   
     struct no_window_exception : public exception
     {
       no_window_exception(const std::string& what = "");
     };
   
     struct no_interactor_exception : public exception
     {
       no_interactor_exception(const std::string& what = "");
     };
   
     struct plugin_exception : public exception
     {
       plugin_exception(const std::string& what = "");
     };
   
   private:
     class internals;
     internals* Internals;
     engine(const engine& opt) = delete;
     engine(engine&& opt) = delete;
     engine& operator=(const engine& opt) = delete;
     engine& operator=(engine&& opt) = delete;
   };
   }
   
   #endif
