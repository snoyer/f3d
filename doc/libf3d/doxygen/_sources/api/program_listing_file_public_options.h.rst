
.. _program_listing_file_public_options.h:

Program Listing for File options.h
==================================

|exhale_lsh| :ref:`Return to documentation for file <file_public_options.h>` (``public/options.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef f3d_options_h
   #define f3d_options_h
   
   #include "exception.h"
   #include "export.h"
   
   #include <string>
   #include <vector>
   
   namespace f3d
   {
   class F3D_EXPORT options
   {
   public:
   
     options();
     ~options();
     options(const options& opt);
     options& operator=(const options& opt) noexcept;
     options(options&& other) noexcept;
     options& operator=(options&& other) noexcept;
   
   
     options& set(const std::string& name, bool value);
     options& set(const std::string& name, int value);
     options& set(const std::string& name, double value);
     options& set(const std::string& name, const std::string& value);
     options& set(const std::string& name, const char* value);
     options& set(const std::string& name, const std::vector<int>& values);
     options& set(const std::string& name, const std::vector<double>& values);
     options& set(const std::string& name, std::initializer_list<int> values);
     options& set(const std::string& name, std::initializer_list<double> values);
   
   
     void get(const std::string& name, bool& value) const;
     void get(const std::string& name, int& value) const;
     void get(const std::string& name, double& value) const;
     void get(const std::string& name, std::string& value) const;
     void get(const std::string& name, std::vector<int>& value) const;
     void get(const std::string& name, std::vector<double>& value) const;
   
   
     bool getAsBool(const std::string& name) const;
     int getAsInt(const std::string& name) const;
     double getAsDouble(const std::string& name) const;
     std::string getAsString(const std::string& name) const;
     std::vector<int> getAsIntVector(const std::string& name) const;
     std::vector<double> getAsDoubleVector(const std::string& name) const;
   
   
     bool& getAsBoolRef(const std::string& name);
     int& getAsIntRef(const std::string& name);
     double& getAsDoubleRef(const std::string& name);
     std::string& getAsStringRef(const std::string& name);
     std::vector<int>& getAsIntVectorRef(const std::string& name);
     std::vector<double>& getAsDoubleVectorRef(const std::string& name);
   
     options& toggle(const std::string& name);
   
     bool isSame(const options& other, const std::string& name) const;
   
     options& copy(const options& other, const std::string& name);
   
     std::vector<std::string> getNames();
   
     std::pair<std::string, unsigned int> getClosestOption(const std::string& option) const;
   
     struct incompatible_exception : public exception
     {
       explicit incompatible_exception(const std::string& what = "");
     };
   
     struct inexistent_exception : public exception
     {
       explicit inexistent_exception(const std::string& what = "");
     };
   
   private:
     class internals;
     internals* Internals;
   };
   }
   
   #endif
