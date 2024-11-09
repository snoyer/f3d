
.. _program_listing_file_public_utils.h:

Program Listing for File utils.h
================================

|exhale_lsh| :ref:`Return to documentation for file <file_public_utils.h>` (``public/utils.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef f3d_utils_h
   #define f3d_utils_h
   
   #include "exception.h"
   #include "export.h"
   
   #include <string>
   #include <vector>
   
   namespace f3d
   {
   class F3D_EXPORT utils
   {
   public:
     static unsigned int textDistance(const std::string& strA, const std::string& strB);
   
     // clang-format off
     static std::vector<std::string> tokenize(std::string_view str);
     // clang-format on
   
     struct tokenize_exception : public exception
     {
       explicit tokenize_exception(const std::string& what = "");
     };
   };
   }
   
   #endif
