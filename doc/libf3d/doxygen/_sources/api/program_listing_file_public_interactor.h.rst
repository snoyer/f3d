
.. _program_listing_file_public_interactor.h:

Program Listing for File interactor.h
=====================================

|exhale_lsh| :ref:`Return to documentation for file <file_public_interactor.h>` (``public/interactor.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef f3d_interactor_h
   #define f3d_interactor_h
   
   #include "export.h"
   #include "options.h"
   #include "window.h"
   
   #include <functional>
   #include <string>
   #include <utility>
   #include <vector>
   
   namespace f3d
   {
   class F3D_EXPORT interactor
   {
   public:
   
     virtual interactor& addCommandCallback(
       std::string action, std::function<bool(const std::vector<std::string>&)> callback) = 0;
   
     virtual interactor& removeCommandCallback(const std::string& action) = 0;
   
     virtual bool triggerCommand(std::string_view command) = 0;
   
   
     enum class ModifierKeys : unsigned char
     {
       ANY = 0x80,      // 10000000
       NONE = 0x0,      // 00000000
       CTRL = 0x1,      // 00000001
       SHIFT = 0x2,     // 00000010
       CTRL_SHIFT = 0x3 // 00000011
     };
   
     virtual interactor& addInteractionCommands(
       std::string interaction, ModifierKeys modifiers, std::vector<std::string> commands) = 0;
   
     virtual interactor& addInteractionCommand(
       std::string interaction, ModifierKeys modifiers, std::string command) = 0;
   
     virtual interactor& removeInteractionCommands(
       std::string interaction, ModifierKeys modifiers) = 0;
   
     virtual unsigned long createTimerCallBack(double time, std::function<void()> callBack) = 0;
   
     virtual void removeTimerCallBack(unsigned long id) = 0;
   
   
     virtual void toggleAnimation() = 0;
     virtual void startAnimation() = 0;
     virtual void stopAnimation() = 0;
     virtual bool isPlayingAnimation() = 0;
   
   
     virtual void enableCameraMovement() = 0;
     virtual void disableCameraMovement() = 0;
   
     virtual bool playInteraction(const std::string& file) = 0;
   
     virtual bool recordInteraction(const std::string& file) = 0;
   
     virtual void start() = 0;
   
     virtual void stop() = 0;
   
     static const std::vector<std::pair<std::string, std::string>>& getDefaultInteractionsInfo();
   
   protected:
     interactor() = default;
     virtual ~interactor() = default;
     interactor(const interactor& opt) = delete;
     interactor(interactor&& opt) = delete;
     interactor& operator=(const interactor& opt) = delete;
     interactor& operator=(interactor&& opt) = delete;
   };
   }
   
   #endif
