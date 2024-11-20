
.. _program_listing_file_public_interactor.h:

Program Listing for File interactor.h
=====================================

|exhale_lsh| :ref:`Return to documentation for file <file_public_interactor.h>` (``public/interactor.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef f3d_interactor_h
   #define f3d_interactor_h
   
   #include "exception.h"
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
   
     virtual interactor& initCommands() = 0;
   
     virtual interactor& addCommand(
       const std::string& action, std::function<void(const std::vector<std::string>&)> callback) = 0;
   
     virtual interactor& removeCommand(const std::string& action) = 0;
   
     virtual std::vector<std::string> getCommandActions() const = 0;
   
     virtual bool triggerCommand(std::string_view command) = 0;
   
   
     enum class ModifierKeys : unsigned char
     {
       ANY = 0x80,      // 10000000
       NONE = 0x0,      // 00000000
       CTRL = 0x1,      // 00000001
       SHIFT = 0x2,     // 00000010
       CTRL_SHIFT = 0x3 // 00000011
     };
   
     virtual interactor& initBindings() = 0;
   
     virtual interactor& addBinding(
       const std::string& interaction, ModifierKeys modifiers, std::vector<std::string> commands) = 0;
   
     virtual interactor& addBinding(
       const std::string& interaction, ModifierKeys modifiers, std::string command) = 0;
   
     interactor& addBinding(
       const std::string& interaction, ModifierKeys modifiers, std::initializer_list<std::string> list)
     {
       return this->addBinding(interaction, modifiers, std::vector<std::string>(list));
     }
   
     virtual interactor& removeBinding(std::string interaction, ModifierKeys modifiers) = 0;
   
     virtual std::vector<std::pair<std::string, ModifierKeys>> getBindingInteractions() const = 0;
   
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
   
     struct already_exists_exception : public exception
     {
       explicit already_exists_exception(const std::string& what = "");
     };
   
     struct command_runtime_exception : public exception
     {
       explicit command_runtime_exception(const std::string& what = "");
     };
   
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
