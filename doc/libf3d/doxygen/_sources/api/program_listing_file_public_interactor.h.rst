
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
   
   struct interaction_bind_t
   {
     enum class ModifierKeys : unsigned char
     {
       ANY = 0x80,      // 10000000
       NONE = 0x0,      // 00000000
       CTRL = 0x1,      // 00000001
       SHIFT = 0x2,     // 00000010
       CTRL_SHIFT = 0x3 // 00000011
     };
   
     ModifierKeys mod;
     std::string inter;
   
     bool operator<(const interaction_bind_t& bind) const
     {
       return this->mod < bind.mod || (this->mod == bind.mod && this->inter < bind.inter);
     }
   
     bool operator==(const interaction_bind_t& bind) const
     {
       return this->mod == bind.mod && this->inter == bind.inter;
     }
   
     std::string format() const
     {
       switch (this->mod)
       {
         case ModifierKeys::CTRL_SHIFT:
           return "Ctrl+Shift+" + this->inter;
         case ModifierKeys::CTRL:
           return "Ctrl+" + this->inter;
         case ModifierKeys::SHIFT:
           return "Shift+" + this->inter;
         case ModifierKeys::ANY:
           return "Any+" + this->inter;
         default:
           // No need to check for NONE
           return this->inter;
       }
     }
   };
   
   class F3D_EXPORT interactor
   {
   public:
   
     virtual interactor& initCommands() = 0;
   
     virtual interactor& addCommand(
       const std::string& action, std::function<void(const std::vector<std::string>&)> callback) = 0;
   
     virtual interactor& removeCommand(const std::string& action) = 0;
   
     virtual std::vector<std::string> getCommandActions() const = 0;
   
     virtual bool triggerCommand(std::string_view command) = 0;
   
     using documentation_callback_t = std::function<std::pair<std::string, std::string>()>;
   
     virtual interactor& initBindings() = 0;
   
     virtual interactor& addBinding(const interaction_bind_t& bind, std::vector<std::string> commands,
       std::string group = {}, documentation_callback_t documentationCallback = nullptr) = 0;
   
     virtual interactor& addBinding(const interaction_bind_t& bind, std::string command,
       std::string group = {}, documentation_callback_t documentationCallback = nullptr) = 0;
   
     interactor& addBinding(const interaction_bind_t& bind, std::initializer_list<std::string> list,
       std::string group = {}, documentation_callback_t documentationCallback = nullptr)
     {
       return this->addBinding(
         bind, std::vector<std::string>(list), std::move(group), std::move(documentationCallback));
     }
   
     virtual interactor& removeBinding(const interaction_bind_t& bind) = 0;
   
     virtual std::vector<std::string> getBindGroups() const = 0;
   
     virtual std::vector<interaction_bind_t> getBindsForGroup(std::string group) const = 0;
   
     virtual std::vector<interaction_bind_t> getBinds() const = 0;
   
     virtual std::pair<std::string, std::string> getBindingDocumentation(
       const interaction_bind_t& bind) const = 0;
   
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
   
     struct already_exists_exception : public exception
     {
       explicit already_exists_exception(const std::string& what = "");
     };
   
     struct does_not_exists_exception : public exception
     {
       explicit does_not_exists_exception(const std::string& what = "");
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
