list(APPEND libf3dSDKTests_list
     TestSDKRenderAndInteract.cxx
     TestSDKEngine.cxx
     TestSDKEngineExceptions.cxx
     TestSDKOptions.cxx
     TestSDKLog.cxx
     TestSDKLoader.cxx
     TestSDKCamera.cxx
    )

# Test image comparison only with VTK > 9.0.1
if(VTK_VERSION VERSION_GREATER 9.0.1)
  list(APPEND libf3dSDKTests_list
    TestSDKCompareWithFile.cxx
    TestSDKDropZone.cxx
    TestSDKRenderFinalShader.cxx
    TestSDKImage.cxx
    TestSDKLoadFromMemory.cxx
    TestSDKMultiColoring.cxx
    TestSDKMultiOptions.cxx
    TestSDKWindowNative.cxx
    TestSDKDynamicBackgroundColor.cxx
    TestSDKDynamicFontFile.cxx
    TestSDKDynamicLightIntensity.cxx
    TestSDKDynamicProperties.cxx
    )
  if(NOT F3D_EXCLUDE_DEPRECATED)
    list(APPEND libf3dSDKTests_list
         TestSDKImageDeprecated.cxx)
  endif()
endif()

# Animation needs https://gitlab.kitware.com/vtk/vtk/-/merge_requests/7246
if(VTK_VERSION VERSION_GREATER_EQUAL 9.0.20201016)
  list(APPEND libf3dSDKTests_list
    TestSDKAnimation.cxx
    )
endif()

# HDRI test needs https://gitlab.kitware.com/vtk/vtk/-/merge_requests/9767
if(VTK_VERSION VERSION_GREATER_EQUAL 9.2.20221220)
  list(APPEND libf3dSDKTests_list
    TestSDKDynamicHDRI.cxx
    )
endif()

# Drop file test needs https://gitlab.kitware.com/vtk/vtk/-/merge_requests/9199
if(VTK_VERSION VERSION_GREATER_EQUAL 9.1.20220519)
  list(APPEND libf3dSDKTests_list
    TestSDKInteractorCallBack.cxx
    TestSDKInteractorDropFullScene.cxx
    )
endif()

# Configure the log file for dropfile test
configure_file("${F3D_SOURCE_DIR}/testing/recordings/TestSDKInteractorCallBack.log.in"
               "${CMAKE_BINARY_DIR}/TestSDKInteractorCallBack.log") # Dragon.vtu; S
configure_file("${F3D_SOURCE_DIR}/testing/recordings/TestSDKInteractorDropFullScene.log.in"
               "${CMAKE_BINARY_DIR}/TestSDKInteractorDropFullScene.log") # world.obj; S

# External window tests
if(F3D_MODULE_EXTERNAL_RENDERING AND NOT VTK_OPENGL_HAS_EGL AND VTK_VERSION VERSION_GREATER 9.0.1)
  find_package(glfw3 QUIET)
  if(glfw3_FOUND)
    list(APPEND libf3dSDKTests_list
        TestSDKExternalWindowGLFW.cxx
        )
  endif()
  find_package(Qt5 QUIET COMPONENTS OpenGL)
  if(Qt5_FOUND)
    list(APPEND libf3dSDKTests_list
        TestSDKExternalWindowQT.cxx
        )
  endif()
endif()

# create the testing file and list of tests
# CMake variables are set to work around this issue:
# https://gitlab.kitware.com/cmake/cmake/-/issues/21049
set(CMAKE_TESTDRIVER_BEFORE_TESTMAIN "f3d::engine::autoloadPlugins();")
set(CMAKE_TESTDRIVER_AFTER_TESTMAIN "")
set(CMAKE_TESTDRIVER_ARGVC_FUNCTION "")
create_test_sourcelist(_libf3dSDKTests libf3dSDKTests.cxx ${libf3dSDKTests_list} EXTRA_INCLUDE engine.h)

# add the executable
add_executable(libf3dSDKTests ${_libf3dSDKTests})
set_target_properties(libf3dSDKTests PROPERTIES
  CXX_VISIBILITY_PRESET hidden
  )

target_compile_definitions(libf3dSDKTests PRIVATE F3D_MODULE_EXR=$<BOOL:${F3D_MODULE_EXR}>)

# List tests that do not require rendering
list(APPEND libf3dSDKTestsNoRender_list
     TestSDKEngineExceptions
     TestSDKOptions
     TestSDKLog
     TestSDKLoader)

# Add all the ADD_TEST for each test
foreach (test ${libf3dSDKTests_list})
  get_filename_component (TName ${test} NAME_WE)
  add_test (NAME libf3d::${TName} COMMAND libf3dSDKTests ${TName}  "${F3D_SOURCE_DIR}/testing/" "${CMAKE_BINARY_DIR}/Testing/Temporary/" "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")
  set_tests_properties(libf3d::${TName} PROPERTIES TIMEOUT 30)
  if (NOT F3D_TESTING_ENABLE_RENDERING_TESTS AND NOT ${TName} IN_LIST libf3dSDKTestsNoRender_list)
    set_tests_properties(libf3d::${TName} PROPERTIES DISABLED ON)
  endif ()
endforeach ()

# Specific test properties to check

# CTest can't check colored output as it contains special chars
set_tests_properties(libf3d::TestSDKLog PROPERTIES PASS_REGULAR_EXPRESSION
"\
Test Info\nTest Warning\nTest Error\n\
Test Info\nTest Warning\nTest Error\n\
Test Warning\nTest Error\nTest Error\n\
Test Print Debug\nTest Print Info\nTest Print Warning\nTest Print Error\n\
Test Debug Coloring")

if(VTK_VERSION VERSION_GREATER_EQUAL 9.2.20221220)
  set_tests_properties(libf3d::TestSDKDynamicHDRI PROPERTIES TIMEOUT 90)
  if(NOT F3D_TESTING_ENABLE_LONG_TIMEOUT_TESTS)
    set_tests_properties(libf3d::TestSDKDynamicHDRI PROPERTIES DISABLED ON)
  endif()
endif()

target_link_libraries(libf3dSDKTests libf3d)

if(glfw3_FOUND)
  # external window test using glfw
  target_link_libraries(libf3dSDKTests glfw)
endif()

if(Qt5_FOUND)
  # external window test using QT
  target_link_libraries(libf3dSDKTests Qt5::OpenGL)
endif()

# make sure the libf3d API is compatible with C++11
set_target_properties(libf3dSDKTests PROPERTIES CXX_STANDARD 11)
