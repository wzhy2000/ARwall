# For more information about using CMake with Android Studio, read the
# documentation: https://d.android.com/studio/projects/add-native-code.html

set(pathToOpenCv C:/work/Japan/OpenCV-android-sdk/sdk)
set(pathJniLibs C:/work/Japan/ARwall/app/src/main/jniLibs)
set(pathCommonm C:/work/Japan/ARwall/common)

# Sets the minimum version of CMake required to build the native library.

cmake_minimum_required(VERSION 3.4.1)

get_filename_component(ndkHelperSrc ${pathCommonm}/ndk_helper ABSOLUTE)
add_subdirectory(${ndkHelperSrc}
        ${pathCommonm}/ndkHelperBin/${CMAKE_BUILD_TYPE}/${ANDROID_ABI})

include_directories(nativeCode/common)
include_directories(nativeCode/simpleARClass)
include_directories(../externals/glm-0.9.7.5)
include_directories(${pathToOpenCv}/native/jni/include)
include_directories(${CMAKE_ANDROID_NDK}/sources/android/ndk_helper)

# add_library( libopencv SHARED IMPORTED)
# set_target_properties(libopencv PROPERTIES IMPORTED_LOCATION
#           ${pathJniLibs}/opencv/${ANDROID_ABI}/libopencv_java3.so)

find_package(OpenCV 3.4 REQUIRED java)

# Creates and names a library, sets it as either STATIC
# or SHARED, and provides the relative paths to its source code.
# You can define multiple libraries, and CMake builds them for you.
# Gradle automatically packages shared libraries with your APK.

add_library( # Sets the name of the library.
        native-lib

        # Sets the library as a shared library.
        SHARED

        # Provides a relative path to your source file(s).
        native-lib.cpp
        jniCalls/MainActivity.cpp
        jniCalls/cameraClass.cpp
        jniCalls/gestureClass.cpp
        jniCalls/myGLRenderer.cpp
        jniCalls/sensorClass.cpp
        nativeCode/simpleARClass/simpleARClass.cpp
        nativeCode/common/backTexture.cpp
        nativeCode/common/textPicture.cpp
        nativeCode/common/misc.cpp
        nativeCode/common/myGLCamera.cpp
        nativeCode/common/myGLFunctions.cpp
        nativeCode/Common/myJNIHelper.cpp
        nativeCode/common/myShader.cpp)

# Searches for a specified prebuilt library and stores the path as a
# variable. Because CMake includes system libraries in the search path by
# default, you only need to specify the name of the public NDK library
# you want to add. CMake verifies that the library exists before
# completing its build.

find_library( # Sets the name of the path variable.
        log-lib

        # Specifies the name of the NDK library that
        # you want CMake to locate.
        log)

# Specifies libraries CMake should link to your target library. You
# can link multiple libraries, such as libraries you define in this
# build script, prebuilt third-party libraries, or system libraries.

target_link_libraries( # Specifies the target library.
        native-lib
        NdkHelper
        # libopencv
        ${OpenCV_LIBRARIES}
        android
        EGL
        GLESv2
        # Links the target library to the log library
        # included in the NDK.
        ${log-lib})

