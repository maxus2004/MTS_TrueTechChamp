cmake_minimum_required(VERSION 3.16)
project(fake_navigation VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(ExternalProject)

add_executable(fake_navigation main.cpp utils.h utils.cpp movement.h movement.cpp path.cpp path.h)

include_directories("deps/asio-1.36.0/include")

include_directories("deps/raylib-5.5_linux_amd64/include")
target_link_libraries(fake_navigation PRIVATE
    ${CMAKE_SOURCE_DIR}/deps/raylib-5.5_linux_amd64/lib/libraylib.a
)

set(OPENCV_DIR "${CMAKE_SOURCE_DIR}/deps/opencv")
if(NOT EXISTS "${OPENCV_DIR}/include/opencv4")
    ExternalProject_Add(
        opencv_project
        GIT_REPOSITORY https://github.com/opencv/opencv.git
        GIT_TAG 4.12.0
        GIT_SHALLOW TRUE
        GIT_PROGRESS TRUE
        PREFIX "${CMAKE_SOURCE_DIR}/../deps/opencv_tmp"
        CMAKE_ARGS
            -DCMAKE_INSTALL_PREFIX=${OPENCV_DIR}
            -DCMAKE_BUILD_TYPE=Release
            -DBUILD_LIST=core,imgproc,highgui,imgcodecs
            -DBUILD_SHARED_LIBS=ON
            -DBUILD_EXAMPLES=OFF
            -DBUILD_TESTS=OFF
            -DBUILD_DOCS=OFF
            -DBUILD_PERF_TESTS=OFF
    )
    add_dependencies(fake_navigation opencv_project)
endif()


include_directories("${OPENCV_DIR}/include/opencv4")

set(OPENCV_CMAKE_DIR "${OPENCV_DIR}/lib/cmake/opencv4")
if(EXISTS "${OPENCV_CMAKE_DIR}/OpenCVConfig.cmake")
    message(STATUS "Using vendored OpenCVConfig at ${OPENCV_CMAKE_DIR}")
    set(OpenCV_DIR "${OPENCV_CMAKE_DIR}" CACHE PATH "OpenCV_DIR" FORCE)
    find_package(OpenCV 4.12 REQUIRED COMPONENTS core imgproc highgui imgcodecs)
    target_link_libraries(fake_navigation PRIVATE
        OpenCV::core
        OpenCV::imgproc
        OpenCV::highgui
        OpenCV::imgcodecs
    )
else()
    message(STATUS "Vendored OpenCV CMake config not found at ${OPENCV_CMAKE_DIR}; using IMPORTED targets fallback.")
    # Fallback
    set(OPENCV_LIB_DIR "${OPENCV_DIR}/lib")
    set(OPENCV_LIB_core "${OPENCV_LIB_DIR}/libopencv_core.so")
    set(OPENCV_LIB_imgproc "${OPENCV_LIB_DIR}/libopencv_imgproc.so")
    set(OPENCV_LIB_highgui "${OPENCV_LIB_DIR}/libopencv_highgui.so")
    set(OPENCV_LIB_imgcodecs "${OPENCV_LIB_DIR}/libopencv_imgcodecs.so")

    function(add_imported name path)
        add_library(${name} SHARED IMPORTED)
        set_target_properties(${name} PROPERTIES IMPORTED_LOCATION "${path}")
    endfunction()

    add_imported(OpenCV::core "${OPENCV_LIB_core}")
    add_imported(OpenCV::imgproc "${OPENCV_LIB_imgproc}")
    add_imported(OpenCV::highgui "${OPENCV_LIB_highgui}")
    add_imported(OpenCV::imgcodecs "${OPENCV_LIB_imgcodecs}")

    target_link_libraries(fake_navigation
        PRIVATE
            OpenCV::core
            OpenCV::imgproc
            OpenCV::highgui
            OpenCV::imgcodecs
    )
endif()

set_target_properties(fake_navigation PROPERTIES
    BUILD_RPATH "${OPENCV_DIR}/lib"
    INSTALL_RPATH "${OPENCV_DIR}/lib"
)
