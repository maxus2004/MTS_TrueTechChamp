cmake_minimum_required(VERSION 3.16)
project(fake_navigation VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(ExternalProject)

add_executable(fake_navigation main.cpp utils.h utils.cpp movement.h movement.cpp)

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
        GIT_TAG 4.10.0
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
set(OpenCV_LIBS
    "${OPENCV_DIR}/lib/libopencv_core.so"
    "${OPENCV_DIR}/lib/libopencv_imgproc.so"
    "${OPENCV_DIR}/lib/libopencv_highgui.so"
    "${OPENCV_DIR}/lib/libopencv_imgcodecs.so"
)
target_link_libraries(fake_navigation PRIVATE ${OpenCV_LIBS})

set_target_properties(fake_navigation PROPERTIES
    BUILD_RPATH "${OPENCV_DIR}/lib"
    INSTALL_RPATH "${OPENCV_DIR}/lib"
)
