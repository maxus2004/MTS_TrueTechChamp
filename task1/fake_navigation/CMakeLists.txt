cmake_minimum_required(VERSION 3.10.0)
project(fake_navigation VERSION 0.1.0 LANGUAGES C CXX)

add_executable(fake_navigation main.cpp)

add_library(asio deps/asio-1.36.0/include/asio.hpp)
include_directories(asio PUBLIC "deps/asio-1.36.0/include")
set_target_properties(asio PROPERTIES LINKER_LANGUAGE CXX)

add_library(raylib deps/raylib-5.5_linux_amd64/include/raylib.h)
include_directories(raylib PUBLIC "deps/raylib-5.5_linux_amd64/include")
set_target_properties(raylib PROPERTIES LINKER_LANGUAGE C)
target_link_libraries(fake_navigation PRIVATE ${CMAKE_SOURCE_DIR}/deps/raylib-5.5_linux_amd64/lib/libraylib.a)

link_libraries(fake_navigation PUBLIC raylib)
link_libraries(fake_navigation PUBLIC asio)