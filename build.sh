#!/bin/bash

qu3e_sources="src/broadphase/*.cpp src/collision/*.cpp src/common/*.cpp src/dynamics/*.cpp src/math/*.cpp src/scene/*.cpp" 
demo_srcs="demo/demo.cpp demo/gl.c"
imgui_srcs="imgui/*.cpp imgui/backends/imgui_impl_glfw.cpp imgui/backends/imgui_impl_opengl3.cpp"

libs=" -lglfw -lunwind -ldw"
include_dirs=" -I. -Iimgui"
flags="-std=c++20  -Wno-format-security"

g++ -o qu3e_demo $demo_srcs $qu3e_sources $imgui_srcs $libs $include_dirs $flags
