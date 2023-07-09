#!/bin/bash

qu3e_sources="src/broadphase/*.cpp src/collision/*.cpp src/common/*.cpp src/dynamics/*.cpp src/math/*.cpp src/scene/*.cpp" 
demo_srcs="demo/demo.cpp demo/gl.c"
imgui_srcs="imgui/*.cpp imgui/backends/imgui_impl_glfw.cpp imgui/backends/imgui_impl_opengl3.cpp"
all_srcs="$qu3e_sources $demo_srcs $imgui_srcs"

libs=" -lglfw -lunwind -ldw"
include_dirs="-I. -Iimgui"
flags="-std=c++20  -Wno-format-security"

cc=g++
obj_dir=obj-cache

exe=qu3e_demo

mkdir -p $obj_dir

all_objs=""

for src_file in $all_srcs
do
  obj_file="$obj_dir/$src_file.o"
  all_objs="$all_objs $obj_file"
  mkdir -p $(dirname $obj_file)

  file_deps=$($cc -MM $src_file $include_dirs | tr "\\\\\n" " " | cut -d " " -f3-)

  should_recompile=false
  if [ "$src_file" -nt "$obj_file" ]; then
    should_recompile=true
  else for dep in $file_deps; do
    if [ "$dep" -nt "$obj_file" ]; then
      should_recompile=true
    fi
    done
  fi

  if [ $should_recompile = true ]; then
    echo "compiling $src_file..."
    $cc -c $src_file -o $obj_file $libs $include_dirs $flags
  fi
done

echo "linking..."
$cc -fuse-ld=mold -o $exe $all_objs $libs $include_dirs $flags

echo "done. built executable '$exe'"
