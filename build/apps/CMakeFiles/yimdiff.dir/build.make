# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.25

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /snap/cmake/1216/bin/cmake

# The command to remove a file.
RM = /snap/cmake/1216/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/jorge/Escritorio/code

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jorge/Escritorio/code/build

# Include any dependencies generated for this target.
include apps/CMakeFiles/yimdiff.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include apps/CMakeFiles/yimdiff.dir/compiler_depend.make

# Include the progress variables for this target.
include apps/CMakeFiles/yimdiff.dir/progress.make

# Include the compile flags for this target's objects.
include apps/CMakeFiles/yimdiff.dir/flags.make

apps/CMakeFiles/yimdiff.dir/yimdiff.cpp.o: apps/CMakeFiles/yimdiff.dir/flags.make
apps/CMakeFiles/yimdiff.dir/yimdiff.cpp.o: /home/jorge/Escritorio/code/apps/yimdiff.cpp
apps/CMakeFiles/yimdiff.dir/yimdiff.cpp.o: apps/CMakeFiles/yimdiff.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jorge/Escritorio/code/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object apps/CMakeFiles/yimdiff.dir/yimdiff.cpp.o"
	cd /home/jorge/Escritorio/code/build/apps && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT apps/CMakeFiles/yimdiff.dir/yimdiff.cpp.o -MF CMakeFiles/yimdiff.dir/yimdiff.cpp.o.d -o CMakeFiles/yimdiff.dir/yimdiff.cpp.o -c /home/jorge/Escritorio/code/apps/yimdiff.cpp

apps/CMakeFiles/yimdiff.dir/yimdiff.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/yimdiff.dir/yimdiff.cpp.i"
	cd /home/jorge/Escritorio/code/build/apps && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jorge/Escritorio/code/apps/yimdiff.cpp > CMakeFiles/yimdiff.dir/yimdiff.cpp.i

apps/CMakeFiles/yimdiff.dir/yimdiff.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/yimdiff.dir/yimdiff.cpp.s"
	cd /home/jorge/Escritorio/code/build/apps && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jorge/Escritorio/code/apps/yimdiff.cpp -o CMakeFiles/yimdiff.dir/yimdiff.cpp.s

# Object files for target yimdiff
yimdiff_OBJECTS = \
"CMakeFiles/yimdiff.dir/yimdiff.cpp.o"

# External object files for target yimdiff
yimdiff_EXTERNAL_OBJECTS =

/home/jorge/Escritorio/code/bin/yimdiff: apps/CMakeFiles/yimdiff.dir/yimdiff.cpp.o
/home/jorge/Escritorio/code/bin/yimdiff: apps/CMakeFiles/yimdiff.dir/build.make
/home/jorge/Escritorio/code/bin/yimdiff: /home/jorge/Escritorio/code/bin/libyocto.a
/home/jorge/Escritorio/code/bin/yimdiff: /home/jorge/Escritorio/code/bin/libstb_image.a
/home/jorge/Escritorio/code/bin/yimdiff: /home/jorge/Escritorio/code/bin/libtinyexr.a
/home/jorge/Escritorio/code/bin/yimdiff: /home/jorge/Escritorio/code/bin/libcgltf.a
/home/jorge/Escritorio/code/bin/yimdiff: /home/jorge/Escritorio/code/bin/libimgui.a
/home/jorge/Escritorio/code/bin/yimdiff: /home/jorge/Escritorio/code/bin/libglad.a
/home/jorge/Escritorio/code/bin/yimdiff: /home/jorge/Escritorio/code/bin/libglfw3.a
/home/jorge/Escritorio/code/bin/yimdiff: /usr/lib/x86_64-linux-gnu/librt.a
/home/jorge/Escritorio/code/bin/yimdiff: /usr/lib/x86_64-linux-gnu/libm.so
/home/jorge/Escritorio/code/bin/yimdiff: /usr/lib/x86_64-linux-gnu/libX11.so
/home/jorge/Escritorio/code/bin/yimdiff: /usr/lib/x86_64-linux-gnu/libGL.so
/home/jorge/Escritorio/code/bin/yimdiff: apps/CMakeFiles/yimdiff.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jorge/Escritorio/code/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/jorge/Escritorio/code/bin/yimdiff"
	cd /home/jorge/Escritorio/code/build/apps && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/yimdiff.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
apps/CMakeFiles/yimdiff.dir/build: /home/jorge/Escritorio/code/bin/yimdiff
.PHONY : apps/CMakeFiles/yimdiff.dir/build

apps/CMakeFiles/yimdiff.dir/clean:
	cd /home/jorge/Escritorio/code/build/apps && $(CMAKE_COMMAND) -P CMakeFiles/yimdiff.dir/cmake_clean.cmake
.PHONY : apps/CMakeFiles/yimdiff.dir/clean

apps/CMakeFiles/yimdiff.dir/depend:
	cd /home/jorge/Escritorio/code/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jorge/Escritorio/code /home/jorge/Escritorio/code/apps /home/jorge/Escritorio/code/build /home/jorge/Escritorio/code/build/apps /home/jorge/Escritorio/code/build/apps/CMakeFiles/yimdiff.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : apps/CMakeFiles/yimdiff.dir/depend

