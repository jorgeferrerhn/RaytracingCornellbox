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
CMAKE_SOURCE_DIR = /home/jorge/Escritorio/RaytracingCornellbox

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jorge/Escritorio/RaytracingCornellbox/build

# Include any dependencies generated for this target.
include exts/tinyexr/CMakeFiles/tinyexr.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include exts/tinyexr/CMakeFiles/tinyexr.dir/compiler_depend.make

# Include the progress variables for this target.
include exts/tinyexr/CMakeFiles/tinyexr.dir/progress.make

# Include the compile flags for this target's objects.
include exts/tinyexr/CMakeFiles/tinyexr.dir/flags.make

exts/tinyexr/CMakeFiles/tinyexr.dir/tinyexr/tinyexr.cpp.o: exts/tinyexr/CMakeFiles/tinyexr.dir/flags.make
exts/tinyexr/CMakeFiles/tinyexr.dir/tinyexr/tinyexr.cpp.o: /home/jorge/Escritorio/RaytracingCornellbox/exts/tinyexr/tinyexr/tinyexr.cpp
exts/tinyexr/CMakeFiles/tinyexr.dir/tinyexr/tinyexr.cpp.o: exts/tinyexr/CMakeFiles/tinyexr.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jorge/Escritorio/RaytracingCornellbox/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object exts/tinyexr/CMakeFiles/tinyexr.dir/tinyexr/tinyexr.cpp.o"
	cd /home/jorge/Escritorio/RaytracingCornellbox/build/exts/tinyexr && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT exts/tinyexr/CMakeFiles/tinyexr.dir/tinyexr/tinyexr.cpp.o -MF CMakeFiles/tinyexr.dir/tinyexr/tinyexr.cpp.o.d -o CMakeFiles/tinyexr.dir/tinyexr/tinyexr.cpp.o -c /home/jorge/Escritorio/RaytracingCornellbox/exts/tinyexr/tinyexr/tinyexr.cpp

exts/tinyexr/CMakeFiles/tinyexr.dir/tinyexr/tinyexr.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tinyexr.dir/tinyexr/tinyexr.cpp.i"
	cd /home/jorge/Escritorio/RaytracingCornellbox/build/exts/tinyexr && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jorge/Escritorio/RaytracingCornellbox/exts/tinyexr/tinyexr/tinyexr.cpp > CMakeFiles/tinyexr.dir/tinyexr/tinyexr.cpp.i

exts/tinyexr/CMakeFiles/tinyexr.dir/tinyexr/tinyexr.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tinyexr.dir/tinyexr/tinyexr.cpp.s"
	cd /home/jorge/Escritorio/RaytracingCornellbox/build/exts/tinyexr && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jorge/Escritorio/RaytracingCornellbox/exts/tinyexr/tinyexr/tinyexr.cpp -o CMakeFiles/tinyexr.dir/tinyexr/tinyexr.cpp.s

# Object files for target tinyexr
tinyexr_OBJECTS = \
"CMakeFiles/tinyexr.dir/tinyexr/tinyexr.cpp.o"

# External object files for target tinyexr
tinyexr_EXTERNAL_OBJECTS =

/home/jorge/Escritorio/RaytracingCornellbox/bin/libtinyexr.a: exts/tinyexr/CMakeFiles/tinyexr.dir/tinyexr/tinyexr.cpp.o
/home/jorge/Escritorio/RaytracingCornellbox/bin/libtinyexr.a: exts/tinyexr/CMakeFiles/tinyexr.dir/build.make
/home/jorge/Escritorio/RaytracingCornellbox/bin/libtinyexr.a: exts/tinyexr/CMakeFiles/tinyexr.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jorge/Escritorio/RaytracingCornellbox/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library /home/jorge/Escritorio/RaytracingCornellbox/bin/libtinyexr.a"
	cd /home/jorge/Escritorio/RaytracingCornellbox/build/exts/tinyexr && $(CMAKE_COMMAND) -P CMakeFiles/tinyexr.dir/cmake_clean_target.cmake
	cd /home/jorge/Escritorio/RaytracingCornellbox/build/exts/tinyexr && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/tinyexr.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
exts/tinyexr/CMakeFiles/tinyexr.dir/build: /home/jorge/Escritorio/RaytracingCornellbox/bin/libtinyexr.a
.PHONY : exts/tinyexr/CMakeFiles/tinyexr.dir/build

exts/tinyexr/CMakeFiles/tinyexr.dir/clean:
	cd /home/jorge/Escritorio/RaytracingCornellbox/build/exts/tinyexr && $(CMAKE_COMMAND) -P CMakeFiles/tinyexr.dir/cmake_clean.cmake
.PHONY : exts/tinyexr/CMakeFiles/tinyexr.dir/clean

exts/tinyexr/CMakeFiles/tinyexr.dir/depend:
	cd /home/jorge/Escritorio/RaytracingCornellbox/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jorge/Escritorio/RaytracingCornellbox /home/jorge/Escritorio/RaytracingCornellbox/exts/tinyexr /home/jorge/Escritorio/RaytracingCornellbox/build /home/jorge/Escritorio/RaytracingCornellbox/build/exts/tinyexr /home/jorge/Escritorio/RaytracingCornellbox/build/exts/tinyexr/CMakeFiles/tinyexr.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : exts/tinyexr/CMakeFiles/tinyexr.dir/depend

