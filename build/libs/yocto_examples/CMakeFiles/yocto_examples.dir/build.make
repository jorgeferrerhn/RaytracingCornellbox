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
include libs/yocto_examples/CMakeFiles/yocto_examples.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include libs/yocto_examples/CMakeFiles/yocto_examples.dir/compiler_depend.make

# Include the progress variables for this target.
include libs/yocto_examples/CMakeFiles/yocto_examples.dir/progress.make

# Include the compile flags for this target's objects.
include libs/yocto_examples/CMakeFiles/yocto_examples.dir/flags.make

libs/yocto_examples/CMakeFiles/yocto_examples.dir/yocto_grade.cpp.o: libs/yocto_examples/CMakeFiles/yocto_examples.dir/flags.make
libs/yocto_examples/CMakeFiles/yocto_examples.dir/yocto_grade.cpp.o: /home/jorge/Escritorio/code/libs/yocto_examples/yocto_grade.cpp
libs/yocto_examples/CMakeFiles/yocto_examples.dir/yocto_grade.cpp.o: libs/yocto_examples/CMakeFiles/yocto_examples.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jorge/Escritorio/code/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object libs/yocto_examples/CMakeFiles/yocto_examples.dir/yocto_grade.cpp.o"
	cd /home/jorge/Escritorio/code/build/libs/yocto_examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT libs/yocto_examples/CMakeFiles/yocto_examples.dir/yocto_grade.cpp.o -MF CMakeFiles/yocto_examples.dir/yocto_grade.cpp.o.d -o CMakeFiles/yocto_examples.dir/yocto_grade.cpp.o -c /home/jorge/Escritorio/code/libs/yocto_examples/yocto_grade.cpp

libs/yocto_examples/CMakeFiles/yocto_examples.dir/yocto_grade.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/yocto_examples.dir/yocto_grade.cpp.i"
	cd /home/jorge/Escritorio/code/build/libs/yocto_examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jorge/Escritorio/code/libs/yocto_examples/yocto_grade.cpp > CMakeFiles/yocto_examples.dir/yocto_grade.cpp.i

libs/yocto_examples/CMakeFiles/yocto_examples.dir/yocto_grade.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/yocto_examples.dir/yocto_grade.cpp.s"
	cd /home/jorge/Escritorio/code/build/libs/yocto_examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jorge/Escritorio/code/libs/yocto_examples/yocto_grade.cpp -o CMakeFiles/yocto_examples.dir/yocto_grade.cpp.s

libs/yocto_examples/CMakeFiles/yocto_examples.dir/yocto_raytrace.cpp.o: libs/yocto_examples/CMakeFiles/yocto_examples.dir/flags.make
libs/yocto_examples/CMakeFiles/yocto_examples.dir/yocto_raytrace.cpp.o: /home/jorge/Escritorio/code/libs/yocto_examples/yocto_raytrace.cpp
libs/yocto_examples/CMakeFiles/yocto_examples.dir/yocto_raytrace.cpp.o: libs/yocto_examples/CMakeFiles/yocto_examples.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jorge/Escritorio/code/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object libs/yocto_examples/CMakeFiles/yocto_examples.dir/yocto_raytrace.cpp.o"
	cd /home/jorge/Escritorio/code/build/libs/yocto_examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT libs/yocto_examples/CMakeFiles/yocto_examples.dir/yocto_raytrace.cpp.o -MF CMakeFiles/yocto_examples.dir/yocto_raytrace.cpp.o.d -o CMakeFiles/yocto_examples.dir/yocto_raytrace.cpp.o -c /home/jorge/Escritorio/code/libs/yocto_examples/yocto_raytrace.cpp

libs/yocto_examples/CMakeFiles/yocto_examples.dir/yocto_raytrace.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/yocto_examples.dir/yocto_raytrace.cpp.i"
	cd /home/jorge/Escritorio/code/build/libs/yocto_examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jorge/Escritorio/code/libs/yocto_examples/yocto_raytrace.cpp > CMakeFiles/yocto_examples.dir/yocto_raytrace.cpp.i

libs/yocto_examples/CMakeFiles/yocto_examples.dir/yocto_raytrace.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/yocto_examples.dir/yocto_raytrace.cpp.s"
	cd /home/jorge/Escritorio/code/build/libs/yocto_examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jorge/Escritorio/code/libs/yocto_examples/yocto_raytrace.cpp -o CMakeFiles/yocto_examples.dir/yocto_raytrace.cpp.s

# Object files for target yocto_examples
yocto_examples_OBJECTS = \
"CMakeFiles/yocto_examples.dir/yocto_grade.cpp.o" \
"CMakeFiles/yocto_examples.dir/yocto_raytrace.cpp.o"

# External object files for target yocto_examples
yocto_examples_EXTERNAL_OBJECTS =

/home/jorge/Escritorio/code/bin/libyocto_examples.a: libs/yocto_examples/CMakeFiles/yocto_examples.dir/yocto_grade.cpp.o
/home/jorge/Escritorio/code/bin/libyocto_examples.a: libs/yocto_examples/CMakeFiles/yocto_examples.dir/yocto_raytrace.cpp.o
/home/jorge/Escritorio/code/bin/libyocto_examples.a: libs/yocto_examples/CMakeFiles/yocto_examples.dir/build.make
/home/jorge/Escritorio/code/bin/libyocto_examples.a: libs/yocto_examples/CMakeFiles/yocto_examples.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jorge/Escritorio/code/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX static library /home/jorge/Escritorio/code/bin/libyocto_examples.a"
	cd /home/jorge/Escritorio/code/build/libs/yocto_examples && $(CMAKE_COMMAND) -P CMakeFiles/yocto_examples.dir/cmake_clean_target.cmake
	cd /home/jorge/Escritorio/code/build/libs/yocto_examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/yocto_examples.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
libs/yocto_examples/CMakeFiles/yocto_examples.dir/build: /home/jorge/Escritorio/code/bin/libyocto_examples.a
.PHONY : libs/yocto_examples/CMakeFiles/yocto_examples.dir/build

libs/yocto_examples/CMakeFiles/yocto_examples.dir/clean:
	cd /home/jorge/Escritorio/code/build/libs/yocto_examples && $(CMAKE_COMMAND) -P CMakeFiles/yocto_examples.dir/cmake_clean.cmake
.PHONY : libs/yocto_examples/CMakeFiles/yocto_examples.dir/clean

libs/yocto_examples/CMakeFiles/yocto_examples.dir/depend:
	cd /home/jorge/Escritorio/code/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jorge/Escritorio/code /home/jorge/Escritorio/code/libs/yocto_examples /home/jorge/Escritorio/code/build /home/jorge/Escritorio/code/build/libs/yocto_examples /home/jorge/Escritorio/code/build/libs/yocto_examples/CMakeFiles/yocto_examples.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : libs/yocto_examples/CMakeFiles/yocto_examples.dir/depend
