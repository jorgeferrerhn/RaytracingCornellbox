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
include exts/imgui/CMakeFiles/imgui.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include exts/imgui/CMakeFiles/imgui.dir/compiler_depend.make

# Include the progress variables for this target.
include exts/imgui/CMakeFiles/imgui.dir/progress.make

# Include the compile flags for this target's objects.
include exts/imgui/CMakeFiles/imgui.dir/flags.make

exts/imgui/CMakeFiles/imgui.dir/imgui/imgui.cpp.o: exts/imgui/CMakeFiles/imgui.dir/flags.make
exts/imgui/CMakeFiles/imgui.dir/imgui/imgui.cpp.o: /home/jorge/Escritorio/code/exts/imgui/imgui/imgui.cpp
exts/imgui/CMakeFiles/imgui.dir/imgui/imgui.cpp.o: exts/imgui/CMakeFiles/imgui.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jorge/Escritorio/code/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object exts/imgui/CMakeFiles/imgui.dir/imgui/imgui.cpp.o"
	cd /home/jorge/Escritorio/code/build/exts/imgui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT exts/imgui/CMakeFiles/imgui.dir/imgui/imgui.cpp.o -MF CMakeFiles/imgui.dir/imgui/imgui.cpp.o.d -o CMakeFiles/imgui.dir/imgui/imgui.cpp.o -c /home/jorge/Escritorio/code/exts/imgui/imgui/imgui.cpp

exts/imgui/CMakeFiles/imgui.dir/imgui/imgui.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/imgui.dir/imgui/imgui.cpp.i"
	cd /home/jorge/Escritorio/code/build/exts/imgui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jorge/Escritorio/code/exts/imgui/imgui/imgui.cpp > CMakeFiles/imgui.dir/imgui/imgui.cpp.i

exts/imgui/CMakeFiles/imgui.dir/imgui/imgui.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/imgui.dir/imgui/imgui.cpp.s"
	cd /home/jorge/Escritorio/code/build/exts/imgui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jorge/Escritorio/code/exts/imgui/imgui/imgui.cpp -o CMakeFiles/imgui.dir/imgui/imgui.cpp.s

exts/imgui/CMakeFiles/imgui.dir/imgui/imgui_draw.cpp.o: exts/imgui/CMakeFiles/imgui.dir/flags.make
exts/imgui/CMakeFiles/imgui.dir/imgui/imgui_draw.cpp.o: /home/jorge/Escritorio/code/exts/imgui/imgui/imgui_draw.cpp
exts/imgui/CMakeFiles/imgui.dir/imgui/imgui_draw.cpp.o: exts/imgui/CMakeFiles/imgui.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jorge/Escritorio/code/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object exts/imgui/CMakeFiles/imgui.dir/imgui/imgui_draw.cpp.o"
	cd /home/jorge/Escritorio/code/build/exts/imgui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT exts/imgui/CMakeFiles/imgui.dir/imgui/imgui_draw.cpp.o -MF CMakeFiles/imgui.dir/imgui/imgui_draw.cpp.o.d -o CMakeFiles/imgui.dir/imgui/imgui_draw.cpp.o -c /home/jorge/Escritorio/code/exts/imgui/imgui/imgui_draw.cpp

exts/imgui/CMakeFiles/imgui.dir/imgui/imgui_draw.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/imgui.dir/imgui/imgui_draw.cpp.i"
	cd /home/jorge/Escritorio/code/build/exts/imgui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jorge/Escritorio/code/exts/imgui/imgui/imgui_draw.cpp > CMakeFiles/imgui.dir/imgui/imgui_draw.cpp.i

exts/imgui/CMakeFiles/imgui.dir/imgui/imgui_draw.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/imgui.dir/imgui/imgui_draw.cpp.s"
	cd /home/jorge/Escritorio/code/build/exts/imgui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jorge/Escritorio/code/exts/imgui/imgui/imgui_draw.cpp -o CMakeFiles/imgui.dir/imgui/imgui_draw.cpp.s

exts/imgui/CMakeFiles/imgui.dir/imgui/imgui_tables.cpp.o: exts/imgui/CMakeFiles/imgui.dir/flags.make
exts/imgui/CMakeFiles/imgui.dir/imgui/imgui_tables.cpp.o: /home/jorge/Escritorio/code/exts/imgui/imgui/imgui_tables.cpp
exts/imgui/CMakeFiles/imgui.dir/imgui/imgui_tables.cpp.o: exts/imgui/CMakeFiles/imgui.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jorge/Escritorio/code/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object exts/imgui/CMakeFiles/imgui.dir/imgui/imgui_tables.cpp.o"
	cd /home/jorge/Escritorio/code/build/exts/imgui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT exts/imgui/CMakeFiles/imgui.dir/imgui/imgui_tables.cpp.o -MF CMakeFiles/imgui.dir/imgui/imgui_tables.cpp.o.d -o CMakeFiles/imgui.dir/imgui/imgui_tables.cpp.o -c /home/jorge/Escritorio/code/exts/imgui/imgui/imgui_tables.cpp

exts/imgui/CMakeFiles/imgui.dir/imgui/imgui_tables.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/imgui.dir/imgui/imgui_tables.cpp.i"
	cd /home/jorge/Escritorio/code/build/exts/imgui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jorge/Escritorio/code/exts/imgui/imgui/imgui_tables.cpp > CMakeFiles/imgui.dir/imgui/imgui_tables.cpp.i

exts/imgui/CMakeFiles/imgui.dir/imgui/imgui_tables.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/imgui.dir/imgui/imgui_tables.cpp.s"
	cd /home/jorge/Escritorio/code/build/exts/imgui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jorge/Escritorio/code/exts/imgui/imgui/imgui_tables.cpp -o CMakeFiles/imgui.dir/imgui/imgui_tables.cpp.s

exts/imgui/CMakeFiles/imgui.dir/imgui/imgui_widgets.cpp.o: exts/imgui/CMakeFiles/imgui.dir/flags.make
exts/imgui/CMakeFiles/imgui.dir/imgui/imgui_widgets.cpp.o: /home/jorge/Escritorio/code/exts/imgui/imgui/imgui_widgets.cpp
exts/imgui/CMakeFiles/imgui.dir/imgui/imgui_widgets.cpp.o: exts/imgui/CMakeFiles/imgui.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jorge/Escritorio/code/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object exts/imgui/CMakeFiles/imgui.dir/imgui/imgui_widgets.cpp.o"
	cd /home/jorge/Escritorio/code/build/exts/imgui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT exts/imgui/CMakeFiles/imgui.dir/imgui/imgui_widgets.cpp.o -MF CMakeFiles/imgui.dir/imgui/imgui_widgets.cpp.o.d -o CMakeFiles/imgui.dir/imgui/imgui_widgets.cpp.o -c /home/jorge/Escritorio/code/exts/imgui/imgui/imgui_widgets.cpp

exts/imgui/CMakeFiles/imgui.dir/imgui/imgui_widgets.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/imgui.dir/imgui/imgui_widgets.cpp.i"
	cd /home/jorge/Escritorio/code/build/exts/imgui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jorge/Escritorio/code/exts/imgui/imgui/imgui_widgets.cpp > CMakeFiles/imgui.dir/imgui/imgui_widgets.cpp.i

exts/imgui/CMakeFiles/imgui.dir/imgui/imgui_widgets.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/imgui.dir/imgui/imgui_widgets.cpp.s"
	cd /home/jorge/Escritorio/code/build/exts/imgui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jorge/Escritorio/code/exts/imgui/imgui/imgui_widgets.cpp -o CMakeFiles/imgui.dir/imgui/imgui_widgets.cpp.s

exts/imgui/CMakeFiles/imgui.dir/imgui/imgui_demo.cpp.o: exts/imgui/CMakeFiles/imgui.dir/flags.make
exts/imgui/CMakeFiles/imgui.dir/imgui/imgui_demo.cpp.o: /home/jorge/Escritorio/code/exts/imgui/imgui/imgui_demo.cpp
exts/imgui/CMakeFiles/imgui.dir/imgui/imgui_demo.cpp.o: exts/imgui/CMakeFiles/imgui.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jorge/Escritorio/code/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object exts/imgui/CMakeFiles/imgui.dir/imgui/imgui_demo.cpp.o"
	cd /home/jorge/Escritorio/code/build/exts/imgui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT exts/imgui/CMakeFiles/imgui.dir/imgui/imgui_demo.cpp.o -MF CMakeFiles/imgui.dir/imgui/imgui_demo.cpp.o.d -o CMakeFiles/imgui.dir/imgui/imgui_demo.cpp.o -c /home/jorge/Escritorio/code/exts/imgui/imgui/imgui_demo.cpp

exts/imgui/CMakeFiles/imgui.dir/imgui/imgui_demo.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/imgui.dir/imgui/imgui_demo.cpp.i"
	cd /home/jorge/Escritorio/code/build/exts/imgui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jorge/Escritorio/code/exts/imgui/imgui/imgui_demo.cpp > CMakeFiles/imgui.dir/imgui/imgui_demo.cpp.i

exts/imgui/CMakeFiles/imgui.dir/imgui/imgui_demo.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/imgui.dir/imgui/imgui_demo.cpp.s"
	cd /home/jorge/Escritorio/code/build/exts/imgui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jorge/Escritorio/code/exts/imgui/imgui/imgui_demo.cpp -o CMakeFiles/imgui.dir/imgui/imgui_demo.cpp.s

exts/imgui/CMakeFiles/imgui.dir/imgui/backends/imgui_impl_glfw.cpp.o: exts/imgui/CMakeFiles/imgui.dir/flags.make
exts/imgui/CMakeFiles/imgui.dir/imgui/backends/imgui_impl_glfw.cpp.o: /home/jorge/Escritorio/code/exts/imgui/imgui/backends/imgui_impl_glfw.cpp
exts/imgui/CMakeFiles/imgui.dir/imgui/backends/imgui_impl_glfw.cpp.o: exts/imgui/CMakeFiles/imgui.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jorge/Escritorio/code/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object exts/imgui/CMakeFiles/imgui.dir/imgui/backends/imgui_impl_glfw.cpp.o"
	cd /home/jorge/Escritorio/code/build/exts/imgui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT exts/imgui/CMakeFiles/imgui.dir/imgui/backends/imgui_impl_glfw.cpp.o -MF CMakeFiles/imgui.dir/imgui/backends/imgui_impl_glfw.cpp.o.d -o CMakeFiles/imgui.dir/imgui/backends/imgui_impl_glfw.cpp.o -c /home/jorge/Escritorio/code/exts/imgui/imgui/backends/imgui_impl_glfw.cpp

exts/imgui/CMakeFiles/imgui.dir/imgui/backends/imgui_impl_glfw.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/imgui.dir/imgui/backends/imgui_impl_glfw.cpp.i"
	cd /home/jorge/Escritorio/code/build/exts/imgui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jorge/Escritorio/code/exts/imgui/imgui/backends/imgui_impl_glfw.cpp > CMakeFiles/imgui.dir/imgui/backends/imgui_impl_glfw.cpp.i

exts/imgui/CMakeFiles/imgui.dir/imgui/backends/imgui_impl_glfw.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/imgui.dir/imgui/backends/imgui_impl_glfw.cpp.s"
	cd /home/jorge/Escritorio/code/build/exts/imgui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jorge/Escritorio/code/exts/imgui/imgui/backends/imgui_impl_glfw.cpp -o CMakeFiles/imgui.dir/imgui/backends/imgui_impl_glfw.cpp.s

exts/imgui/CMakeFiles/imgui.dir/imgui/backends/imgui_impl_opengl3.cpp.o: exts/imgui/CMakeFiles/imgui.dir/flags.make
exts/imgui/CMakeFiles/imgui.dir/imgui/backends/imgui_impl_opengl3.cpp.o: /home/jorge/Escritorio/code/exts/imgui/imgui/backends/imgui_impl_opengl3.cpp
exts/imgui/CMakeFiles/imgui.dir/imgui/backends/imgui_impl_opengl3.cpp.o: exts/imgui/CMakeFiles/imgui.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jorge/Escritorio/code/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object exts/imgui/CMakeFiles/imgui.dir/imgui/backends/imgui_impl_opengl3.cpp.o"
	cd /home/jorge/Escritorio/code/build/exts/imgui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT exts/imgui/CMakeFiles/imgui.dir/imgui/backends/imgui_impl_opengl3.cpp.o -MF CMakeFiles/imgui.dir/imgui/backends/imgui_impl_opengl3.cpp.o.d -o CMakeFiles/imgui.dir/imgui/backends/imgui_impl_opengl3.cpp.o -c /home/jorge/Escritorio/code/exts/imgui/imgui/backends/imgui_impl_opengl3.cpp

exts/imgui/CMakeFiles/imgui.dir/imgui/backends/imgui_impl_opengl3.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/imgui.dir/imgui/backends/imgui_impl_opengl3.cpp.i"
	cd /home/jorge/Escritorio/code/build/exts/imgui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jorge/Escritorio/code/exts/imgui/imgui/backends/imgui_impl_opengl3.cpp > CMakeFiles/imgui.dir/imgui/backends/imgui_impl_opengl3.cpp.i

exts/imgui/CMakeFiles/imgui.dir/imgui/backends/imgui_impl_opengl3.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/imgui.dir/imgui/backends/imgui_impl_opengl3.cpp.s"
	cd /home/jorge/Escritorio/code/build/exts/imgui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jorge/Escritorio/code/exts/imgui/imgui/backends/imgui_impl_opengl3.cpp -o CMakeFiles/imgui.dir/imgui/backends/imgui_impl_opengl3.cpp.s

# Object files for target imgui
imgui_OBJECTS = \
"CMakeFiles/imgui.dir/imgui/imgui.cpp.o" \
"CMakeFiles/imgui.dir/imgui/imgui_draw.cpp.o" \
"CMakeFiles/imgui.dir/imgui/imgui_tables.cpp.o" \
"CMakeFiles/imgui.dir/imgui/imgui_widgets.cpp.o" \
"CMakeFiles/imgui.dir/imgui/imgui_demo.cpp.o" \
"CMakeFiles/imgui.dir/imgui/backends/imgui_impl_glfw.cpp.o" \
"CMakeFiles/imgui.dir/imgui/backends/imgui_impl_opengl3.cpp.o"

# External object files for target imgui
imgui_EXTERNAL_OBJECTS =

/home/jorge/Escritorio/code/bin/libimgui.a: exts/imgui/CMakeFiles/imgui.dir/imgui/imgui.cpp.o
/home/jorge/Escritorio/code/bin/libimgui.a: exts/imgui/CMakeFiles/imgui.dir/imgui/imgui_draw.cpp.o
/home/jorge/Escritorio/code/bin/libimgui.a: exts/imgui/CMakeFiles/imgui.dir/imgui/imgui_tables.cpp.o
/home/jorge/Escritorio/code/bin/libimgui.a: exts/imgui/CMakeFiles/imgui.dir/imgui/imgui_widgets.cpp.o
/home/jorge/Escritorio/code/bin/libimgui.a: exts/imgui/CMakeFiles/imgui.dir/imgui/imgui_demo.cpp.o
/home/jorge/Escritorio/code/bin/libimgui.a: exts/imgui/CMakeFiles/imgui.dir/imgui/backends/imgui_impl_glfw.cpp.o
/home/jorge/Escritorio/code/bin/libimgui.a: exts/imgui/CMakeFiles/imgui.dir/imgui/backends/imgui_impl_opengl3.cpp.o
/home/jorge/Escritorio/code/bin/libimgui.a: exts/imgui/CMakeFiles/imgui.dir/build.make
/home/jorge/Escritorio/code/bin/libimgui.a: exts/imgui/CMakeFiles/imgui.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jorge/Escritorio/code/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Linking CXX static library /home/jorge/Escritorio/code/bin/libimgui.a"
	cd /home/jorge/Escritorio/code/build/exts/imgui && $(CMAKE_COMMAND) -P CMakeFiles/imgui.dir/cmake_clean_target.cmake
	cd /home/jorge/Escritorio/code/build/exts/imgui && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/imgui.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
exts/imgui/CMakeFiles/imgui.dir/build: /home/jorge/Escritorio/code/bin/libimgui.a
.PHONY : exts/imgui/CMakeFiles/imgui.dir/build

exts/imgui/CMakeFiles/imgui.dir/clean:
	cd /home/jorge/Escritorio/code/build/exts/imgui && $(CMAKE_COMMAND) -P CMakeFiles/imgui.dir/cmake_clean.cmake
.PHONY : exts/imgui/CMakeFiles/imgui.dir/clean

exts/imgui/CMakeFiles/imgui.dir/depend:
	cd /home/jorge/Escritorio/code/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jorge/Escritorio/code /home/jorge/Escritorio/code/exts/imgui /home/jorge/Escritorio/code/build /home/jorge/Escritorio/code/build/exts/imgui /home/jorge/Escritorio/code/build/exts/imgui/CMakeFiles/imgui.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : exts/imgui/CMakeFiles/imgui.dir/depend
