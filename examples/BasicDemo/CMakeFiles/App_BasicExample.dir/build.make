# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/lolitsjef/Desktop/bullet3

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lolitsjef/Desktop/bullet3

# Include any dependencies generated for this target.
include examples/BasicDemo/CMakeFiles/App_BasicExample.dir/depend.make

# Include the progress variables for this target.
include examples/BasicDemo/CMakeFiles/App_BasicExample.dir/progress.make

# Include the compile flags for this target's objects.
include examples/BasicDemo/CMakeFiles/App_BasicExample.dir/flags.make

examples/BasicDemo/CMakeFiles/App_BasicExample.dir/BasicExample.cpp.o: examples/BasicDemo/CMakeFiles/App_BasicExample.dir/flags.make
examples/BasicDemo/CMakeFiles/App_BasicExample.dir/BasicExample.cpp.o: examples/BasicDemo/BasicExample.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lolitsjef/Desktop/bullet3/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/BasicDemo/CMakeFiles/App_BasicExample.dir/BasicExample.cpp.o"
	cd /home/lolitsjef/Desktop/bullet3/examples/BasicDemo && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/App_BasicExample.dir/BasicExample.cpp.o -c /home/lolitsjef/Desktop/bullet3/examples/BasicDemo/BasicExample.cpp

examples/BasicDemo/CMakeFiles/App_BasicExample.dir/BasicExample.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/App_BasicExample.dir/BasicExample.cpp.i"
	cd /home/lolitsjef/Desktop/bullet3/examples/BasicDemo && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lolitsjef/Desktop/bullet3/examples/BasicDemo/BasicExample.cpp > CMakeFiles/App_BasicExample.dir/BasicExample.cpp.i

examples/BasicDemo/CMakeFiles/App_BasicExample.dir/BasicExample.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/App_BasicExample.dir/BasicExample.cpp.s"
	cd /home/lolitsjef/Desktop/bullet3/examples/BasicDemo && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lolitsjef/Desktop/bullet3/examples/BasicDemo/BasicExample.cpp -o CMakeFiles/App_BasicExample.dir/BasicExample.cpp.s

examples/BasicDemo/CMakeFiles/App_BasicExample.dir/main.cpp.o: examples/BasicDemo/CMakeFiles/App_BasicExample.dir/flags.make
examples/BasicDemo/CMakeFiles/App_BasicExample.dir/main.cpp.o: examples/BasicDemo/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lolitsjef/Desktop/bullet3/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object examples/BasicDemo/CMakeFiles/App_BasicExample.dir/main.cpp.o"
	cd /home/lolitsjef/Desktop/bullet3/examples/BasicDemo && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/App_BasicExample.dir/main.cpp.o -c /home/lolitsjef/Desktop/bullet3/examples/BasicDemo/main.cpp

examples/BasicDemo/CMakeFiles/App_BasicExample.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/App_BasicExample.dir/main.cpp.i"
	cd /home/lolitsjef/Desktop/bullet3/examples/BasicDemo && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lolitsjef/Desktop/bullet3/examples/BasicDemo/main.cpp > CMakeFiles/App_BasicExample.dir/main.cpp.i

examples/BasicDemo/CMakeFiles/App_BasicExample.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/App_BasicExample.dir/main.cpp.s"
	cd /home/lolitsjef/Desktop/bullet3/examples/BasicDemo && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lolitsjef/Desktop/bullet3/examples/BasicDemo/main.cpp -o CMakeFiles/App_BasicExample.dir/main.cpp.s

# Object files for target App_BasicExample
App_BasicExample_OBJECTS = \
"CMakeFiles/App_BasicExample.dir/BasicExample.cpp.o" \
"CMakeFiles/App_BasicExample.dir/main.cpp.o"

# External object files for target App_BasicExample
App_BasicExample_EXTERNAL_OBJECTS =

examples/BasicDemo/App_BasicExample: examples/BasicDemo/CMakeFiles/App_BasicExample.dir/BasicExample.cpp.o
examples/BasicDemo/App_BasicExample: examples/BasicDemo/CMakeFiles/App_BasicExample.dir/main.cpp.o
examples/BasicDemo/App_BasicExample: examples/BasicDemo/CMakeFiles/App_BasicExample.dir/build.make
examples/BasicDemo/App_BasicExample: src/BulletDynamics/libBulletDynamics.a
examples/BasicDemo/App_BasicExample: src/BulletCollision/libBulletCollision.a
examples/BasicDemo/App_BasicExample: src/LinearMath/libLinearMath.a
examples/BasicDemo/App_BasicExample: examples/BasicDemo/CMakeFiles/App_BasicExample.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lolitsjef/Desktop/bullet3/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable App_BasicExample"
	cd /home/lolitsjef/Desktop/bullet3/examples/BasicDemo && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/App_BasicExample.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/BasicDemo/CMakeFiles/App_BasicExample.dir/build: examples/BasicDemo/App_BasicExample

.PHONY : examples/BasicDemo/CMakeFiles/App_BasicExample.dir/build

examples/BasicDemo/CMakeFiles/App_BasicExample.dir/clean:
	cd /home/lolitsjef/Desktop/bullet3/examples/BasicDemo && $(CMAKE_COMMAND) -P CMakeFiles/App_BasicExample.dir/cmake_clean.cmake
.PHONY : examples/BasicDemo/CMakeFiles/App_BasicExample.dir/clean

examples/BasicDemo/CMakeFiles/App_BasicExample.dir/depend:
	cd /home/lolitsjef/Desktop/bullet3 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lolitsjef/Desktop/bullet3 /home/lolitsjef/Desktop/bullet3/examples/BasicDemo /home/lolitsjef/Desktop/bullet3 /home/lolitsjef/Desktop/bullet3/examples/BasicDemo /home/lolitsjef/Desktop/bullet3/examples/BasicDemo/CMakeFiles/App_BasicExample.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/BasicDemo/CMakeFiles/App_BasicExample.dir/depend

