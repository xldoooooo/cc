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
CMAKE_SOURCE_DIR = /home/hyy/xld/singleQuad/c

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hyy/xld/singleQuad/c/build

# Include any dependencies generated for this target.
include CMakeFiles/NMPC_app.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/NMPC_app.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/NMPC_app.dir/flags.make

CMakeFiles/NMPC_app.dir/src/NMPC_app.cpp.o: CMakeFiles/NMPC_app.dir/flags.make
CMakeFiles/NMPC_app.dir/src/NMPC_app.cpp.o: ../src/NMPC_app.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hyy/xld/singleQuad/c/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/NMPC_app.dir/src/NMPC_app.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/NMPC_app.dir/src/NMPC_app.cpp.o -c /home/hyy/xld/singleQuad/c/src/NMPC_app.cpp

CMakeFiles/NMPC_app.dir/src/NMPC_app.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/NMPC_app.dir/src/NMPC_app.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hyy/xld/singleQuad/c/src/NMPC_app.cpp > CMakeFiles/NMPC_app.dir/src/NMPC_app.cpp.i

CMakeFiles/NMPC_app.dir/src/NMPC_app.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/NMPC_app.dir/src/NMPC_app.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hyy/xld/singleQuad/c/src/NMPC_app.cpp -o CMakeFiles/NMPC_app.dir/src/NMPC_app.cpp.s

# Object files for target NMPC_app
NMPC_app_OBJECTS = \
"CMakeFiles/NMPC_app.dir/src/NMPC_app.cpp.o"

# External object files for target NMPC_app
NMPC_app_EXTERNAL_OBJECTS =

NMPC_app: CMakeFiles/NMPC_app.dir/src/NMPC_app.cpp.o
NMPC_app: CMakeFiles/NMPC_app.dir/build.make
NMPC_app: libocp_shared_lib.so
NMPC_app: libsim_shared_lib.so
NMPC_app: CMakeFiles/NMPC_app.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hyy/xld/singleQuad/c/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable NMPC_app"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/NMPC_app.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/NMPC_app.dir/build: NMPC_app

.PHONY : CMakeFiles/NMPC_app.dir/build

CMakeFiles/NMPC_app.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/NMPC_app.dir/cmake_clean.cmake
.PHONY : CMakeFiles/NMPC_app.dir/clean

CMakeFiles/NMPC_app.dir/depend:
	cd /home/hyy/xld/singleQuad/c/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hyy/xld/singleQuad/c /home/hyy/xld/singleQuad/c /home/hyy/xld/singleQuad/c/build /home/hyy/xld/singleQuad/c/build /home/hyy/xld/singleQuad/c/build/CMakeFiles/NMPC_app.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/NMPC_app.dir/depend

