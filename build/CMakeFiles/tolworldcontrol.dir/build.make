# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.11

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
CMAKE_COMMAND = /usr/local/Cellar/cmake/3.11.1/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/3.11.1/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/karinemiras/projects/coevolution-revolve/tol-revolve/cpp

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/karinemiras/projects/coevolution-revolve/tol-revolve/build

# Include any dependencies generated for this target.
include CMakeFiles/tolworldcontrol.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/tolworldcontrol.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/tolworldcontrol.dir/flags.make

CMakeFiles/tolworldcontrol.dir/tol/plugin/WorldController.cpp.o: CMakeFiles/tolworldcontrol.dir/flags.make
CMakeFiles/tolworldcontrol.dir/tol/plugin/WorldController.cpp.o: /Users/karinemiras/projects/coevolution-revolve/tol-revolve/cpp/tol/plugin/WorldController.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/karinemiras/projects/coevolution-revolve/tol-revolve/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/tolworldcontrol.dir/tol/plugin/WorldController.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/tolworldcontrol.dir/tol/plugin/WorldController.cpp.o -c /Users/karinemiras/projects/coevolution-revolve/tol-revolve/cpp/tol/plugin/WorldController.cpp

CMakeFiles/tolworldcontrol.dir/tol/plugin/WorldController.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tolworldcontrol.dir/tol/plugin/WorldController.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/karinemiras/projects/coevolution-revolve/tol-revolve/cpp/tol/plugin/WorldController.cpp > CMakeFiles/tolworldcontrol.dir/tol/plugin/WorldController.cpp.i

CMakeFiles/tolworldcontrol.dir/tol/plugin/WorldController.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tolworldcontrol.dir/tol/plugin/WorldController.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/karinemiras/projects/coevolution-revolve/tol-revolve/cpp/tol/plugin/WorldController.cpp -o CMakeFiles/tolworldcontrol.dir/tol/plugin/WorldController.cpp.s

# Object files for target tolworldcontrol
tolworldcontrol_OBJECTS = \
"CMakeFiles/tolworldcontrol.dir/tol/plugin/WorldController.cpp.o"

# External object files for target tolworldcontrol
tolworldcontrol_EXTERNAL_OBJECTS =

libtolworldcontrol.dylib: CMakeFiles/tolworldcontrol.dir/tol/plugin/WorldController.cpp.o
libtolworldcontrol.dylib: CMakeFiles/tolworldcontrol.dir/build.make
libtolworldcontrol.dylib: /usr/local/lib/libgazebo.dylib
libtolworldcontrol.dylib: /usr/local/lib/libgazebo_client.dylib
libtolworldcontrol.dylib: /usr/local/lib/libgazebo_gui_building.dylib
libtolworldcontrol.dylib: /usr/local/lib/libgazebo_gui_viewers.dylib
libtolworldcontrol.dylib: /usr/local/lib/libgazebo_gui_model.dylib
libtolworldcontrol.dylib: /usr/local/lib/libgazebo_gui.dylib
libtolworldcontrol.dylib: /usr/local/lib/libgazebo_sensors.dylib
libtolworldcontrol.dylib: /usr/local/lib/libgazebo_rendering.dylib
libtolworldcontrol.dylib: /usr/local/lib/libgazebo_selection_buffer.dylib
libtolworldcontrol.dylib: /usr/local/lib/libgazebo_physics_bullet.dylib
libtolworldcontrol.dylib: /usr/local/lib/libgazebo_physics_simbody.dylib
libtolworldcontrol.dylib: /usr/local/lib/libgazebo_physics_ode.dylib
libtolworldcontrol.dylib: /usr/local/lib/libgazebo_physics.dylib
libtolworldcontrol.dylib: /usr/local/lib/libgazebo_ode.dylib
libtolworldcontrol.dylib: /usr/local/lib/libgazebo_transport.dylib
libtolworldcontrol.dylib: /usr/local/lib/libgazebo_msgs.dylib
libtolworldcontrol.dylib: /usr/local/lib/libgazebo_util.dylib
libtolworldcontrol.dylib: /usr/local/lib/libgazebo_common.dylib
libtolworldcontrol.dylib: /usr/local/lib/libgazebo_skyx.dylib
libtolworldcontrol.dylib: /usr/local/lib/libgazebo_gimpact.dylib
libtolworldcontrol.dylib: /usr/local/lib/libgazebo_opcode.dylib
libtolworldcontrol.dylib: /usr/local/lib/libgazebo_opende_ou.dylib
libtolworldcontrol.dylib: /usr/local/lib/libgazebo_math.dylib
libtolworldcontrol.dylib: /usr/local/lib/libgazebo_ccd.dylib
libtolworldcontrol.dylib: /usr/local/lib/libboost_system-mt.dylib
libtolworldcontrol.dylib: /usr/local/lib/libprotobuf.dylib
libtolworldcontrol.dylib: /usr/local/Cellar/sdformat3/3.7.0_5/lib/libsdformat.dylib
libtolworldcontrol.dylib: /usr/local/Cellar/ignition-math2/2.8.0/lib/libignition-math2.dylib
libtolworldcontrol.dylib: /usr/local/Cellar/ogre/1.7.4_4/lib/libOgreMain.dylib
libtolworldcontrol.dylib: /usr/local/Cellar/ogre/1.7.4_4/lib/libOgreTerrain.dylib
libtolworldcontrol.dylib: /usr/local/Cellar/ogre/1.7.4_4/lib/libOgrePaging.dylib
libtolworldcontrol.dylib: /usr/local/Cellar/ignition-math2/2.8.0/lib/libignition-math2.dylib
libtolworldcontrol.dylib: ../../revolve/build/lib/librevolve-gazebo.a
libtolworldcontrol.dylib: ../../revolve/build/lib/librevolve-spec.a
libtolworldcontrol.dylib: /usr/local/lib/libgazebo.dylib
libtolworldcontrol.dylib: /usr/local/lib/libgazebo_client.dylib
libtolworldcontrol.dylib: /usr/local/lib/libgazebo_gui_building.dylib
libtolworldcontrol.dylib: /usr/local/lib/libgazebo_gui_viewers.dylib
libtolworldcontrol.dylib: /usr/local/lib/libgazebo_gui_model.dylib
libtolworldcontrol.dylib: /usr/local/lib/libgazebo_gui.dylib
libtolworldcontrol.dylib: /usr/local/lib/libgazebo_sensors.dylib
libtolworldcontrol.dylib: /usr/local/lib/libgazebo_rendering.dylib
libtolworldcontrol.dylib: /usr/local/lib/libgazebo_selection_buffer.dylib
libtolworldcontrol.dylib: /usr/local/lib/libgazebo_physics_bullet.dylib
libtolworldcontrol.dylib: /usr/local/lib/libgazebo_physics_simbody.dylib
libtolworldcontrol.dylib: /usr/local/lib/libgazebo_physics_ode.dylib
libtolworldcontrol.dylib: /usr/local/lib/libgazebo_physics.dylib
libtolworldcontrol.dylib: /usr/local/lib/libgazebo_ode.dylib
libtolworldcontrol.dylib: /usr/local/lib/libgazebo_transport.dylib
libtolworldcontrol.dylib: /usr/local/lib/libgazebo_msgs.dylib
libtolworldcontrol.dylib: /usr/local/lib/libgazebo_util.dylib
libtolworldcontrol.dylib: /usr/local/lib/libgazebo_common.dylib
libtolworldcontrol.dylib: /usr/local/lib/libgazebo_skyx.dylib
libtolworldcontrol.dylib: /usr/local/lib/libgazebo_gimpact.dylib
libtolworldcontrol.dylib: /usr/local/lib/libgazebo_opcode.dylib
libtolworldcontrol.dylib: /usr/local/lib/libgazebo_opende_ou.dylib
libtolworldcontrol.dylib: /usr/local/lib/libgazebo_math.dylib
libtolworldcontrol.dylib: /usr/local/lib/libgazebo_ccd.dylib
libtolworldcontrol.dylib: /usr/local/lib/libboost_system-mt.dylib
libtolworldcontrol.dylib: /usr/local/lib/libprotobuf.dylib
libtolworldcontrol.dylib: /usr/local/Cellar/sdformat3/3.7.0_5/lib/libsdformat.dylib
libtolworldcontrol.dylib: /usr/local/Cellar/ignition-math2/2.8.0/lib/libignition-math2.dylib
libtolworldcontrol.dylib: /usr/local/Cellar/ogre/1.7.4_4/lib/libOgreMain.dylib
libtolworldcontrol.dylib: /usr/local/Cellar/ogre/1.7.4_4/lib/libOgreTerrain.dylib
libtolworldcontrol.dylib: /usr/local/Cellar/ogre/1.7.4_4/lib/libOgrePaging.dylib
libtolworldcontrol.dylib: /usr/local/Cellar/ignition-math2/2.8.0/lib/libignition-math2.dylib
libtolworldcontrol.dylib: ../../revolve/build/lib/librevolve-gazebo.a
libtolworldcontrol.dylib: ../../revolve/build/lib/librevolve-spec.a
libtolworldcontrol.dylib: CMakeFiles/tolworldcontrol.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/karinemiras/projects/coevolution-revolve/tol-revolve/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libtolworldcontrol.dylib"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/tolworldcontrol.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/tolworldcontrol.dir/build: libtolworldcontrol.dylib

.PHONY : CMakeFiles/tolworldcontrol.dir/build

CMakeFiles/tolworldcontrol.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/tolworldcontrol.dir/cmake_clean.cmake
.PHONY : CMakeFiles/tolworldcontrol.dir/clean

CMakeFiles/tolworldcontrol.dir/depend:
	cd /Users/karinemiras/projects/coevolution-revolve/tol-revolve/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/karinemiras/projects/coevolution-revolve/tol-revolve/cpp /Users/karinemiras/projects/coevolution-revolve/tol-revolve/cpp /Users/karinemiras/projects/coevolution-revolve/tol-revolve/build /Users/karinemiras/projects/coevolution-revolve/tol-revolve/build /Users/karinemiras/projects/coevolution-revolve/tol-revolve/build/CMakeFiles/tolworldcontrol.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/tolworldcontrol.dir/depend

