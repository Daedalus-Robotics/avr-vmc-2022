# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/avr/vmc/avr-vmc-2022/vmc/apriltag/c

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/avr/vmc/avr-vmc-2022/vmc/apriltag/c

# Include any dependencies generated for this target.
include CMakeFiles/avrapriltags.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/avrapriltags.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/avrapriltags.dir/flags.make

CMakeFiles/avrapriltags.dir/src/avrapriltags.cpp.o: CMakeFiles/avrapriltags.dir/flags.make
CMakeFiles/avrapriltags.dir/src/avrapriltags.cpp.o: src/avrapriltags.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/avr/vmc/avr-vmc-2022/vmc/apriltag/c/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/avrapriltags.dir/src/avrapriltags.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/avrapriltags.dir/src/avrapriltags.cpp.o -c /home/avr/vmc/avr-vmc-2022/vmc/apriltag/c/src/avrapriltags.cpp

CMakeFiles/avrapriltags.dir/src/avrapriltags.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/avrapriltags.dir/src/avrapriltags.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/avr/vmc/avr-vmc-2022/vmc/apriltag/c/src/avrapriltags.cpp > CMakeFiles/avrapriltags.dir/src/avrapriltags.cpp.i

CMakeFiles/avrapriltags.dir/src/avrapriltags.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/avrapriltags.dir/src/avrapriltags.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/avr/vmc/avr-vmc-2022/vmc/apriltag/c/src/avrapriltags.cpp -o CMakeFiles/avrapriltags.dir/src/avrapriltags.cpp.s

CMakeFiles/avrapriltags.dir/src/avrapriltags.cpp.o.requires:

.PHONY : CMakeFiles/avrapriltags.dir/src/avrapriltags.cpp.o.requires

CMakeFiles/avrapriltags.dir/src/avrapriltags.cpp.o.provides: CMakeFiles/avrapriltags.dir/src/avrapriltags.cpp.o.requires
	$(MAKE) -f CMakeFiles/avrapriltags.dir/build.make CMakeFiles/avrapriltags.dir/src/avrapriltags.cpp.o.provides.build
.PHONY : CMakeFiles/avrapriltags.dir/src/avrapriltags.cpp.o.provides

CMakeFiles/avrapriltags.dir/src/avrapriltags.cpp.o.provides.build: CMakeFiles/avrapriltags.dir/src/avrapriltags.cpp.o


CMakeFiles/avrapriltags.dir/src/apriltags.cpp.o: CMakeFiles/avrapriltags.dir/flags.make
CMakeFiles/avrapriltags.dir/src/apriltags.cpp.o: src/apriltags.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/avr/vmc/avr-vmc-2022/vmc/apriltag/c/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/avrapriltags.dir/src/apriltags.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/avrapriltags.dir/src/apriltags.cpp.o -c /home/avr/vmc/avr-vmc-2022/vmc/apriltag/c/src/apriltags.cpp

CMakeFiles/avrapriltags.dir/src/apriltags.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/avrapriltags.dir/src/apriltags.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/avr/vmc/avr-vmc-2022/vmc/apriltag/c/src/apriltags.cpp > CMakeFiles/avrapriltags.dir/src/apriltags.cpp.i

CMakeFiles/avrapriltags.dir/src/apriltags.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/avrapriltags.dir/src/apriltags.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/avr/vmc/avr-vmc-2022/vmc/apriltag/c/src/apriltags.cpp -o CMakeFiles/avrapriltags.dir/src/apriltags.cpp.s

CMakeFiles/avrapriltags.dir/src/apriltags.cpp.o.requires:

.PHONY : CMakeFiles/avrapriltags.dir/src/apriltags.cpp.o.requires

CMakeFiles/avrapriltags.dir/src/apriltags.cpp.o.provides: CMakeFiles/avrapriltags.dir/src/apriltags.cpp.o.requires
	$(MAKE) -f CMakeFiles/avrapriltags.dir/build.make CMakeFiles/avrapriltags.dir/src/apriltags.cpp.o.provides.build
.PHONY : CMakeFiles/avrapriltags.dir/src/apriltags.cpp.o.provides

CMakeFiles/avrapriltags.dir/src/apriltags.cpp.o.provides.build: CMakeFiles/avrapriltags.dir/src/apriltags.cpp.o


CMakeFiles/avrapriltags.dir/src/undistort.cpp.o: CMakeFiles/avrapriltags.dir/flags.make
CMakeFiles/avrapriltags.dir/src/undistort.cpp.o: src/undistort.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/avr/vmc/avr-vmc-2022/vmc/apriltag/c/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/avrapriltags.dir/src/undistort.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/avrapriltags.dir/src/undistort.cpp.o -c /home/avr/vmc/avr-vmc-2022/vmc/apriltag/c/src/undistort.cpp

CMakeFiles/avrapriltags.dir/src/undistort.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/avrapriltags.dir/src/undistort.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/avr/vmc/avr-vmc-2022/vmc/apriltag/c/src/undistort.cpp > CMakeFiles/avrapriltags.dir/src/undistort.cpp.i

CMakeFiles/avrapriltags.dir/src/undistort.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/avrapriltags.dir/src/undistort.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/avr/vmc/avr-vmc-2022/vmc/apriltag/c/src/undistort.cpp -o CMakeFiles/avrapriltags.dir/src/undistort.cpp.s

CMakeFiles/avrapriltags.dir/src/undistort.cpp.o.requires:

.PHONY : CMakeFiles/avrapriltags.dir/src/undistort.cpp.o.requires

CMakeFiles/avrapriltags.dir/src/undistort.cpp.o.provides: CMakeFiles/avrapriltags.dir/src/undistort.cpp.o.requires
	$(MAKE) -f CMakeFiles/avrapriltags.dir/build.make CMakeFiles/avrapriltags.dir/src/undistort.cpp.o.provides.build
.PHONY : CMakeFiles/avrapriltags.dir/src/undistort.cpp.o.provides

CMakeFiles/avrapriltags.dir/src/undistort.cpp.o.provides.build: CMakeFiles/avrapriltags.dir/src/undistort.cpp.o


CMakeFiles/avrapriltags.dir/src/cam_properties.cpp.o: CMakeFiles/avrapriltags.dir/flags.make
CMakeFiles/avrapriltags.dir/src/cam_properties.cpp.o: src/cam_properties.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/avr/vmc/avr-vmc-2022/vmc/apriltag/c/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/avrapriltags.dir/src/cam_properties.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/avrapriltags.dir/src/cam_properties.cpp.o -c /home/avr/vmc/avr-vmc-2022/vmc/apriltag/c/src/cam_properties.cpp

CMakeFiles/avrapriltags.dir/src/cam_properties.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/avrapriltags.dir/src/cam_properties.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/avr/vmc/avr-vmc-2022/vmc/apriltag/c/src/cam_properties.cpp > CMakeFiles/avrapriltags.dir/src/cam_properties.cpp.i

CMakeFiles/avrapriltags.dir/src/cam_properties.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/avrapriltags.dir/src/cam_properties.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/avr/vmc/avr-vmc-2022/vmc/apriltag/c/src/cam_properties.cpp -o CMakeFiles/avrapriltags.dir/src/cam_properties.cpp.s

CMakeFiles/avrapriltags.dir/src/cam_properties.cpp.o.requires:

.PHONY : CMakeFiles/avrapriltags.dir/src/cam_properties.cpp.o.requires

CMakeFiles/avrapriltags.dir/src/cam_properties.cpp.o.provides: CMakeFiles/avrapriltags.dir/src/cam_properties.cpp.o.requires
	$(MAKE) -f CMakeFiles/avrapriltags.dir/build.make CMakeFiles/avrapriltags.dir/src/cam_properties.cpp.o.provides.build
.PHONY : CMakeFiles/avrapriltags.dir/src/cam_properties.cpp.o.provides

CMakeFiles/avrapriltags.dir/src/cam_properties.cpp.o.provides.build: CMakeFiles/avrapriltags.dir/src/cam_properties.cpp.o


# Object files for target avrapriltags
avrapriltags_OBJECTS = \
"CMakeFiles/avrapriltags.dir/src/avrapriltags.cpp.o" \
"CMakeFiles/avrapriltags.dir/src/apriltags.cpp.o" \
"CMakeFiles/avrapriltags.dir/src/undistort.cpp.o" \
"CMakeFiles/avrapriltags.dir/src/cam_properties.cpp.o"

# External object files for target avrapriltags
avrapriltags_EXTERNAL_OBJECTS =

avrapriltags: CMakeFiles/avrapriltags.dir/src/avrapriltags.cpp.o
avrapriltags: CMakeFiles/avrapriltags.dir/src/apriltags.cpp.o
avrapriltags: CMakeFiles/avrapriltags.dir/src/undistort.cpp.o
avrapriltags: CMakeFiles/avrapriltags.dir/src/cam_properties.cpp.o
avrapriltags: CMakeFiles/avrapriltags.dir/build.make
avrapriltags: CMakeFiles/avrapriltags.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/avr/vmc/avr-vmc-2022/vmc/apriltag/c/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX executable avrapriltags"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/avrapriltags.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/avrapriltags.dir/build: avrapriltags

.PHONY : CMakeFiles/avrapriltags.dir/build

CMakeFiles/avrapriltags.dir/requires: CMakeFiles/avrapriltags.dir/src/avrapriltags.cpp.o.requires
CMakeFiles/avrapriltags.dir/requires: CMakeFiles/avrapriltags.dir/src/apriltags.cpp.o.requires
CMakeFiles/avrapriltags.dir/requires: CMakeFiles/avrapriltags.dir/src/undistort.cpp.o.requires
CMakeFiles/avrapriltags.dir/requires: CMakeFiles/avrapriltags.dir/src/cam_properties.cpp.o.requires

.PHONY : CMakeFiles/avrapriltags.dir/requires

CMakeFiles/avrapriltags.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/avrapriltags.dir/cmake_clean.cmake
.PHONY : CMakeFiles/avrapriltags.dir/clean

CMakeFiles/avrapriltags.dir/depend:
	cd /home/avr/vmc/avr-vmc-2022/vmc/apriltag/c && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/avr/vmc/avr-vmc-2022/vmc/apriltag/c /home/avr/vmc/avr-vmc-2022/vmc/apriltag/c /home/avr/vmc/avr-vmc-2022/vmc/apriltag/c /home/avr/vmc/avr-vmc-2022/vmc/apriltag/c /home/avr/vmc/avr-vmc-2022/vmc/apriltag/c/CMakeFiles/avrapriltags.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/avrapriltags.dir/depend
