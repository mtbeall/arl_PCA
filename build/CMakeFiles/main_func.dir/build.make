# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/daman/ros_workspace/arl_PCA

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/daman/ros_workspace/arl_PCA/build

# Include any dependencies generated for this target.
include CMakeFiles/main_func.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/main_func.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/main_func.dir/flags.make

CMakeFiles/main_func.dir/src/projection.o: CMakeFiles/main_func.dir/flags.make
CMakeFiles/main_func.dir/src/projection.o: ../src/projection.cpp
CMakeFiles/main_func.dir/src/projection.o: ../manifest.xml
CMakeFiles/main_func.dir/src/projection.o: /opt/ros/fuerte/share/roslang/manifest.xml
CMakeFiles/main_func.dir/src/projection.o: /opt/ros/fuerte/share/roscpp/manifest.xml
CMakeFiles/main_func.dir/src/projection.o: /opt/ros/fuerte/share/std_msgs/manifest.xml
CMakeFiles/main_func.dir/src/projection.o: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
CMakeFiles/main_func.dir/src/projection.o: /opt/ros/fuerte/share/sensor_msgs/manifest.xml
CMakeFiles/main_func.dir/src/projection.o: /opt/ros/fuerte/stacks/bullet/manifest.xml
CMakeFiles/main_func.dir/src/projection.o: /opt/ros/fuerte/share/rosconsole/manifest.xml
CMakeFiles/main_func.dir/src/projection.o: /opt/ros/fuerte/stacks/geometry/angles/manifest.xml
CMakeFiles/main_func.dir/src/projection.o: /opt/ros/fuerte/share/rospy/manifest.xml
CMakeFiles/main_func.dir/src/projection.o: /opt/ros/fuerte/share/rostest/manifest.xml
CMakeFiles/main_func.dir/src/projection.o: /opt/ros/fuerte/share/roswtf/manifest.xml
CMakeFiles/main_func.dir/src/projection.o: /opt/ros/fuerte/share/message_filters/manifest.xml
CMakeFiles/main_func.dir/src/projection.o: /opt/ros/fuerte/stacks/geometry/tf/manifest.xml
CMakeFiles/main_func.dir/src/projection.o: /opt/ros/fuerte/share/ros/core/rosbuild/manifest.xml
CMakeFiles/main_func.dir/src/projection.o: /opt/ros/fuerte/share/roslib/manifest.xml
CMakeFiles/main_func.dir/src/projection.o: /opt/ros/fuerte/stacks/pluginlib/manifest.xml
CMakeFiles/main_func.dir/src/projection.o: /opt/ros/fuerte/stacks/image_common/image_transport/manifest.xml
CMakeFiles/main_func.dir/src/projection.o: /opt/ros/fuerte/share/std_srvs/manifest.xml
CMakeFiles/main_func.dir/src/projection.o: /opt/ros/fuerte/stacks/common_rosdeps/manifest.xml
CMakeFiles/main_func.dir/src/projection.o: /opt/ros/fuerte/stacks/image_common/camera_calibration_parsers/manifest.xml
CMakeFiles/main_func.dir/src/projection.o: /opt/ros/fuerte/stacks/image_common/camera_info_manager/manifest.xml
CMakeFiles/main_func.dir/src/projection.o: /home/daman/ros_workspace/ardrone_autonomy/manifest.xml
CMakeFiles/main_func.dir/src/projection.o: /opt/ros/fuerte/stacks/geometry/tf/msg_gen/generated
CMakeFiles/main_func.dir/src/projection.o: /opt/ros/fuerte/stacks/geometry/tf/srv_gen/generated
CMakeFiles/main_func.dir/src/projection.o: /home/daman/ros_workspace/ardrone_autonomy/msg_gen/generated
CMakeFiles/main_func.dir/src/projection.o: /home/daman/ros_workspace/ardrone_autonomy/srv_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/daman/ros_workspace/arl_PCA/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/main_func.dir/src/projection.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/main_func.dir/src/projection.o -c /home/daman/ros_workspace/arl_PCA/src/projection.cpp

CMakeFiles/main_func.dir/src/projection.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main_func.dir/src/projection.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/daman/ros_workspace/arl_PCA/src/projection.cpp > CMakeFiles/main_func.dir/src/projection.i

CMakeFiles/main_func.dir/src/projection.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main_func.dir/src/projection.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/daman/ros_workspace/arl_PCA/src/projection.cpp -o CMakeFiles/main_func.dir/src/projection.s

CMakeFiles/main_func.dir/src/projection.o.requires:
.PHONY : CMakeFiles/main_func.dir/src/projection.o.requires

CMakeFiles/main_func.dir/src/projection.o.provides: CMakeFiles/main_func.dir/src/projection.o.requires
	$(MAKE) -f CMakeFiles/main_func.dir/build.make CMakeFiles/main_func.dir/src/projection.o.provides.build
.PHONY : CMakeFiles/main_func.dir/src/projection.o.provides

CMakeFiles/main_func.dir/src/projection.o.provides.build: CMakeFiles/main_func.dir/src/projection.o

# Object files for target main_func
main_func_OBJECTS = \
"CMakeFiles/main_func.dir/src/projection.o"

# External object files for target main_func
main_func_EXTERNAL_OBJECTS =

../bin/main_func: CMakeFiles/main_func.dir/src/projection.o
../bin/main_func: CMakeFiles/main_func.dir/build.make
../bin/main_func: CMakeFiles/main_func.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../bin/main_func"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/main_func.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/main_func.dir/build: ../bin/main_func
.PHONY : CMakeFiles/main_func.dir/build

CMakeFiles/main_func.dir/requires: CMakeFiles/main_func.dir/src/projection.o.requires
.PHONY : CMakeFiles/main_func.dir/requires

CMakeFiles/main_func.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/main_func.dir/cmake_clean.cmake
.PHONY : CMakeFiles/main_func.dir/clean

CMakeFiles/main_func.dir/depend:
	cd /home/daman/ros_workspace/arl_PCA/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/daman/ros_workspace/arl_PCA /home/daman/ros_workspace/arl_PCA /home/daman/ros_workspace/arl_PCA/build /home/daman/ros_workspace/arl_PCA/build /home/daman/ros_workspace/arl_PCA/build/CMakeFiles/main_func.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/main_func.dir/depend

