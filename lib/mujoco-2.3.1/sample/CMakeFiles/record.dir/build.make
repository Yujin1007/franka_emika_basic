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
CMAKE_SOURCE_DIR = /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1

# Include any dependencies generated for this target.
include sample/CMakeFiles/record.dir/depend.make

# Include the progress variables for this target.
include sample/CMakeFiles/record.dir/progress.make

# Include the compile flags for this target's objects.
include sample/CMakeFiles/record.dir/flags.make

sample/CMakeFiles/record.dir/record.cc.o: sample/CMakeFiles/record.dir/flags.make
sample/CMakeFiles/record.dir/record.cc.o: sample/record.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object sample/CMakeFiles/record.dir/record.cc.o"
	cd /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/sample && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/record.dir/record.cc.o -c /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/sample/record.cc

sample/CMakeFiles/record.dir/record.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/record.dir/record.cc.i"
	cd /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/sample && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/sample/record.cc > CMakeFiles/record.dir/record.cc.i

sample/CMakeFiles/record.dir/record.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/record.dir/record.cc.s"
	cd /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/sample && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/sample/record.cc -o CMakeFiles/record.dir/record.cc.s

# Object files for target record
record_OBJECTS = \
"CMakeFiles/record.dir/record.cc.o"

# External object files for target record
record_EXTERNAL_OBJECTS =

bin/record: sample/CMakeFiles/record.dir/record.cc.o
bin/record: sample/CMakeFiles/record.dir/build.make
bin/record: lib/libmujoco.so.2.3.1
bin/record: lib/libglfw3.a
bin/record: /usr/lib/x86_64-linux-gnu/librt.so
bin/record: /usr/lib/x86_64-linux-gnu/libm.so
bin/record: /usr/lib/x86_64-linux-gnu/libX11.so
bin/record: sample/CMakeFiles/record.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../bin/record"
	cd /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/sample && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/record.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
sample/CMakeFiles/record.dir/build: bin/record

.PHONY : sample/CMakeFiles/record.dir/build

sample/CMakeFiles/record.dir/clean:
	cd /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/sample && $(CMAKE_COMMAND) -P CMakeFiles/record.dir/cmake_clean.cmake
.PHONY : sample/CMakeFiles/record.dir/clean

sample/CMakeFiles/record.dir/depend:
	cd /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1 /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/sample /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1 /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/sample /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/sample/CMakeFiles/record.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : sample/CMakeFiles/record.dir/depend

