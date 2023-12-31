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
include _deps/qhull-build/CMakeFiles/qhullp.dir/depend.make

# Include the progress variables for this target.
include _deps/qhull-build/CMakeFiles/qhullp.dir/progress.make

# Include the compile flags for this target's objects.
include _deps/qhull-build/CMakeFiles/qhullp.dir/flags.make

_deps/qhull-build/CMakeFiles/qhullp.dir/src/qhull/unix.c.o: _deps/qhull-build/CMakeFiles/qhullp.dir/flags.make
_deps/qhull-build/CMakeFiles/qhullp.dir/src/qhull/unix.c.o: _deps/qhull-src/src/qhull/unix.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object _deps/qhull-build/CMakeFiles/qhullp.dir/src/qhull/unix.c.o"
	cd /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/_deps/qhull-build && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/qhullp.dir/src/qhull/unix.c.o   -c /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/_deps/qhull-src/src/qhull/unix.c

_deps/qhull-build/CMakeFiles/qhullp.dir/src/qhull/unix.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/qhullp.dir/src/qhull/unix.c.i"
	cd /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/_deps/qhull-build && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/_deps/qhull-src/src/qhull/unix.c > CMakeFiles/qhullp.dir/src/qhull/unix.c.i

_deps/qhull-build/CMakeFiles/qhullp.dir/src/qhull/unix.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/qhullp.dir/src/qhull/unix.c.s"
	cd /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/_deps/qhull-build && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/_deps/qhull-src/src/qhull/unix.c -o CMakeFiles/qhullp.dir/src/qhull/unix.c.s

# Object files for target qhullp
qhullp_OBJECTS = \
"CMakeFiles/qhullp.dir/src/qhull/unix.c.o"

# External object files for target qhullp
qhullp_EXTERNAL_OBJECTS =

bin/qhullp: _deps/qhull-build/CMakeFiles/qhullp.dir/src/qhull/unix.c.o
bin/qhullp: _deps/qhull-build/CMakeFiles/qhullp.dir/build.make
bin/qhullp: lib/libqhull_p.so.8.1-alpha1
bin/qhullp: _deps/qhull-build/CMakeFiles/qhullp.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C executable ../../bin/qhullp"
	cd /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/_deps/qhull-build && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/qhullp.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
_deps/qhull-build/CMakeFiles/qhullp.dir/build: bin/qhullp

.PHONY : _deps/qhull-build/CMakeFiles/qhullp.dir/build

_deps/qhull-build/CMakeFiles/qhullp.dir/clean:
	cd /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/_deps/qhull-build && $(CMAKE_COMMAND) -P CMakeFiles/qhullp.dir/cmake_clean.cmake
.PHONY : _deps/qhull-build/CMakeFiles/qhullp.dir/clean

_deps/qhull-build/CMakeFiles/qhullp.dir/depend:
	cd /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1 /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/_deps/qhull-src /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1 /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/_deps/qhull-build /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/_deps/qhull-build/CMakeFiles/qhullp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : _deps/qhull-build/CMakeFiles/qhullp.dir/depend

