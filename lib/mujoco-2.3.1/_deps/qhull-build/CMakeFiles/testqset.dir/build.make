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
include _deps/qhull-build/CMakeFiles/testqset.dir/depend.make

# Include the progress variables for this target.
include _deps/qhull-build/CMakeFiles/testqset.dir/progress.make

# Include the compile flags for this target's objects.
include _deps/qhull-build/CMakeFiles/testqset.dir/flags.make

_deps/qhull-build/CMakeFiles/testqset.dir/src/libqhull/qset.c.o: _deps/qhull-build/CMakeFiles/testqset.dir/flags.make
_deps/qhull-build/CMakeFiles/testqset.dir/src/libqhull/qset.c.o: _deps/qhull-src/src/libqhull/qset.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object _deps/qhull-build/CMakeFiles/testqset.dir/src/libqhull/qset.c.o"
	cd /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/_deps/qhull-build && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/testqset.dir/src/libqhull/qset.c.o   -c /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/_deps/qhull-src/src/libqhull/qset.c

_deps/qhull-build/CMakeFiles/testqset.dir/src/libqhull/qset.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/testqset.dir/src/libqhull/qset.c.i"
	cd /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/_deps/qhull-build && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/_deps/qhull-src/src/libqhull/qset.c > CMakeFiles/testqset.dir/src/libqhull/qset.c.i

_deps/qhull-build/CMakeFiles/testqset.dir/src/libqhull/qset.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/testqset.dir/src/libqhull/qset.c.s"
	cd /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/_deps/qhull-build && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/_deps/qhull-src/src/libqhull/qset.c -o CMakeFiles/testqset.dir/src/libqhull/qset.c.s

_deps/qhull-build/CMakeFiles/testqset.dir/src/libqhull/mem.c.o: _deps/qhull-build/CMakeFiles/testqset.dir/flags.make
_deps/qhull-build/CMakeFiles/testqset.dir/src/libqhull/mem.c.o: _deps/qhull-src/src/libqhull/mem.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object _deps/qhull-build/CMakeFiles/testqset.dir/src/libqhull/mem.c.o"
	cd /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/_deps/qhull-build && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/testqset.dir/src/libqhull/mem.c.o   -c /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/_deps/qhull-src/src/libqhull/mem.c

_deps/qhull-build/CMakeFiles/testqset.dir/src/libqhull/mem.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/testqset.dir/src/libqhull/mem.c.i"
	cd /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/_deps/qhull-build && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/_deps/qhull-src/src/libqhull/mem.c > CMakeFiles/testqset.dir/src/libqhull/mem.c.i

_deps/qhull-build/CMakeFiles/testqset.dir/src/libqhull/mem.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/testqset.dir/src/libqhull/mem.c.s"
	cd /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/_deps/qhull-build && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/_deps/qhull-src/src/libqhull/mem.c -o CMakeFiles/testqset.dir/src/libqhull/mem.c.s

_deps/qhull-build/CMakeFiles/testqset.dir/src/libqhull/usermem.c.o: _deps/qhull-build/CMakeFiles/testqset.dir/flags.make
_deps/qhull-build/CMakeFiles/testqset.dir/src/libqhull/usermem.c.o: _deps/qhull-src/src/libqhull/usermem.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object _deps/qhull-build/CMakeFiles/testqset.dir/src/libqhull/usermem.c.o"
	cd /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/_deps/qhull-build && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/testqset.dir/src/libqhull/usermem.c.o   -c /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/_deps/qhull-src/src/libqhull/usermem.c

_deps/qhull-build/CMakeFiles/testqset.dir/src/libqhull/usermem.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/testqset.dir/src/libqhull/usermem.c.i"
	cd /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/_deps/qhull-build && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/_deps/qhull-src/src/libqhull/usermem.c > CMakeFiles/testqset.dir/src/libqhull/usermem.c.i

_deps/qhull-build/CMakeFiles/testqset.dir/src/libqhull/usermem.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/testqset.dir/src/libqhull/usermem.c.s"
	cd /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/_deps/qhull-build && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/_deps/qhull-src/src/libqhull/usermem.c -o CMakeFiles/testqset.dir/src/libqhull/usermem.c.s

_deps/qhull-build/CMakeFiles/testqset.dir/src/testqset/testqset.c.o: _deps/qhull-build/CMakeFiles/testqset.dir/flags.make
_deps/qhull-build/CMakeFiles/testqset.dir/src/testqset/testqset.c.o: _deps/qhull-src/src/testqset/testqset.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object _deps/qhull-build/CMakeFiles/testqset.dir/src/testqset/testqset.c.o"
	cd /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/_deps/qhull-build && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/testqset.dir/src/testqset/testqset.c.o   -c /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/_deps/qhull-src/src/testqset/testqset.c

_deps/qhull-build/CMakeFiles/testqset.dir/src/testqset/testqset.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/testqset.dir/src/testqset/testqset.c.i"
	cd /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/_deps/qhull-build && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/_deps/qhull-src/src/testqset/testqset.c > CMakeFiles/testqset.dir/src/testqset/testqset.c.i

_deps/qhull-build/CMakeFiles/testqset.dir/src/testqset/testqset.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/testqset.dir/src/testqset/testqset.c.s"
	cd /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/_deps/qhull-build && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/_deps/qhull-src/src/testqset/testqset.c -o CMakeFiles/testqset.dir/src/testqset/testqset.c.s

# Object files for target testqset
testqset_OBJECTS = \
"CMakeFiles/testqset.dir/src/libqhull/qset.c.o" \
"CMakeFiles/testqset.dir/src/libqhull/mem.c.o" \
"CMakeFiles/testqset.dir/src/libqhull/usermem.c.o" \
"CMakeFiles/testqset.dir/src/testqset/testqset.c.o"

# External object files for target testqset
testqset_EXTERNAL_OBJECTS =

bin/testqset: _deps/qhull-build/CMakeFiles/testqset.dir/src/libqhull/qset.c.o
bin/testqset: _deps/qhull-build/CMakeFiles/testqset.dir/src/libqhull/mem.c.o
bin/testqset: _deps/qhull-build/CMakeFiles/testqset.dir/src/libqhull/usermem.c.o
bin/testqset: _deps/qhull-build/CMakeFiles/testqset.dir/src/testqset/testqset.c.o
bin/testqset: _deps/qhull-build/CMakeFiles/testqset.dir/build.make
bin/testqset: _deps/qhull-build/CMakeFiles/testqset.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking C executable ../../bin/testqset"
	cd /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/_deps/qhull-build && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/testqset.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
_deps/qhull-build/CMakeFiles/testqset.dir/build: bin/testqset

.PHONY : _deps/qhull-build/CMakeFiles/testqset.dir/build

_deps/qhull-build/CMakeFiles/testqset.dir/clean:
	cd /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/_deps/qhull-build && $(CMAKE_COMMAND) -P CMakeFiles/testqset.dir/cmake_clean.cmake
.PHONY : _deps/qhull-build/CMakeFiles/testqset.dir/clean

_deps/qhull-build/CMakeFiles/testqset.dir/depend:
	cd /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1 /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/_deps/qhull-src /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1 /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/_deps/qhull-build /home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/_deps/qhull-build/CMakeFiles/testqset.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : _deps/qhull-build/CMakeFiles/testqset.dir/depend

