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

# Escaping for special characters.
EQUALS = =

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/edno/projetos/robot/cmdMessenger-cpp

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/edno/projetos/robot/cmdMessenger-cpp/build

# Include any dependencies generated for this target.
include CMakeFiles/led.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/led.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/led.dir/flags.make

CMakeFiles/led.dir/src/examples/led.cpp.o: CMakeFiles/led.dir/flags.make
CMakeFiles/led.dir/src/examples/led.cpp.o: ../src/examples/led.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/edno/projetos/robot/cmdMessenger-cpp/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/led.dir/src/examples/led.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/led.dir/src/examples/led.cpp.o -c /home/edno/projetos/robot/cmdMessenger-cpp/src/examples/led.cpp

CMakeFiles/led.dir/src/examples/led.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/led.dir/src/examples/led.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/edno/projetos/robot/cmdMessenger-cpp/src/examples/led.cpp > CMakeFiles/led.dir/src/examples/led.cpp.i

CMakeFiles/led.dir/src/examples/led.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/led.dir/src/examples/led.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/edno/projetos/robot/cmdMessenger-cpp/src/examples/led.cpp -o CMakeFiles/led.dir/src/examples/led.cpp.s

CMakeFiles/led.dir/src/examples/led.cpp.o.requires:
.PHONY : CMakeFiles/led.dir/src/examples/led.cpp.o.requires

CMakeFiles/led.dir/src/examples/led.cpp.o.provides: CMakeFiles/led.dir/src/examples/led.cpp.o.requires
	$(MAKE) -f CMakeFiles/led.dir/build.make CMakeFiles/led.dir/src/examples/led.cpp.o.provides.build
.PHONY : CMakeFiles/led.dir/src/examples/led.cpp.o.provides

CMakeFiles/led.dir/src/examples/led.cpp.o.provides.build: CMakeFiles/led.dir/src/examples/led.cpp.o

# Object files for target led
led_OBJECTS = \
"CMakeFiles/led.dir/src/examples/led.cpp.o"

# External object files for target led
led_EXTERNAL_OBJECTS =

led: CMakeFiles/led.dir/src/examples/led.cpp.o
led: CMakeFiles/led.dir/build.make
led: libmessenger.so
led: CMakeFiles/led.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable led"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/led.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/led.dir/build: led
.PHONY : CMakeFiles/led.dir/build

CMakeFiles/led.dir/requires: CMakeFiles/led.dir/src/examples/led.cpp.o.requires
.PHONY : CMakeFiles/led.dir/requires

CMakeFiles/led.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/led.dir/cmake_clean.cmake
.PHONY : CMakeFiles/led.dir/clean

CMakeFiles/led.dir/depend:
	cd /home/edno/projetos/robot/cmdMessenger-cpp/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/edno/projetos/robot/cmdMessenger-cpp /home/edno/projetos/robot/cmdMessenger-cpp /home/edno/projetos/robot/cmdMessenger-cpp/build /home/edno/projetos/robot/cmdMessenger-cpp/build /home/edno/projetos/robot/cmdMessenger-cpp/build/CMakeFiles/led.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/led.dir/depend

