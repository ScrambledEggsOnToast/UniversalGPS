# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.3

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
CMAKE_SOURCE_DIR = /home/joshkirklin/ugps-cpp

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/joshkirklin/ugps-cpp

# Include any dependencies generated for this target.
include thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/depend.make

# Include the progress variables for this target.
include thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/progress.make

# Include the compile flags for this target's objects.
include thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/flags.make

thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/gtest/src/gtest-all.cc.o: thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/flags.make
thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/gtest/src/gtest-all.cc.o: thirdparty/gmock-1.7.0/gtest/src/gtest-all.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/joshkirklin/ugps-cpp/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/gtest/src/gtest-all.cc.o"
	cd /home/joshkirklin/ugps-cpp/thirdparty/gmock-1.7.0 && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/gmock_main.dir/gtest/src/gtest-all.cc.o -c /home/joshkirklin/ugps-cpp/thirdparty/gmock-1.7.0/gtest/src/gtest-all.cc

thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/gtest/src/gtest-all.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gmock_main.dir/gtest/src/gtest-all.cc.i"
	cd /home/joshkirklin/ugps-cpp/thirdparty/gmock-1.7.0 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/joshkirklin/ugps-cpp/thirdparty/gmock-1.7.0/gtest/src/gtest-all.cc > CMakeFiles/gmock_main.dir/gtest/src/gtest-all.cc.i

thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/gtest/src/gtest-all.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gmock_main.dir/gtest/src/gtest-all.cc.s"
	cd /home/joshkirklin/ugps-cpp/thirdparty/gmock-1.7.0 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/joshkirklin/ugps-cpp/thirdparty/gmock-1.7.0/gtest/src/gtest-all.cc -o CMakeFiles/gmock_main.dir/gtest/src/gtest-all.cc.s

thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/gtest/src/gtest-all.cc.o.requires:

.PHONY : thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/gtest/src/gtest-all.cc.o.requires

thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/gtest/src/gtest-all.cc.o.provides: thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/gtest/src/gtest-all.cc.o.requires
	$(MAKE) -f thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/build.make thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/gtest/src/gtest-all.cc.o.provides.build
.PHONY : thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/gtest/src/gtest-all.cc.o.provides

thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/gtest/src/gtest-all.cc.o.provides.build: thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/gtest/src/gtest-all.cc.o


thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/src/gmock-all.cc.o: thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/flags.make
thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/src/gmock-all.cc.o: thirdparty/gmock-1.7.0/src/gmock-all.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/joshkirklin/ugps-cpp/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/src/gmock-all.cc.o"
	cd /home/joshkirklin/ugps-cpp/thirdparty/gmock-1.7.0 && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/gmock_main.dir/src/gmock-all.cc.o -c /home/joshkirklin/ugps-cpp/thirdparty/gmock-1.7.0/src/gmock-all.cc

thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/src/gmock-all.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gmock_main.dir/src/gmock-all.cc.i"
	cd /home/joshkirklin/ugps-cpp/thirdparty/gmock-1.7.0 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/joshkirklin/ugps-cpp/thirdparty/gmock-1.7.0/src/gmock-all.cc > CMakeFiles/gmock_main.dir/src/gmock-all.cc.i

thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/src/gmock-all.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gmock_main.dir/src/gmock-all.cc.s"
	cd /home/joshkirklin/ugps-cpp/thirdparty/gmock-1.7.0 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/joshkirklin/ugps-cpp/thirdparty/gmock-1.7.0/src/gmock-all.cc -o CMakeFiles/gmock_main.dir/src/gmock-all.cc.s

thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/src/gmock-all.cc.o.requires:

.PHONY : thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/src/gmock-all.cc.o.requires

thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/src/gmock-all.cc.o.provides: thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/src/gmock-all.cc.o.requires
	$(MAKE) -f thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/build.make thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/src/gmock-all.cc.o.provides.build
.PHONY : thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/src/gmock-all.cc.o.provides

thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/src/gmock-all.cc.o.provides.build: thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/src/gmock-all.cc.o


thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/src/gmock_main.cc.o: thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/flags.make
thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/src/gmock_main.cc.o: thirdparty/gmock-1.7.0/src/gmock_main.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/joshkirklin/ugps-cpp/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/src/gmock_main.cc.o"
	cd /home/joshkirklin/ugps-cpp/thirdparty/gmock-1.7.0 && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/gmock_main.dir/src/gmock_main.cc.o -c /home/joshkirklin/ugps-cpp/thirdparty/gmock-1.7.0/src/gmock_main.cc

thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/src/gmock_main.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gmock_main.dir/src/gmock_main.cc.i"
	cd /home/joshkirklin/ugps-cpp/thirdparty/gmock-1.7.0 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/joshkirklin/ugps-cpp/thirdparty/gmock-1.7.0/src/gmock_main.cc > CMakeFiles/gmock_main.dir/src/gmock_main.cc.i

thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/src/gmock_main.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gmock_main.dir/src/gmock_main.cc.s"
	cd /home/joshkirklin/ugps-cpp/thirdparty/gmock-1.7.0 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/joshkirklin/ugps-cpp/thirdparty/gmock-1.7.0/src/gmock_main.cc -o CMakeFiles/gmock_main.dir/src/gmock_main.cc.s

thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/src/gmock_main.cc.o.requires:

.PHONY : thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/src/gmock_main.cc.o.requires

thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/src/gmock_main.cc.o.provides: thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/src/gmock_main.cc.o.requires
	$(MAKE) -f thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/build.make thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/src/gmock_main.cc.o.provides.build
.PHONY : thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/src/gmock_main.cc.o.provides

thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/src/gmock_main.cc.o.provides.build: thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/src/gmock_main.cc.o


# Object files for target gmock_main
gmock_main_OBJECTS = \
"CMakeFiles/gmock_main.dir/gtest/src/gtest-all.cc.o" \
"CMakeFiles/gmock_main.dir/src/gmock-all.cc.o" \
"CMakeFiles/gmock_main.dir/src/gmock_main.cc.o"

# External object files for target gmock_main
gmock_main_EXTERNAL_OBJECTS =

lib/libgmock_main.a: thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/gtest/src/gtest-all.cc.o
lib/libgmock_main.a: thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/src/gmock-all.cc.o
lib/libgmock_main.a: thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/src/gmock_main.cc.o
lib/libgmock_main.a: thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/build.make
lib/libgmock_main.a: thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/joshkirklin/ugps-cpp/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX static library ../../lib/libgmock_main.a"
	cd /home/joshkirklin/ugps-cpp/thirdparty/gmock-1.7.0 && $(CMAKE_COMMAND) -P CMakeFiles/gmock_main.dir/cmake_clean_target.cmake
	cd /home/joshkirklin/ugps-cpp/thirdparty/gmock-1.7.0 && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/gmock_main.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/build: lib/libgmock_main.a

.PHONY : thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/build

thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/requires: thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/gtest/src/gtest-all.cc.o.requires
thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/requires: thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/src/gmock-all.cc.o.requires
thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/requires: thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/src/gmock_main.cc.o.requires

.PHONY : thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/requires

thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/clean:
	cd /home/joshkirklin/ugps-cpp/thirdparty/gmock-1.7.0 && $(CMAKE_COMMAND) -P CMakeFiles/gmock_main.dir/cmake_clean.cmake
.PHONY : thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/clean

thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/depend:
	cd /home/joshkirklin/ugps-cpp && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/joshkirklin/ugps-cpp /home/joshkirklin/ugps-cpp/thirdparty/gmock-1.7.0 /home/joshkirklin/ugps-cpp /home/joshkirklin/ugps-cpp/thirdparty/gmock-1.7.0 /home/joshkirklin/ugps-cpp/thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : thirdparty/gmock-1.7.0/CMakeFiles/gmock_main.dir/depend

