# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.24

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/jadondutra/Code/SpecGen

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jadondutra/Code/SpecGen/build

# Include any dependencies generated for this target.
include CMakeFiles/SpecGen.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/SpecGen.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/SpecGen.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/SpecGen.dir/flags.make

CMakeFiles/SpecGen.dir/main.cpp.o: CMakeFiles/SpecGen.dir/flags.make
CMakeFiles/SpecGen.dir/main.cpp.o: /home/jadondutra/Code/SpecGen/main.cpp
CMakeFiles/SpecGen.dir/main.cpp.o: CMakeFiles/SpecGen.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jadondutra/Code/SpecGen/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/SpecGen.dir/main.cpp.o"
	/usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/SpecGen.dir/main.cpp.o -MF CMakeFiles/SpecGen.dir/main.cpp.o.d -o CMakeFiles/SpecGen.dir/main.cpp.o -c /home/jadondutra/Code/SpecGen/main.cpp

CMakeFiles/SpecGen.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/SpecGen.dir/main.cpp.i"
	/usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jadondutra/Code/SpecGen/main.cpp > CMakeFiles/SpecGen.dir/main.cpp.i

CMakeFiles/SpecGen.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/SpecGen.dir/main.cpp.s"
	/usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jadondutra/Code/SpecGen/main.cpp -o CMakeFiles/SpecGen.dir/main.cpp.s

CMakeFiles/SpecGen.dir/src/actuators.cpp.o: CMakeFiles/SpecGen.dir/flags.make
CMakeFiles/SpecGen.dir/src/actuators.cpp.o: /home/jadondutra/Code/SpecGen/src/actuators.cpp
CMakeFiles/SpecGen.dir/src/actuators.cpp.o: CMakeFiles/SpecGen.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jadondutra/Code/SpecGen/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/SpecGen.dir/src/actuators.cpp.o"
	/usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/SpecGen.dir/src/actuators.cpp.o -MF CMakeFiles/SpecGen.dir/src/actuators.cpp.o.d -o CMakeFiles/SpecGen.dir/src/actuators.cpp.o -c /home/jadondutra/Code/SpecGen/src/actuators.cpp

CMakeFiles/SpecGen.dir/src/actuators.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/SpecGen.dir/src/actuators.cpp.i"
	/usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jadondutra/Code/SpecGen/src/actuators.cpp > CMakeFiles/SpecGen.dir/src/actuators.cpp.i

CMakeFiles/SpecGen.dir/src/actuators.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/SpecGen.dir/src/actuators.cpp.s"
	/usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jadondutra/Code/SpecGen/src/actuators.cpp -o CMakeFiles/SpecGen.dir/src/actuators.cpp.s

# Object files for target SpecGen
SpecGen_OBJECTS = \
"CMakeFiles/SpecGen.dir/main.cpp.o" \
"CMakeFiles/SpecGen.dir/src/actuators.cpp.o"

# External object files for target SpecGen
SpecGen_EXTERNAL_OBJECTS =

SpecGen: CMakeFiles/SpecGen.dir/main.cpp.o
SpecGen: CMakeFiles/SpecGen.dir/src/actuators.cpp.o
SpecGen: CMakeFiles/SpecGen.dir/build.make
SpecGen: CMakeFiles/SpecGen.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jadondutra/Code/SpecGen/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable SpecGen"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/SpecGen.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/SpecGen.dir/build: SpecGen
.PHONY : CMakeFiles/SpecGen.dir/build

CMakeFiles/SpecGen.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/SpecGen.dir/cmake_clean.cmake
.PHONY : CMakeFiles/SpecGen.dir/clean

CMakeFiles/SpecGen.dir/depend:
	cd /home/jadondutra/Code/SpecGen/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jadondutra/Code/SpecGen /home/jadondutra/Code/SpecGen /home/jadondutra/Code/SpecGen/build /home/jadondutra/Code/SpecGen/build /home/jadondutra/Code/SpecGen/build/CMakeFiles/SpecGen.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/SpecGen.dir/depend

