# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 4.0

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
CMAKE_COMMAND = /snap/cmake/1457/bin/cmake

# The command to remove a file.
RM = /snap/cmake/1457/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/siddarth/FRR-Software-Interface/Project_CPP

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/siddarth/FRR-Software-Interface/Project_CPP/build

# Include any dependencies generated for this target.
include CMakeFiles/my_sim.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/my_sim.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/my_sim.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/my_sim.dir/flags.make

CMakeFiles/my_sim.dir/codegen:
.PHONY : CMakeFiles/my_sim.dir/codegen

CMakeFiles/my_sim.dir/main.cpp.o: CMakeFiles/my_sim.dir/flags.make
CMakeFiles/my_sim.dir/main.cpp.o: /home/siddarth/FRR-Software-Interface/Project_CPP/main.cpp
CMakeFiles/my_sim.dir/main.cpp.o: CMakeFiles/my_sim.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/siddarth/FRR-Software-Interface/Project_CPP/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/my_sim.dir/main.cpp.o"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/my_sim.dir/main.cpp.o -MF CMakeFiles/my_sim.dir/main.cpp.o.d -o CMakeFiles/my_sim.dir/main.cpp.o -c /home/siddarth/FRR-Software-Interface/Project_CPP/main.cpp

CMakeFiles/my_sim.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/my_sim.dir/main.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/siddarth/FRR-Software-Interface/Project_CPP/main.cpp > CMakeFiles/my_sim.dir/main.cpp.i

CMakeFiles/my_sim.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/my_sim.dir/main.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/siddarth/FRR-Software-Interface/Project_CPP/main.cpp -o CMakeFiles/my_sim.dir/main.cpp.s

# Object files for target my_sim
my_sim_OBJECTS = \
"CMakeFiles/my_sim.dir/main.cpp.o"

# External object files for target my_sim
my_sim_EXTERNAL_OBJECTS =

my_sim: CMakeFiles/my_sim.dir/main.cpp.o
my_sim: CMakeFiles/my_sim.dir/build.make
my_sim: CMakeFiles/my_sim.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/siddarth/FRR-Software-Interface/Project_CPP/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable my_sim"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/my_sim.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/my_sim.dir/build: my_sim
.PHONY : CMakeFiles/my_sim.dir/build

CMakeFiles/my_sim.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/my_sim.dir/cmake_clean.cmake
.PHONY : CMakeFiles/my_sim.dir/clean

CMakeFiles/my_sim.dir/depend:
	cd /home/siddarth/FRR-Software-Interface/Project_CPP/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/siddarth/FRR-Software-Interface/Project_CPP /home/siddarth/FRR-Software-Interface/Project_CPP /home/siddarth/FRR-Software-Interface/Project_CPP/build /home/siddarth/FRR-Software-Interface/Project_CPP/build /home/siddarth/FRR-Software-Interface/Project_CPP/build/CMakeFiles/my_sim.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/my_sim.dir/depend

