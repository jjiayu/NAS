# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.31

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
CMAKE_COMMAND = /Applications/CMake.app/Contents/bin/cmake

# The command to remove a file.
RM = /Applications/CMake.app/Contents/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/jiayu/Desktop/nas_ws/NAS

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/jiayu/Desktop/nas_ws/NAS/build

# Include any dependencies generated for this target.
include CMakeFiles/nas_plan.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/nas_plan.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/nas_plan.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/nas_plan.dir/flags.make

CMakeFiles/nas_plan.dir/codegen:
.PHONY : CMakeFiles/nas_plan.dir/codegen

CMakeFiles/nas_plan.dir/src/nas_plan.cpp.o: CMakeFiles/nas_plan.dir/flags.make
CMakeFiles/nas_plan.dir/src/nas_plan.cpp.o: /Users/jiayu/Desktop/nas_ws/NAS/src/nas_plan.cpp
CMakeFiles/nas_plan.dir/src/nas_plan.cpp.o: CMakeFiles/nas_plan.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/jiayu/Desktop/nas_ws/NAS/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/nas_plan.dir/src/nas_plan.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/nas_plan.dir/src/nas_plan.cpp.o -MF CMakeFiles/nas_plan.dir/src/nas_plan.cpp.o.d -o CMakeFiles/nas_plan.dir/src/nas_plan.cpp.o -c /Users/jiayu/Desktop/nas_ws/NAS/src/nas_plan.cpp

CMakeFiles/nas_plan.dir/src/nas_plan.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/nas_plan.dir/src/nas_plan.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/jiayu/Desktop/nas_ws/NAS/src/nas_plan.cpp > CMakeFiles/nas_plan.dir/src/nas_plan.cpp.i

CMakeFiles/nas_plan.dir/src/nas_plan.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/nas_plan.dir/src/nas_plan.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/jiayu/Desktop/nas_ws/NAS/src/nas_plan.cpp -o CMakeFiles/nas_plan.dir/src/nas_plan.cpp.s

# Object files for target nas_plan
nas_plan_OBJECTS = \
"CMakeFiles/nas_plan.dir/src/nas_plan.cpp.o"

# External object files for target nas_plan
nas_plan_EXTERNAL_OBJECTS =

nas_plan: CMakeFiles/nas_plan.dir/src/nas_plan.cpp.o
nas_plan: CMakeFiles/nas_plan.dir/build.make
nas_plan: libtree_lib.dylib
nas_plan: libutils_lib.dylib
nas_plan: /opt/homebrew/lib/libvtkInteractionStyle-9.4.9.4.dylib
nas_plan: /opt/homebrew/lib/libvtkRenderingOpenGL2-9.4.9.4.dylib
nas_plan: /opt/homebrew/lib/libvtkRenderingFreeType-9.4.9.4.dylib
nas_plan: libnode_lib.dylib
nas_plan: /opt/homebrew/lib/libvtkIOImage-9.4.9.4.dylib
nas_plan: /opt/homebrew/lib/libvtkRenderingHyperTreeGrid-9.4.9.4.dylib
nas_plan: /opt/homebrew/lib/libvtkImagingCore-9.4.9.4.dylib
nas_plan: /opt/homebrew/lib/libvtkRenderingUI-9.4.9.4.dylib
nas_plan: /opt/homebrew/lib/libvtkglad-9.4.9.4.dylib
nas_plan: /opt/homebrew/lib/libvtkRenderingCore-9.4.9.4.dylib
nas_plan: /opt/homebrew/lib/libvtkFiltersSources-9.4.9.4.dylib
nas_plan: /opt/homebrew/lib/libvtkFiltersGeneral-9.4.9.4.dylib
nas_plan: /opt/homebrew/lib/libvtkFiltersCore-9.4.9.4.dylib
nas_plan: /opt/homebrew/lib/libvtkCommonExecutionModel-9.4.9.4.dylib
nas_plan: /opt/homebrew/lib/libvtkCommonDataModel-9.4.9.4.dylib
nas_plan: /opt/homebrew/lib/libvtkCommonTransforms-9.4.9.4.dylib
nas_plan: /opt/homebrew/lib/libvtkCommonMisc-9.4.9.4.dylib
nas_plan: /opt/homebrew/lib/libvtkCommonMath-9.4.9.4.dylib
nas_plan: /opt/homebrew/lib/libvtkkissfft-9.4.9.4.dylib
nas_plan: /opt/homebrew/lib/libvtkCommonCore-9.4.9.4.dylib
nas_plan: /opt/homebrew/lib/libvtktoken-9.4.9.4.dylib
nas_plan: /opt/homebrew/lib/libvtksys-9.4.9.4.dylib
nas_plan: /opt/homebrew/lib/libfreetype.dylib
nas_plan: /opt/homebrew/lib/libgmpxx.dylib
nas_plan: /opt/homebrew/lib/libmpfr.dylib
nas_plan: /opt/homebrew/lib/libgmp.dylib
nas_plan: CMakeFiles/nas_plan.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/Users/jiayu/Desktop/nas_ws/NAS/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable nas_plan"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/nas_plan.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/nas_plan.dir/build: nas_plan
.PHONY : CMakeFiles/nas_plan.dir/build

CMakeFiles/nas_plan.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/nas_plan.dir/cmake_clean.cmake
.PHONY : CMakeFiles/nas_plan.dir/clean

CMakeFiles/nas_plan.dir/depend:
	cd /Users/jiayu/Desktop/nas_ws/NAS/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/jiayu/Desktop/nas_ws/NAS /Users/jiayu/Desktop/nas_ws/NAS /Users/jiayu/Desktop/nas_ws/NAS/build /Users/jiayu/Desktop/nas_ws/NAS/build /Users/jiayu/Desktop/nas_ws/NAS/build/CMakeFiles/nas_plan.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/nas_plan.dir/depend

