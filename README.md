# CASSR: Continuous A-Star Search through Reachability for real time footstep planning

## Installation Guide

### Prerequisites
- C++17 or later
- CMake 3.10+
- CGAL library
- CasADi optimization library
- nlohmann/json library
- Boost libraries

### Dependencies Installation (macOS)
```bash
# Install via Homebrew
brew install cgal boost nlohmann-json

# Install CasADi (follow official instructions)
# https://web.casadi.org/get/
```

### Build Instructions
```bash
# Clone the repository and checkout devel branch
git clone -b devel <repository-url>
cd NAS

# Create build directory
mkdir build && cd build

# Configure and build
cmake ..
make

# Run footstep planning
./astar_plan
```

### Notes

- The cmake file is design for Mac OS. You may need to adapt the CmakeLists.txt file for other operation systems. 

- The constants.hpp specifies the hyper parameters. 
