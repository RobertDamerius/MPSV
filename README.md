![](documentation/img/icon.svg)

# MPSV

The **M**otion **P**lanner for **S**urface **V**ehicles solves motion planning problems for fully actuated surface vehicles with non-linear dynamics in an environment with non-convex non-moving obstacles.
The method is based on the RRT* algorithm.
The problem is decomposed into two sequential problems in which first a feasible path is searched for and then a collision-free trajectory is calculated based on the path found.
For references, see [below](#publication).


![](documentation/img/Overview.svg)


## Repository Overview

The directory structure of this repository is as follows.

| File / Directory   | Description                                               |
| :----------------- | :-------------------------------------------------------- |
| examples           | contains C++ examples                                     |
| documentation      | contains an HTML-based documentation                      |
| include            | contains the header-only library (C++)                    |
| matlab             | contains the MATLAB/Simulink project for the MPSV toolbox |
| LICENSE            | license information                                       |
| Makefile           | used to build C++ examples                                |
| README.md          | this file                                                 |


## Header-only Library (C++)

The MPSV solver is implemented as a header-only C++ library and requires a compiler with C++23 support.
Include the header as follows:

```cpp
#include <mpsv/mpsv.hpp>

using namespace mpsv;

int main() {
    planner::PathPlanner planner;
    return 0;
}
```

All public types are in the `mpsv` namespace (for example `mpsv::planner` and `mpsv::geometry`).

**OpenMP**

MPSV requires OpenMP for parallel computing.
It has been tested on Windows via MSYS2/MinGW64 and Ubuntu.
On Ubuntu/Debian, install `libomp-dev` (or use the GCC/OpenMP toolchain that ships with your compiler).


## Planning Levels

The API of MPSV consists of several planning levels that are shown in the following image.
The snippets below are illustrative; see the files in [examples](examples/) for complete, compilable examples.

![](documentation/img/PlanningLevels.svg)

### PathPlanner

- find path from initial pose to final pose
- RRT* with fixed number of nodes and warm start capabilities

```cpp
mpsv::planner::PathPlanner planner;

mpsv::planner::PathPlannerInput dataIn = GetInput();
mpsv::planner::PathPlannerParameterSet parameter = GetParameter();
mpsv::planner::PathPlannerOutput dataOut;

constexpr double maxComputationTime = 1.0;

planner.Solve(dataOut, dataIn, parameter, maxComputationTime);
```

### MotionPlanner

- find trajectory from initial state/input to final pose around a given path
- RRT* with fixed number of nodes
- internal control system based on feedback linearization and full-state feedback to generate dynamically feasible trajectory

```cpp
mpsv::planner::MotionPlanner planner;

mpsv::planner::MotionPlannerInput dataIn = GetInput();
mpsv::planner::MotionPlannerParameterSet parameter = GetParameter();
mpsv::planner::MotionPlannerOutput dataOut;

constexpr double maxComputationTime = 1.0;

planner.Solve(dataOut, dataIn, parameter, maxComputationTime);
```

### SequentialPlanner

- combine path and motion planner in sequential manner
- path from path planner is used as guidance for the motion planner

```cpp
mpsv::planner::SequentialPlanner planner;

mpsv::planner::SequentialPlannerInput dataIn = GetInput();
mpsv::planner::SequentialPlannerParameterSet parameter = GetParameter();
mpsv::planner::SequentialPlannerOutput dataOut;

constexpr double maxComputationTimePath = 1.0;
constexpr double maxComputationTimeMotion = 1.0;

planner.Solve(dataOut, dataIn, parameter, maxComputationTimePath, maxComputationTimeMotion);
```

### OnlinePlanner

- continuously solve sequential planning problems
- solves motion planning problems for a future timepoint
- merges solution trajectory seamless with a previous solution

```cpp
mpsv::planner::OnlinePlanner planner;

for(...){
    mpsv::planner::OnlinePlannerInput dataIn = GetInput();
    mpsv::planner::OnlinePlannerParameterSet parameter = GetParameter();
    mpsv::planner::OnlinePlannerOutput dataOut;

    planner.Solve(dataOut, dataIn, parameter);
}
```

### AsyncOnlinePlanner

- run the online planner in a separate thread
- results are obtained via polling or callback

```cpp
constexpr int16_t pathMaxNumNodes      = 200;
constexpr uint32_t pathMaxNumSamples   = 1000000;
constexpr int16_t motionMaxNumNodes    = 200;
constexpr uint32_t motionMaxNumSamples = 1000000;
constexpr uint32_t maxNumCells         = 4096 * 4096;
constexpr int32_t threadPriority       = 20;
constexpr int32_t ompNumThreads        = -1; // ignore
constexpr int32_t ompDynamic           = -1; // ignore
std::vector<int32_t> cpuCoreIDs        = {}; // ignore

mpsv::planner::AsyncOnlinePlanner planner;
planner.Start(pathMaxNumNodes, pathMaxNumSamples, motionMaxNumNodes, motionMaxNumSamples, maxNumCells, threadPriority, ompNumThreads, ompDynamic, cpuCoreIDs);

for(...){
    mpsv::planner::AsyncOnlinePlannerInput dataIn = GetInput();
    mpsv::planner::AsyncOnlinePlannerParameterSet parameter = GetParameter();
    mpsv::planner::AsyncOnlinePlannerOutput dataOut;

    planner.SetInput(dataIn, parameter);
    planner.GetOutput(dataOut);
}

planner.Stop();
```

## Building Examples (C++)

Take a look at the C++ examples in the [examples](examples/) directory to see how to use the different planning levels of MPSV.

To build a specific example, run

```
make example=NAME
```

where `NAME` has to be replaced by the name of the example, e.g. the folder name of the example to be built.

To build all examples, run

```
make examples
```


## MATLAB/Simulink Support

A MATLAB/Simulink toolbox exists that allows you to run the planner in Simulink.
Only the *PathPlanner* and the *AsyncOnlinePlanner* are currently implemented.
To use the toolbox, open the [matlab/MPSV.prj](matlab/MPSV.prj) project file or add it as reference project to an existing MATLAB project.
Run `mpsv.BuildDrivers()` once to build all driver blocks of the simulink library.
All blocks support C++ code generation.

Example models for Simulink are located at [matlab/examples/](matlab/examples/).


## Publication
*Robert Damerius, Nick Eisenblätter, Johannes R. Marx, Muhammad Khalis Mohd Fadil and Torsten Jeinsch, Real-Time Motion Planning for Fully Actuated Autonomous Surface Vehicles, 34th Mediterranean Conference on Control and Automation (MED), 2026, doi: 10.1109/MED70602.2026.11598383*
```bibtex
@inproceedings{Damerius2026,
    author    = {Damerius, Robert and Eisenblätter, Nick and Marx, Johannes R. and Fadil, Muhammad Khalis Mohd and Jeinsch, Torsten},
    booktitle = {34th Mediterranean Conference on Control and Automation (MED)},
    title     = {{Real-Time Motion Planning for Fully Actuated Autonomous Surface Vehicles}},
    year      = {2026},
    month     = {June},
    address   = {Ancona, Italy},
    pages     = {195-200},
    issn      = {2473-3504},
    doi       = {10.1109/MED70602.2026.11598383}
}
```

*Robert Damerius, Beitrag zur Bewegungsplanung autonomer Wasserfahrzeuge, Dissertation Universität Rostock, 2025, doi: 10.18453/rosdok_id00005155*
```bibtex
@book{Damerius2025,
    author    = {Damerius, Robert},
    title     = {{Beitrag zur Bewegungsplanung autonomer Wasserfahrzeuge}},
    year      = {2025},
    address   = {Rostock},
    school    = {Universit{\"a}t Rostock},
    note      = {Dissertation Universit{\"a}t Rostock 2025},
    doi       = {10.18453/rosdok_id00005155}
}
```

*Robert Damerius, Johannes R. Marx and Torsten Jeinsch, Fast Trajectory Generation on a Path using Feedback Linearization, 22nd IFAC World Congress, 2023, doi: 10.1016/j.ifacol.2023.10.796*
```bibtex
@inproceedings{Damerius2023,
    author    = {Robert Damerius and Johannes R. Marx and Torsten Jeinsch},
    title     = {{Fast Trajectory Generation on a Path using Feedback Linearization}},
    booktitle = {22nd World Congress of the International Federation of Automatic Control},
    address   = {Yokohama, Japan},
    month     = {July},
    volume    = {56},
    number    = {2},
    pages     = {10990-10995},
    year      = {2023},
    note      = {22nd IFAC World Congress},
    issn      = {2405-8963},
    doi       = {10.1016/j.ifacol.2023.10.796}
}
```

*Robert Damerius and Torsten Jeinsch, Real-Time Path Planning for Fully Actuated Autonomous Surface Vehicles, 30th Mediterranean Conference on Control and Automation (MED), 2022, doi: 10.1109/MED54222.2022.9837178*
```bibtex
@inproceedings{Damerius2022,
    author    = {Damerius, Robert and Jeinsch, Torsten},
    title     = {{Real-Time Path Planning for Fully Actuated Autonomous Surface Vehicles}},
    booktitle = {30th Mediterranean Conference on Control and Automation (MED)},
    pages     = {508-513},
    year      = {2022},
    month     = {June},
    address   = {Athens, Greece},
    doi       = {10.1109/MED54222.2022.9837178}
}
```
