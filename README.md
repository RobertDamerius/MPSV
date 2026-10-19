![](documentation/img/icon.svg)

# MPSV
The **M**otion **P**lanner for **S**urface **V**ehicles solves motion planning problems for fully actuated surface vehicles with non-linear dynamics in an environment with non-convex non-moving obstacles.
The method is based on the RRT* algorithm.
The problem is decomposed into two sequential problems in which first a feasible path is searched for and then a collision-free trajectory is calculated based on the path found.
For references, see [below](#publication).


![](documentation/img/Overview.svg)


## Header-only library with MATLAB/Simulink support
The MPSV solver is implemented as header-only library (C++) and requires a compiler that supports the C++23 language features.
There exists a MATLAB/Simulink toolbox that allows you to run the planner in Simulink.
To use the toolbox, open the [matlab/MPSV.prj](matlab/MPSV.prj) project file or add it as reference project to an existing MATLAB project.
Run `mpsv.BuildDrivers()` once to build all driver blocks of the simulink library.
All blocks are supported for C++ code generation.

**OpenMP**

MPSV requires OpenMP for parallel computing.
It has been tested on Windows via MSYS2/MinGW64 and Ubuntu.
The package usually comes with GCC.
On Ubuntu it can be installed via `sudo apt install libomp-dev`, for example.
It may be necessary to set the environment variables `OMP_PLACES` or `OMP_PROC_BIND`.

## Overview
The directory structure of this repository is as follows.

| File / Directory   | Description                                               |
| :----------------- | :-------------------------------------------------------- |
| apps               | contains example applications (C++)                       |
| documentation      | contains an HTML-based documentation                      |
| include            | contains the header-only library (C++)                    |
| matlab             | contains the MATLAB/Simulink project for the MPSV toolbox |
| LICENSE            | license information                                       |
| Makefile           | used to build example applications                        |
| README.md          | this file                                                 |


## Planning Levels
The API of MPSV consists of several planning levels that are shown in the following image.

![](documentation/img/PlanningLevels.svg)

**PathPlanner**
- find path from initial pose to final pose
- RRT* with fixed number of nodes and warm start capabilities

**MotionPlanner**
- find trajectory from initial state/input to final pose around a given path
- RRT* with fixed number of nodes
- internal control system based on feedback linearization and full-state feedback to generate dynamically feasible trajectory

**SequentialPlanner**
- combine path and motion planner in sequential manner
- path from path planner is used as guidance for the motion planner

**OnlinePlanner**
- continuously solve sequential planning problems
- solves motion planning problems for a future timepoint
- merges solution trajectory seamless with a previous solution

**AsyncOnlinePlanner**
- run the online planner in a separate thread
- results are obtained via polling or callback

When using the header-only library the desired planning level can be used directly in a custom C++ application (see examples).
However, when using MATLAB/Simulink, only the *PathPlanner* and the *AsyncOnlinePlanner* are currently implemented.


## Examples (C++)
Take a look to example applications in the [apps](apps/) directory on how to use the different planning levels of MPSV.

#### Building a single example
To build a specific application example, run
```
make app=NAME
```
where `NAME` has to be replaced by the name of the application, e.g. the folder name of the application to be built.

#### Building all examples
To build all example applications, run
```
make apps
```


## Examples (MATLAB/Simulink)
Example models for Simulink are located at [matlab/examples/](matlab/examples/).
Make sure that you build already build the driver blocks of the Simulink library with `mpsv.BuildDrivers()`.


## Publication
*Robert Damerius, Nick Eisenblätter, Johannes R. Marx, Muhammad Khalis Mohd Fadil and Torsten Jeinsch, Real-Time Motion Planning for Fully Actuated Autonomous Surface Vehicles, 34th Mediterranean Conference on Control and Automation (MED), 2026, doi: 10.1109/MED70602.2026.11598383*
```
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
```
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
```
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
```
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

