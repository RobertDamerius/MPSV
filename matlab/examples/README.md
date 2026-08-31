# MPSV Examples (MATLAB/Simulink)
This directory contains example models that show how to use simulink library to solve planning problems.

| Model                | Description                                                                                                            |
| :------------------- | :--------------------------------------------------------------------------------------------------------------------- |
| PathPlanning         | solves a path planning problem in each step of the simulation                                                          |
| AsyncOnlinePlanning  | solves an online planning problem asynchronously in a separate thread and polls results in each step of the simulation |
| Standalone           | solves an online planning problem asynchronously by outsourcing the calculation to the stand alone application         |

The folder [ExplorePath](ExplorePath/) gives an example on how to design a controller for path exploration which is required for motion planning.
