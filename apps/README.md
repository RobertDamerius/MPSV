# MPSV Examples (C++)
This directory contains several example applications that show how to use the different planning levels of MPSV.


| App                  | Description                                                                                                    |
| :------------------- | :------------------------------------------------------------------------------------------------------------- |
| mpsv                 | standalone application for the asynchronous online planner                                                     |
| benchmark            | runs several test functions and algorithms of MPSV and measures execution times                                |
| pathplanning         | solves one path planning problem for a limited execution time                                                  |
| motionplanning       | solves one motion planning problem for a limited execution time                                                |
| sequentialplanning   | runs the sequential planner (path and motion planner) for a limited execution time                             |
| onlineplanning       | solves several sequential planning problems in a loop                                                          |
| asynconlineplanning  | solves one asynchronous planning problem in a separate thread and polls for results                            |
| asynconlineplanning2 | solves online planning problems for 120 seconds and writes results to a file                                   |

