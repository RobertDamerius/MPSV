# Benchmark
This directory contains the source code of the benchmark application.
The application is built by running `make app=benchmark` in the repository root directory.
The binaries will be built to this directory.

The application runs different tests using single-threading and multi-threading and measures execution time.
Each test is performed 10 times and the average as well as min/max value and standard deviation are calculated.
The results are printed to the console output.
There are the following benchmark tests:

> **1. Polygon Overlap**
> 
> Counts the number of overlaps of 10000 triangles.

> **2. Cost Map (Single Thread)**
> 
> Creates a 100-by-100 look-up table and calculate a cost function value for each cell. The cost function is defined to be the minimum squared distance of a cell to all edges of 10000 triangles.

> **3. Cost Map (Multi Thread)**
> 
> Performs the same calculation as in benchmark 2.
> However, all cells of the look-up table are processed in parallel.

> **4. Forward Simulation**
> 
> Predicts the motion of a non-linear dynamical system for 1000 different initial states over a time range of 10000 discrete-time steps.

> **5. Path Planning**
> 
> Solves a realistic path planning problem with 126 static obstacles and 50000 iterations.

> **6. Motion Planning**
> 
> Solves a realistic path planning problem with 126 static obstacles and 400 iterations.

## Running the Benchmark
Simply run the benchmark application from the command line, e.g. (Linux)
```
./benchmark
```

On some systems like debian with the PREEMPT_RT patch the default behaviour of OpenMP (OMP) might change depending on how the application is executed.
Pay attention when using `taskset` to pin the program to specific cores.
When running the benchmark on multiple cores, benchmark 3 should always be faster than benchmark 2.
If this is not the case, try to explicitly set the environment variable `OMP_PROC_BIND` to `true`, for example by running:
```
OMP_PROC_BIND=true taskset 0xff ./benchmark
```
