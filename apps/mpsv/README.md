# MPSV Standalone Application
This directory contains the source code of the standalone application.
The application is built by running `make app=mpsv` in the repository root directory.
The binaries will be built to this directory.

The application is designed to run on a linux computer with the PREEMPT_RT patch.
Commands can be sent to the application via UDP messages.
The results are also sent to a specified destination address via UDP.
Configuration can be done via the `MPSV.json` file.

#### UDP Messages
The following messages have to be send to the standalone application via UDP:
  - **Input**: navigation data such as pose and velocity as well as static obstacles and the goal pose
  - **Parameter**: geometric vehicle shape, tuning parameters for cost function and hyper parameters for planning algorithms

The following messages are send from the standalone application via UDP:
  - **Output**: computed path and trajectory as well as status information about the planner

#### Protocol
For all messages, the machine byte-order is used.
All messages have a constant size.
The format for all messages is given in as structures in the header file [mpsv/planner/Serialization.hpp](../../source/mpsv/planner/Serialization.hpp).


#### Linux: Configure Network Buffers
By default the network buffers for sending and receiving messages are limited to a fairly small value of a few hundred KB, depending on the distribution.
To increase the maximum size of those buffers to several MB, run
```
sudo sysctl -w net.core.rmem_max=33554432
sudo sysctl -w net.core.wmem_max=33554432
```

## How To Launch
To start the application manually, run the following command in this directory:
```
sudo ./mpsv
```

If the output should be printed to the console instead of being redirected into a protocol file, run
```
sudo ./mpsv --console
```

If the application is to be started without waiting for termination, the following command can be executed.
```
sudo nohup ./mpsv &
```


## How To Autostart
If you want to autostart the application via a cron job, edit the crontab via
```
sudo crontab -e
```
and add the line
```
@reboot /home/UserName/PathToApplication/mpsv >/dev/null 2>&1
```
where ``UserName`` and ``PathToApplication`` have to be modified.
If the application should be started on specific cpu cores, e.g. 0,1,2,3, modify the line to
```
@reboot taskset 0x0f /home/UserName/PathToApplication/mpsv >/dev/null 2>&1
```
