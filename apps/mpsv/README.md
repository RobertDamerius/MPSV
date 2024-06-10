# MPSV Standalone Application
This directory contains the standalone binary as well as source files and protocol files.
The application is designed to run on a linux computer with the PREEMPT_RT patch.
Commands can be sent to the application via UDP messages.
The results are also sent to a specified destination address via UDP.
The application is configured using the ``MPSV.json`` file.


#### Configure network buffers
By default the network buffers for sending and receiving messages are limited to a fairly small value of about 100 KB or 200 KB depending on the distribution. To increase the maximum size of those buffers to 36 MB for example, run
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
