# Danitech-master
## Read this before continuing!
Welcome to the GitHub repository for our masters thesis "Autonomous driving of articulated steered vehicle in uneven terrain".
We worked on creating a software suite for autonomous navigation in an uneven unstructured terrain with multiple levels.

Before going too deep in this repository, we want to remind you that we attached this as a courtesy if you for any reason was interested in a deeper dive.
It was never the intention that this repository would become available to anybody but us.
We used this repository as a sandbox, which means it contains experiments, prototypes and some horrible code. 
Not all the files seen here are part of the final project, but were part of the journey, so don't expect anything polished if you're going to a random file.

Best of luck.

## Import submodules
This repository includes submodules for some of its packages for easier management. 
Download these submodules with the following commands:

```
git submodule init
git submodule update
```

And everything should be able to build


## Start ROS2 Connection
Due to the network configuration at the university, fast rtps (or simply the transportlayer of ROS2) is setup to use a **discovery server** and a **super client**, which has to be started before ROS2 will properly work between machines (and locally out of the box).
Start the discovery server with 
```
fastdds discovery -i 0 -l IP -p 11811
```
Or the alias ```discovery-server``` 
To properly connect to the server for communication, a xml config has to be exported to fastrtps with:
```
export FASTRTPS_DEFAULT_PROFILES_FILE=~/.ros/fastdds.xml
```
where the [xml file](https://github.com/drengene/Danitech-master/blob/main/other/fastdds.xml) has the IP of the server, to set the client up as a super client, so all topics can be discovered, and not just when specifically asked for a connection.
The xml file is 


## Start GPS module with RTK correction through NTRIP
To use the ublox gps module with proper localization both the NTRIP and ublox node needs to be launched with the following
```
ros2 launch rtk_ntrip_client rtk_ntrip_client.launch.py username:=*** password:="'***'"
ros2 launch sensors_launch ublox_gps_node-launch.py
```

The username and password has to be given as a parameter for security reasons. The double quotation marks are to make sure it is read as a string.
Assuming you are using the default NavSatFix gps mode for the ntrip client, the node will subscribe to ```/rtk/fix``` published by ublox. The corrections will then be published on ```/rtk/rtcm```, which the ublox node will use to update the localization. The ```/rtk/navpvt``` is for status information. 
