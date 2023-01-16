# Danitech-master

## Import submodules
This repository includes submodules for some of its packages for easier management. 
Download these submodules with the following commands:

```
git submodule init
git submodule update
```

And everything should be able to build


## Start GPS module with RTK correction through NTRIP
To use the ublox gps module with proper localization both the NTRIP and ublox node needs to be launched with the following
```
ros2 launch rtk_ntrip_client rtk_ntrip_client.launch.py username:=*** password:="'***'"
ros2 launch sensors_launch ublox_gps_node-launch.py
```

The username and password has to be given as a parameter for security reasons. The double quotation marks are to make sure it is read as a string.
Assuming you are using the default NavSatFix gps mode for the ntrip client, the node will subscribe to ```/rtk/fix``` published by ublox. The corrections will then be published on ```/rtk/rtcm```, which the ublox node will use to update the localization. The ```/rtk/navpvt``` is for status information. 