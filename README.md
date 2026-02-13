# QUniBone
This is the software for both
Linux-to-UNIBUS bridge "UniBone"
and
Linux-to-QBUS bridge "QBone"

"UniBone" connects a BeagleBone Black micro Linux system to ancient DEC UNIBUS,
"QBone" does the same for DEC QBUS.

UniBone/QBone can keep old PDP-11s running, by emulating devices and aiding in repair.

As UNIBUS and QBUS are quite similar, only one software project compiles for both devices.

In-source differentiation is done via "#define UNIBUS" or "#define QBUS".
Source files special to only one bus are marked with suffix "_u" respective "_q".

See project pages at retrocmp.com [for UniBone](http://retrocmp.com/projects/unibone/) and [for QBone](http://retrocmp.com/projects/qbone/)

# STATUS OF THIS FORK

This is just my own local modifications for my specific PDP-11/34A.

I only intend to work on this as free time and my personal situation allows.

No effort was made to make any of my code conformant to any particular language standard or formatting style beyond that which I personally use.

No effort was made to ensure code cleanliness, coverage, or testability.

Files touched may or may not have gained UTF-8 characters.

# MODIFICATIONS ON THIS FORK

None of these are considered to be "production-ready" or even complete.

* 10.01_base/2_src/arm/device.*pp: Added callback to notify device when its worker is terminating so it can delete itself.
* 10.02_devices/2_src/rl0102.cpp: Made type read-write so I can have RL01s. This appears to have been an oversight.
* 10.02_devices/2_src/storagedrive.*pp: Added RK06 and RK07, and a function to determine if the image file is not null.
* 10.02_devices/2_src/storageimage.*pp: Added rawfile type for tapes.
* 10.03_app_demo/2_src/makefile_u and 10.03_app_demo/2_src/menu_devices.cpp: Added new items described below.

# NEW ITEMS ON THIS FORK

None of these are considered to be "production-ready" or even complete.

* 10.02_devices/2_src/dz11.*pp: DZ11 terminal multiplexer. Makes RSTS happy.
* 10.02_devices/2_src/lp11.*pp: LP11 printer controller. Makes RSTS happy.
* 10.02_devices/2_src/rk611.*pp, 10.02_devices/2_src/rk067.*pp: RK611 controller and RK06/7 disk drives. Makes RSTS happy.
* 10.02_devices/2_src/tm11.*pp, 10.02_devices/2_src/tm11_drive.*pp: TM11B controller and TM03 tape drives. Placeholder only.
* 10.02_devices/2_src/ts11.*pp, 10.02_devices/2_src/ts11_drive.*pp: TS11 controller and TS04 tape drive. Incomplete.
* 10.02_devices/2_src/netcom.*pp: Networking (inbound TCP connections) for terminal multiplexers. Makes RSTS happy.
* 10.02_devices/2_src/parity.*pp: Parity tables for things that need them. 
* 10.02_devices/2_src/tapecontroller.*pp: Initial support for tape images. Might get merged into storageimage later, might not.
* 10.02_devices/2_src/tapedrive.*pp: Emulated tape transports interface.

# INTENDED FUTURE WORK

* Interlan NI1010A, DH11 terminal multiplexer, and/or CH11 to support MINITS once my PDP-10 is repaired.

