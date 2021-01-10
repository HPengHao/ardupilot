#!/bin/bash
/home/bob/ardupilot_3.4.0/Tools/autotest/sim_vehicle.py -f gazebo-iris --map --console  -m --out=udpbcast:192.168.48.255:14550 -w
