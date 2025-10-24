#!/bin/sh
# launcher.sh
# navigate to a directory, then execute a python script, then back home

sudo pigpiod
cd /
sudo -u pi python3.11 /home/pi/loc_mind-main/operational_system/MainCode.py &
