Drone vision hovering Project
===============================
Code meant to be run on Dronesmith Luci to test drone hovering algorithms. 
Previously Prof. Jianxinliu and I worked on this drone vision hovering project. This work has been published on ICUAS 2016 with tests on 3DR DIY platform. Now I am trying to move this work to Dronesmith platform and do more tests indoors. Hopefully, we can get those tests done before the deadline (the end of August,2016) of the special issue in JINT.

Hardware Settings
=================================
Dronesmith Luci platform with breakout board. A UVC camera is connected to the breakout board.

Software Prerequisites
================================
Adding repos;
Python, pip, dronekit 2.8

`pip install pymavlink`

`pip install pyserial`

First Run
=================================
Kill dslink;

`cd scripts`
`python objtrack-fopi.py` 

PS: Current progress:
https://files.slack.com/files-tmb/T0M3RTFQ9-F20ATJP9C-a5384b4bf5/pasted_image_at_2016_08_10_11_19_pm_720.png



Thank the project of ardupilot-balloon-finder
========================

Code meant to be run on an Odroid to allow an ArduCopter Pixhawk based multicopter to find red balloons for Sparkfun's AVC 2014 competition

Installation instructions can be found here: http://dev.ardupilot.com/wiki/odroid-via-mavlink/#Red_Balloon_Finder

Thank Dronesmith
==============================
http://www.dronesmith.io/
https://github.com/Dronesmith-tech/Simple-Dual-Cam


