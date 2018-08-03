Drone vision hovering Project
===============================
Drone code for flight tests in an ICUAS 2016 paper.
Vision detecting a red circle on ground. Controlling drone to hover on the red circle.

Citation
========
Bo Shang, Jianxin Liu, Tiebiao Zhao, and YangQuan Chen. Fractional order robust visual servoing control of a quadrotor UAV with larger sampling period. In 2016 International Conference on Unmanned Aircraft Systems (ICUAS), pages 1228-1234. IEEE,
2016.

    @inproceedings{shang2016fractional,
      title={Fractional order robust visual servoing control of a quadrotor {UAV} with larger sampling period},
      author={Shang, Bo and Liu, Jianxin and Zhao, Tiebiao and Chen, YangQuan},
      booktitle={2016 International Conference on Unmanned Aircraft Systems (ICUAS)},
      pages={1228--1234},
      year={2016},
      organization={IEEE}
    }

Hardware Settings
=================================
Odroid, camera, USB Wi-Fi dongle mounted on a 3DR DIY platform.

Software Prerequisites
================================
Adding repos;

Python, pip, dronekit 2.8

`pip install pymavlink`

`pip install pyserial`

First Run
=================================
Test baudrate:

screen /dev/ttyUSB

Kill dslink;

`cd scripts`

`python objtrack-fopi.py` 

Visit: http://192.168.0.102:8081/ to get the GUI. (192.168.0.102 needs to be changed to the drone's IP.)

PS: Current progress:

https://files.slack.com/files-tmb/T0M3RTFQ9-F22355DA4-83cbe3c2ec/pasted_image_at_2016_08_16_10_31_pm_720.png



Thank the project of ardupilot-balloon-finder
========================

Code meant to be run on an Odroid to allow an ArduCopter Pixhawk based multicopter to find red balloons for Sparkfun's AVC 2014 competition

Installation instructions can be found here: http://dev.ardupilot.com/wiki/odroid-via-mavlink/#Red_Balloon_Finder

