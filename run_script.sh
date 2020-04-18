#!/bin/bash

source devel/setup.bash
rosrun drone_rl takeoff.py
rosrun drone_rl yaw.py