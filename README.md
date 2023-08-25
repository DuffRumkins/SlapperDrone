# SlapperDrone

This repository contains the base versions of the vision and control algorithms built for the Slapper perching drone. This takes the form of two ROS1 melodic nodes, slapper\_control and slapper\_vision. 

## Vision

The vision algorithm uses an implementation of the RANSAC algorithm to detect (imperfect) cylindrical structures from a point cloud video stream. The dimensions and poses of the structures that are searched for can be modified using RQT dynamic reconfigure or by modifying the launch files. An example parameter file is included that was found to work for the Slapper drone during tests in the TU Delft Cyberzoo.

## Control

The control node contains various planning and control scripts that are designed to be run on board drone hardware. These are just the base forms and will need modification depending on the drone platform used. Among others, there are nodes for tracking a detected perch undergoing slight distrubances (eg. wind), approaching a detected perch, and validating if a direct approach to a detected perch is feasible.
