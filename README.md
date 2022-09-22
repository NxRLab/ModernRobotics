# Modern Robotics:  Mechanics, Planning, and Control
# Code Library

This repository contains the code library accompanying [_Modern Robotics: 
Mechanics, Planning, and Control_](http://modernrobotics.org) (Kevin Lynch 
and Frank Park, Cambridge University Press 2017). The 
[user manual](/doc/MRlib.pdf) is in the doc directory.

The functions are available in:

* Python
* MATLAB
* Mathematica

Each function has a commented section above it explaining the inputs required for its use as well as an example of how it can be used and what the output will be. This repository also contains a pdf document that provides an overview of the available functions using MATLAB syntax. Functions are organized according to the chapter in which they are introduced in the book. Basic functions, such as functions to calculate the magnitude of a vector, normalize a vector, test if the value is near zero, and perform matrix operations such as multiplication and inverses, are not documented here.

The primary purpose of the provided software is to be easy to read and educational, reinforcing the concepts in the book. The code is optimized neither for efficiency nor robustness.

Some unofficial versions in other languages are being developed:
* [C++ version](https://github.com/Le0nX/ModernRoboticsCpp)
* [Julia version](https://github.com/ferrolho/ModernRoboticsBook.jl)
* [Nim version](https://github.com/Nimbotics/ModernRoboticsNim)

Some libraries built on ours:
* [KinematicsFromDescriptionTool](https://github.com/Interbotix/kinematics_from_description), which calculates the kinematics input parameters from a robot's URDF or robot_description parameter using ROS and Python3.
* [tf_rbdl](https://github.com/junhyeokahn/tf_rbdl#tf_rbdl), which refactors the Python version using the package `tensorflow`.

Any contribution is welcomed but the maintenance team for this library here doesn't vouch for the reliability of those projects.
