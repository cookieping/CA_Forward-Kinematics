# CA_Forward-Kinematics
Homework for Computer Animation and Effects of NCTU
### This project implements Forward Kinematics and Time Warping.
Its framework is developed with C++ and OpenGL.

Forward Kinematics refers to the use of the kinematic equations of a robot to compute the position of the end-effort from specified values for the joint parameters.
It is frequently used in animation, computer game, and robotics.
One of the difference between forward kinematics and inverse kinematics is that forward kinematics maps from joint space to cartesian space, 
and inverse kinematics maps from cartesian space to joint space.

Time warping is applied in the field of computer animation. It can be used to estimate a specific frame with the keyframe.
According to the keyframe, we can interpolate the result we want.

In this project, I implement two functions- forwardSolver(posture, bone) and timeWarper(vector of posture, old keyframe, newkeyframe). 
The former converts data from joint space to the Cartersian space, and the latter modifies the given motion sequences with arbitary profile.
