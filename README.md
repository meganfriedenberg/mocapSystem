# mocapSystem
Motion capture system in OpenGL using quaternions and different interpolation schemes. Project for CSCI 520.


- [Skeleton class](https://github.com/meganfriedenberg/mocapSystem/blob/main/mocapSolver/skeleton.cpp) parses the ASF file to construct a skeleton hierarchy.

- [Motion class](https://github.com/meganfriedenberg/mocapSystem/blob/main/mocapSolver/motion.cpp) parses the AMC file to store joint angles for every frame into an array of postures. Each posture defines the root position/orientation for all bones at one particular frame. 

- [Display Skeleton class](https://github.com/meganfriedenberg/mocapSystem/blob/main/mocapSolver/displaySkeleton.cpp) Renders the skeleton in OpenGL for a given posture

- [Interpolator class](https://github.com/meganfriedenberg/mocapSystem/blob/main/mocapSolver/interpolator.cpp) Supports several different interpolation types: Linear Euler, Bezier Euler, Slerp Quaternion, Bezier Slerp Quaternion, with quaternion interpolation using De Casteljau's algorithm for Bezier splines.


![GIF of Mocap system playing for dancing.](https://github.com/meganfriedenberg/meganfriedenberg.github.io/blob/master/images/mocap.gif?raw=true)
