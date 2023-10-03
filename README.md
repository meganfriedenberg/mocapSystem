# mocapSystem
Motion capture system in OpenGL using quaternions and different interpolation schemes. Project for CSCI 520.


- [Skeleton class](https://github.com/meganfriedenberg/mocapSystem/blob/main/mocapSolver/skeleton.cpp) parses the ASF file to construct a skeleton hierarchy.

- [Motion class](https://github.com/meganfriedenberg/mocapSystem/blob/main/mocapSolver/motion.cpp) parses the AMC file to store joint angles for every frame into an array of postures. Each posture defines the root position/orientation for all bones at one particular frame. 

- [Display Skeleton class](https://github.com/meganfriedenberg/mocapSystem/blob/main/mocapSolver/displaySkeleton.cpp) Renders the skeleton in OpenGL for a given posture

- [Interpolator class](https://github.com/meganfriedenberg/mocapSystem/blob/main/mocapSolver/interpolator.cpp) Supports several different interpolation types: Linear Euler, Bezier Euler, Slerp Quaternion, Bezier Slerp Quaternion, with quaternion interpolation using De Casteljau's algorithm for Bezier splines.


![GIF of Mocap system playing for dancing.](https://github.com/[meganfriedenberg]/[meganfriedenberg.github.io]/blob/[master]/images/mocap.gif?raw=true)   
Green reflects my interpolation scheme which is Bezier Slerp Quaternion here, against the red ground truth.

For better ease in understanding the aforemationed math equations used in [Interpolator](https://github.com/meganfriedenberg/mocapSystem/blob/main/mocapSolver/interpolator.cpp), take note of the following two diagrams.   
      

![Diagram showing De Casteljau's algorithm on a Bezier spline.](https://github.com/meganfriedenberg/mocapSystem/blob/main/images/DeCasteljau.JPG?raw=true)  


- Q0 = SLERP(P0, P1, t)
- Q1 = SLERP(P1, P2, t)
- Q2 = SLERP(P2, P3, t)
- R0 = SLERP(Q0, Q1, t)
- R1 = SLERP(Q1, Q2, t)
Therefore, P(t) = SLERP(R0, R1, t). The quaternion interpolation schemas heavily use the calculation of P(t).   
    

![Diagram showing how to use quaternions on a Bezier spline.](https://github.com/meganfriedenberg/mocapSystem/blob/main/images/Bezier.JPG?raw=true)    
    
Given quaternions Qn-1, Qn, and Qn+1:
aBarN = SLERP(SLERP(Qn-1, Qn, 2.0), Qn+1, 0.5)
aN = SLERP(Qn, aBarN, 1.0/5)
bN = SLERP(Qn, aBarN, -1.0/3)  
To relate this back, Qn-1 is the previous posture, Qn is the current posture, and Qn+1 is the next posture. De Casteljau's interpolation functions in the same way.

