## 4-DOF-2-link-arm-motion-planning
This repository contains MATLAB code for implementing the Probabilistic Roadmap (PRM) and Rapidly Exploring Random Tree (RRT) algorithms for motion planning of a 4-DOF 2-link arm in the presence of a spherical obstacle. The project provides visualization of the robot in various configurations, checks for joint limits and collisions, generates uniformly distributed samples within joint limits, constructs a PRM roadmap, finds a collision-free path using the PRM algorithm, implements the RRT algorithm to find another collision-free path, and applies path smoothing techniques to remove unnecessary waypoints. 


Generating random joint angle samples from workspace:

https://github.com/josejosepht/4-DOF-2-link-arm-motion-planning/assets/97187460/33841030-cefa-4c18-a1d0-319df9bfb964


Path generated from PRM algorithm:

https://github.com/josejosepht/4-DOF-2-link-arm-motion-planning/assets/97187460/577ef532-b089-4731-b18a-da865e91d12c

Path generated from RRT algorithm:

https://github.com/josejosepht/4-DOF-2-link-arm-motion-planning/assets/97187460/bc2c63ad-b89b-4e9b-929e-a65d9395b2e1

Smoothened out RRT algorithm path:

https://github.com/josejosepht/4-DOF-2-link-arm-motion-planning/assets/97187460/ec05b102-fc1a-459d-b96a-d267417382e4

