<picture>
  <source media="(prefers-color-scheme: dark)" srcset="https://github.com/ManchesterRoboticsLtd/MR3001B_Design_and_Development_of_Robots_I/blob/main/Misc/Logos/Logotipo%20Vertical%20Bco_Transparente.png">
  <source media="(prefers-color-scheme: light)" srcset="https://github.com/ManchesterRoboticsLtd/MR3001B_Design_and_Development_of_Robots_I/blob/main/Misc/Logos/Logotipo%20Vertical%20Azul%20transparente.png">
  <img alt="Shows ITESM logo in black or white." width="160" align="right">
</picture>

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="https://github.com/ManchesterRoboticsLtd/MR3001B_Design_and_Development_of_Robots_I/blob/main/Misc/Logos/MCR2_Logo_White.png">
  <source media="(prefers-color-scheme: light)" srcset="https://github.com/ManchesterRoboticsLtd/MR3001B_Design_and_Development_of_Robots_I/blob/main/Misc/Logos/MCR2_Logo_Black.png">
  <img alt="Shows MCR2 logo in black or white." width="150" align="right">
</picture>

---
# MR3001C Design and Development of Robots II

  ## Session
  * This session intends for the students to learn about the difference between simulator environments and state-space theory that will be used in the next sessions and develop a simple dynamical simulation using the knowledge of the previous sessions.

### << This session contains some "follow-me" activities; we encourage the students to bring their laptop with Ubuntu and ROS (Noetic) installed or hte MCR2 VM during the session >>

  ## Session
  * Dynamical simulators
  * State Space Theory
  * Robot Dynamics

  ## Mini Challenge
  * Develop a Dynamic simulator for a DC motor using URDF Files.
  * The mini challenge and further instructions for this session are located in the folder Challenge. 

  
  Please note: This repository contains all the necessary files and presentations given during this session and the instructions for each one of the activities of the session.

---

## YouTube Video
  In this video, Rebeca shows us some of the critical concepts of ROS. This video is for reference only. The activities shown are not required to be performed. Some concepts in this video will be seen in session 2.

<a href="http://www.youtube.com/watch?feature=player_embedded&v=IF-k6KhhRkE
" target="_blank"><p align="center"><img src="http://img.youtube.com/vi/IF-k6KhhRkE/0.jpg" 
alt="ROS Basics" width="300" border="10"/></p></a> 

<div align="center"> https://www.youtube.com/watch?v=IF-k6KhhRkE&ab_channel=ManchesterRobotics </div>

---

## Ubuntu installation for MAC
 * If the computer has an Intel processor, it is possible to use  VMWare Fusion Player V12 onwards.
 * If the computer has an M! or M2 processor, the hypervisor UTM to execute the ubuntu server 22.04 ARM, this version has no GUI, so Ubuntu Desktop must be installed manually
    (sudo apt install ubuntu-desktop)
   Use the following links:
   
   [Link 1](https://www.youtube.com/watch?v=O19mv1pe76M)
   
   [Link 2](https://www.youtube.com/watch?v=-XFNUeWhsIQ)
   
 * Another way for M1/M2 processors is to use Parallels Desktop (Paid).

---

### Useful Links: 

## YouTube Videos
  * [Dynamical Systems](https://www.youtube.com/watch?v=9RG-AuUSuhM&list=PLqCuMQTwnIP99CrzdPEroGhdAhzVfvWgR&index=2&ab_channel=ManchesterRobotics)
  * [Pendulum System](https://www.youtube.com/watch?v=NjcRbP_L_-Y&list=PLqCuMQTwnIP99CrzdPEroGhdAhzVfvWgR&index=3&ab_channel=ManchesterRobotics)
  * [Pendulum Phase Portrait](https://www.youtube.com/watch?v=xi1vUEH9OO8&list=PLqCuMQTwnIP99CrzdPEroGhdAhzVfvWgR&index=4&ab_channel=ManchesterRobotics)
  * [Pendulum Phase Portrait 3D](https://www.youtube.com/watch?v=EKiWPS61nqo&list=PLqCuMQTwnIP99CrzdPEroGhdAhzVfvWgR&index=5&ab_channel=ManchesterRobotics)
  * [Pendulum State Space](https://www.youtube.com/watch?v=eZfnn9VGsqY&list=PLqCuMQTwnIP99CrzdPEroGhdAhzVfvWgR&index=6&ab_channel=ManchesterRobotics)
  * [Pendulum Phase Portrait](https://www.youtube.com/watch?v=mTH1MSlv97c&list=PLqCuMQTwnIP99CrzdPEroGhdAhzVfvWgR&index=7&ab_channel=ManchesterRobotics)
  * [Van der Pol](https://www.youtube.com/watch?v=78Cqfejpb3U&list=PLqCuMQTwnIP99CrzdPEroGhdAhzVfvWgR&index=8&ab_channel=ManchesterRobotics)
  * [Lorenz Attractor](https://www.youtube.com/watch?v=lSlIbP3YYyM&list=PLqCuMQTwnIP99CrzdPEroGhdAhzVfvWgR&index=9&ab_channel=ManchesterRobotics)
  * [Linear systems](https://www.youtube.com/watch?v=XuR6oNYjG10&list=PLqCuMQTwnIP99CrzdPEroGhdAhzVfvWgR&index=10&ab_channel=ManchesterRobotics)
---

#### Dynamical Systems
  * [Nonlinear Systems](https://books.google.fr/books/about/Nonlinear_Systems.html?id=t_d1QgAACAAJ&redir_esc=y)
  * [Nonlinear & Adaptive Systems](https://digital-library.theiet.org/content/books/ce/pbce084e)
  * [Nonlinear Dynamical Systems](https://books.google.fr/books/about/Nonlinear_Dynamical_Systems.html?id=FPlQAAAAMAAJ&redir_esc=y)
  * [Linear Systems Primer](https://wp.kntu.ac.ir/hrahmanei/Adv-Control-Books/A-Linear-Systems-Primer.pdf)
  * [Lectures on Nonlinear Systems](https://web.mit.edu/nsl/www/videos/lectures.html)
  * [Applied Nonlinear Control](https://books.google.fr/books/about/Applied_Nonlinear_Control.html?id=cwpRAAAAMAAJ&redir_esc=y)
  * [Dynamic Model of a DC Motor](https://www.ijser.org/researchpaper/Dynamic-Model-Analysis-of-a-DC-Motor-in-MATLAB.pdf)

#### Robotics
* [Dynamic Model of a Differential Drive Robot](https://www.hilarispublisher.com/open-access/dynamic-modelling-of-differentialdrive-mobile-robots-using-lagrange-and-newtoneuler-methodologies-a-unified-framework-2168-9695.1000107.pdf)
* [Kinematic Model of Differential Drive Robot](https://globaljournals.org/GJRE_Volume14/1-Kinematics-Localization-and-Control.pdf)
* [Introduction to Autonomous Mobile Robots](https://ieeexplore.ieee.org/book/6267528)

#### Ubuntu
  * [Ubuntu Installation](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview)

#### ROS
 * [URDF Files](http://wiki.ros.org/urdf)
 * [URDF Tutorials](http://wiki.ros.org/urdf/Tutorials)
 * [URDF Joints](http://wiki.ros.org/urdf/XML/joint)
 * [URDF Robot](http://wiki.ros.org/urdf/XML/robot)
 * [URDF Link](http://wiki.ros.org/urdf/XML/link)
 * [Messages and Services](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv)
 * [ROS Installation](http://wiki.ros.org/noetic/Installation/Ubuntu)
 * [ROS book](https://www.cse.sc.edu/~jokane/agitr/)
 * [ROS Packages](http://wiki.ros.org/ROS/Tutorials/CreatingPackage)
 * [ROS Workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
 * [Publisher and Subscribers](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29)
 * [Roslaunch](http://wiki.ros.org/roslaunch)

#### Virtual Machine (Google Drive): 
  * [VM Ware](https://drive.google.com/file/d/1Kqt8E69nB5pxYzyVztyoxF0UY9yCHLns/view)
  * [ROS Preinstalled VM](https://drive.google.com/file/d/1LCn433uN5pf8dcauWDagKEKjORsE3fZR/view)
 ---
