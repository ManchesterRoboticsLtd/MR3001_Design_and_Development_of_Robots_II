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
# MR3001B Design and Development of Robots I

  ## Introduction
* The objective of this course, created by Manchester Robotics Ltd. (MCR2), is to equip students with the skills needed to operate mobile robots in different environments.

* This course is divided into two sections, carefully designed for the user to learn about the different aspects of ROS, from topics and messages to control and simulation of a real robot. These sections will span throughout a five-week period, with two lessons per week.

* The first part of this course introduces the basic concepts and general knowledge of the ROS environment to the user.
The second part of the course is dedicated to the analysis, control, and simulation of mobile robots in different environments. 

* This course will be based on challenges to make the student aware of the problems faced while implementing advanced intelligent algorithms in robotics.
   
## General Information
* MCR2 Person in Charge: Dr Alexandru Stancu and Dr Mario Martinez
* Tecnológico de Monterrey Person in Charge: Dr.  Consuelo Rodríguez Padilla and Dr. Carlos Vazquez
* Duration: 5 Weeks
*	Student counselling: Via appointment.
*	Classes: 10 sessions, 1 group.
    *	Group 1: Tuesday, Wednesday and Friday, 9- 11 AM (Central Mexico Time)
* Start Date: 12 Febraury 2023

## Live Sessions (Recordings)
  * TBD
     
## General Requirements
General requirements. Please be aware that a set of requirements specific to each session will be shown in each session subsection (Some items may be repeated).
* Computer with access to Zoom (online classes).
* Computer with Ubuntu 20.04 and ROS Noetic or the Virtual Machine provided by MCR2.
* Knowledge of Windows. 
* Basic knowledge of Ubuntu (recommended).
* Basic understanding of robotics (recommended).
* Basic understanding of control (recommended).

## Weekly Sessions

### Week 1: ROS Introduction
  This session will introduce the teaching team and the basics of ROS.

#### Session 1:
 * Who are we? 
 * Introduction to robotics.
 * Introduction to VM/Ubuntu

#### Session 2:
  *	Introduction to ROS
  *	Overview of ROS Environment
    *	Topics, Messages

 #### Session 3:
  *	Introduction to ROS
  *	Overview of ROS Environment
    *	Topics, Messages


  #### Activities

   * ##### Activity 1: Talker and Listener
     Generate a node that sends a message to another node to listen to it.

   * ##### Activity 2: Launch Files
     Create a Launch file for the nodes created.

  #### Mini-Challenges

  * ##### Mini-Challenge 1: ROS Nodes
    Generate a node that sends a signal to another node to process it.
  
  **Requirements:** 
 * Computer with access to Zoom
 * Ubuntu 20.04
 * ROS Noetic Installed (Full installation).
 * If Ubuntu 20.04 cannot be installed, MCR2 offers a Virtual Machine with ROS preinstalled (installation instructions in Week 1 Folder).


### Week 2: ROS Practicalities
  This week, we will be delivering a comprehensive introduction to some of the most practical and relevant features of the Robot Operating System (ROS). 

#### Session 1:
 * Solution of Mini-challenge 1.
 * ROS Namespaces
 * ROS Parameter Server

#### Session 2:
  *	RVIZ Introduction
  * Transformations

 #### Session 3:
  *	RVIZ Introduction
  * Transformations

  #### Activities

   * ##### Activity 1: Namespaces
     Use namespaces to create two node groups

   * ##### Activity 2: Parameters
     Parametrise previous nodes.

   * ##### Activity 3: Parameter Files
     Use parameter files to parametrise previous nodes.

   * ##### Activity 4: Marker
     Create a simple marker in RVIZ.

   * ##### Activity 5.1: Static Transform
     Create a static transform for RVIZ

   * ##### Activity 5.2: TF activity
     Create dynamic transforms in RVIZ.

   * ##### Activity 6: Planets
     Create multiple movable markers in RVIZ.

   * ##### Activity 7: TF Listener
     Listen to a transform in RVIZ.


  #### Mini-Challenges

  * ##### Mini-Challenge 2: Mobile robot
    Show a simple mobile robot in RVIZ using markers and TF.  
  
  **Requirements:** 
    Requirements of Session 1.


### Week 3: URDF Files
This week, we will introduce the concept of URDF files in the context of Robot Operating System (ROS). This will involve an in-depth exploration of these files' underlying architecture and functionality, focusing on their role in facilitating the creation and deployment of robotic systems.

#### Session 1:
 * Solution of Mini-challenge 2.
 * URDF Introduction
 * Joints
 * Robot State Publisher
 * Joint State Publisher


#### Session 2:
  * Joint State publisher
  * Links


 #### Session 3:
  *	Joint State publisher
  * Links


  #### Activities

   * ##### Activity 1: Simple URDF file.
     Create a simple URDF.

   * ##### Activity 2.1: Create a movable joint.
     Create a simple movable joint

   * ##### Activity 2.2: Create a prismatic joint.
     Create a simple prismatic joint.

   * ##### Activity 2.3: Create a revolute joint.
     Create a simple revolute joint.

   * ##### Activity 3.1: Making a Joint State Publisher.
     Create a simple joint stat publisher.

   * ##### Activity 3.2: Making a Joint State Publisher for a prismatic joint.
     Create a simple joint state publisher for a prismatic joint.

   * ##### Activity 3.3: Making a Joint State Publisher for a revolute joint.
     Create a simple joint state publisher for a revolute joint.

   * ##### Activity 4.1: Making a pneumatic cylinder model.
     Make a simple pneumatic cylinder

   * ##### Activity 4.2: Making a servo motor model.
     Make a simple servo motor model.


  #### Mini-Challenges

  * ##### Mini-Challenge 3: DC Motor
    Show a DC motor dynamical behaviour using a first-order system node in RVIZ.  
  
  **Requirements:** 
    Requirements of Session 1.


### Week 4: Mobile Robots Simulation.
This week's curriculum will encompass the core concepts of mobile robots and simulation.

#### Session 1:
 * Solution of Mini-challenge 3.
 * Simulation and Modelling Basics.
 * Dynamical Systems


#### Session 2:
  * Dynamical Systems
  * Differential Drive Basics.


 #### Session 3:
  * Differential Drive Basics.
  * Kinematic Model of a DDR.
  * Gazebo simulation of a DDR.



  #### Activities

   * ##### Activity 1: Motor Simulation.
     Simulate the dynamics of a Motor.


  #### Mini-Challenges

  * ##### Mini-Challenge 4: DDR Kinematic Simulator
    Develop a node to perform a kinematic simulation of a DDR.  
  
  **Requirements:** 
    Requirements of Session 1.


### Week 5: Open Loop/Closed Loop Control.
This week will introduce some basics of open-loop and closed-loop control for mobile robotics.

#### Session 1:
 * Open loop control theory.
 * Differential Drive Robot Open-Loop Control.


#### Session 2:
  * Open loop control theory.
  * Differential Drive Robot Open-Loop Control.
  * Closed loop control theory.


 #### Session 3:
  * Closed loop control theory.
  * Differential Drive Robot Closed-Loop Control for path following.
  * Presentation of the Final Challenge


  #### Activities

   * ##### Activity 1: Teleoperation of a mobile robot.
     Simple DDR control.

   * ##### Activity 2: Open Loop control for a DDR.
     Open Loop Path following of a real Mobile Robot

   * ##### Activity 3: Teleoperation of a mobile robot.
     Simple closed-loop control

  #### Mini-Challenges

  * ##### Mini-Challenge 5: Closed loop control of a mobile robot
    Closed Loop control for Path following a Mobile Robot.  
  
  **Requirements:** 
    Requirements of Session 1.


  ---
## Declaration

At Manchester Robotics, we firmly believe that innovation is driven by change, so we have made it our mission to change access to educational robotics. We hope you enjoy our products and support this revolution.

So, from the team at MCR2, we would like to say 

                                                          Thank you!
                                                   {Learn, Create, Innovate};
---
  ## Disclaimer
 *THE PIECES, IMAGES, VIDEOS, DOCUMENTATION, ETC. SHOWN HERE ARE FOR INFORMATIVE PURPOSES ONLY. THE DESIGN IS PROPRIETARY AND CONFIDENTIAL TO MANCHESTER ROBOTICS LTD. (MCR2). THE INFORMATION, CODE, SIMULATORS, DRAWINGS, VIDEOS PRESENTATIONS ETC. CONTAINED IN THIS REPOSITORY IS THE SOLE PROPERTY OF MANCHESTER ROBOTICS LTD. ANY REPRODUCTION OR USAGE IN PART OR AS A WHOLE WITHOUT THE WRITTEN PERMISSION OF MANCHESTER ROBOTICS LTD. IS STRICTLY PROHIBITED*

*THIS WEBSITE MAY CONTAIN LINKS TO OTHER WEBSITES OR CONTENT BELONGING TO OR ORIGINATING FROM THIRD PARTIES OR LINKS TO WEBSITES AND FEATURES IN BANNERS OR OTHER ADVERTISING. SUCH EXTERNAL LINKS ARE NOT INVESTIGATED, MONITORED, OR CHECKED FOR ACCURACY, ADEQUACY, VALIDITY, RELIABILITY, AVAILABILITY OR COMPLETENESS BY US.*

*WE DO NOT WARRANT, ENDORSE, GUARANTEE, OR ASSUME RESPONSIBILITY FOR THE ACCURACY OR RELIABILITY OF ANY INFORMATION OFFERED BY THIRD-PARTY WEBSITES LINKED THROUGH THE SITE OR ANY WEBSITE OR FEATURE LINKED IN ANY BANNER OR OTHER ADVERTISING.*
