# Functionality Description

Alpha version of README, more scripts and functionality to be added.

## 1. Teleoperation and Driving

* For driving the rover, launch the world in Gazebo and run the following node:

       rosrun lsd key_drive.py
       
### Voilà! 
  
<p align="center">
    <img src = "https://user-images.githubusercontent.com/45683974/95751043-a31df800-0cbb-11eb-9a51-8f617ae7436e.gif" width="900" height="450">
</p>

## 2. Holding Torque

* For now a certain preset has been chosen, which is bound to vary:

      rosrun lsd holding_torque.py
      
## 3. Reading the link angles of the motors:

* Here, a template script has been created, that extracts the pitch angles of all the 4 motors from the `/gazebo/link_states` topic:

      rosrun lsd link_angles.py
