# Digital-Twin of a Traffic Scene Using RSU, AWSIM, And Autoware
B. Sc. Thesis concerning the semi-automatic creation of a digital twin of a traffic scene near Amirkabir University of Technology

Project Proposal Accepted at 5/11/2023

## Introduction: 
In the rapidly evolving landscape of Intelligent Transportation Systems (ITS), traffic management, and autonomous vehicles, realistic simulation environments are pivotal for testing and advancing cutting-edge technologies. This thesis introduces a practical approach on creating a Digital Twin of a Traffic Scene, enabling high-fidelity simulations of real-world traffic scenarios.

### Key Components:
- **Ouster Lidar Data Integration:** Real-time conversion of Ouster Lidar `.pcap` data to `PointCloud2` ROS2 topics.
- **Dynamic Object Detection:** Detection of 3D objects in real-time using **Autoware's Perception Module** with self-made launchers specifically designed to accept any kind of Lidar PointCloud2 data.
- **AWSIM Integration:** Seamless integration of tracked objects into AWSIM, a Unity-based simulator.

### Project Goals:

- **Realism:** Accurate modeling of real-world traffic conditions captured by sensor data.
- **Automation:** An automatic methodology for real-time data transformation and object integration.
- **Validation:** Utilization of innovative tools for effective validation and testing.

### Significance:

- Enhances the testing of ITS and traffic management algorithms in a dynamic and realistic virtual environment.
- Provides a valuable platform for autonomous vehicle development, training, and adaptation to region-specific traffic scenarios.
- Eliminates the need for synthesized traffic conditions and labor-intensive simulation setups.

This project presents an opportunity to explore the realms of digital twinning, revolutionizing how we simulate and analyze traffic scenarios for the benefit of future transportation systems and autonomous vehicles.

## Notes

* I currently have a lot on my plate and can't manage to document this project. But once I defend this thesis, I'll document everything!
* **Still waiting for Professor Javanmardi's approval on when to upload the source code. For now I'll be uploading the project runtimes.**   
* [Checkout my other repository where I'm writing the simulation code for AWSIM.](https://github.com/rzninvo/AWSIM)

## **(LATEST UPDATE)** The real time simulation (Semi Digital Twin) for Amirkabir University of Technology Rasht Street (GIF):

![RASHT_DT_DEMO](./resource/Digital_Twin.gif)

## The real time simulation (Semi Digital Twin) for an intersection demo:

https://github.com/rzninvo/Digital-Twin-of-a-Traffic-Scene-Using-RSU-and-AWSIM/assets/46872428/61aba3b1-a6cb-4f1a-9d2e-9236baa5d7ad

## The real time detection for an intersection demo:
https://github.com/rzninvo/Digital-Twin-of-a-Traffic-Scene-Using-RSU-and-AWSIM/assets/46872428/2549ab65-fce6-48c2-88c2-b1d3b224020b

## The real time detection for a driving car demo:
https://github.com/rzninvo/Digital-Twin-of-a-Traffic-Scene-Using-RSU-and-AWSIM/assets/46872428/0ee02405-9e35-4a63-bf8c-4ef97401d949



## The AWSIM Demo After Configurating Everything
https://github.com/rzninvo/Digital-Twin-of-a-Traffic-Scene-Using-RSU-and-AWSIM/assets/46872428/f5b05a3e-4cc1-45c5-b072-100a61d440fc

## Project Progress and Timechart:
* **Step 1** : Getting Lidar input data from our Computer Engineering faculty as a demo and attempting to create it's 3D model.   
HALF DONE!
Currently requesting my university to give us a license to record a one-hour lidar recording. The 3D modeling comes after!

* **Step 2** : Finishing the Robotics Course and learning ROS.   
DONE!

* **Step 3** : Setting up AWSIM and it's required libraries.   
DONE!   

* **Step 4** : Learning ROS2 Humble and the architecture behind Autoware and AWSIM.  
DONE!

* **Step 5**  : Learning PCL and converting pcap recordings to PointCloud2 ROS2 Topics and publishing it.  
DONE!

* **Step 6** : Implementing the real-time 3D detection and tracking using CenterPoint.  
DONE!

* **Step 7** : Learning Unity  
DONE!

* **Step 8** : [Added the `autoware_auto_perception_msgs` to the Ros2ForUnity Plugin in AWSIM.](https://github.com/rzninvo/AWSIM/commit/aeeadf17f201f0bec529d97b834286d8ddc114c2)  
DONE!

* **Step 9** : Creating a new sample scene in which any Tracked Object in Autoware with the confidence of 70+, Spawns a Vehicle.  
DONE!

* **Step 10** : Making a SLAM map based on the recorded pcap file and creating a Lanelet2 Map using the Tier4 website.
