# Welcome to the ROS Particle Filter Detector System (PFDS)
<div>
&nbsp 
&nbsp 
&nbsp 
<img src="https://user-images.githubusercontent.com/55800613/79771851-602b2b00-82fd-11ea-98ba-a53b54eeb381.png" width="400" height="400">
&nbsp 
&nbsp 
&nbsp 
&nbsp 
&nbsp 
<img src="https://user-images.githubusercontent.com/55800613/79771949-7cc76300-82fd-11ea-9a92-719507dcde5d.png" width="400" height="400">
</div>


This repository is a ROS node that is aimed at tracking particle cloud feeding the system with previous information from existing exteroceptive sensors. This is a free and open source package that proved to give encoraging result in the estimation of the expected position of particles given specific initial conditions. 
It will be utilized mostly for tracking of specific blood cells particles for estimating the position of blood cell, cancer cell based on specific cancer markers, fluid flow inside blood vessel or even specific request from the public and according to the general needs. The particle filter is implemented using the Robot Operating System (ROS) and Orocos Bayesian Filtering Library (BFL).

It has the following dependencies:
1. Qt5 
2. ROS
3. Bayesian Library (the orocos-bfl project)

**ABSTRACT - BIOMEDICAL APPLICATION**
The PFDS can be used in a multiple ways. A great use of this package will be in the tracking of specific blood cells particles for estimating the position of a: i) cell, ii) cancer cell based on specific cancer markers, iii) fluid flow inside blood vessel. A specific number of exteroceptive sensors will be mounted on the human body such as:
1. A pressure sensor on the body detecting the pressure (and temperature)
2. Blood speed sensor
3. Inertial Measurment Unit system that give rotation of the human body part under analysis
4. Location sensor (e.g. GPS system) to localize the reference frame
5. an a-priori known map from an ultra-sound device that gives the internal part composition of the body
6. Laser device to scan the human body
7. Camera system for stereo-pair use and 3D composition of images

All this information combined together will be able to provide the exact location of the specific cell someone is looking for. ROS will be used for visualization purposes. Additional application under development to be added at a second stage is the research of a PHD filter (Probability Hypothesis Density) for multiple tracking purposes. This means that this capability will allow the operator/doctor to track multiple cells and or establish their spread based on a previous patients information loaded. This is surely a great tool for research and test of various data and conducting different simulations.


**ABSTRACT - ULTRASOUND ADDITIONAL APPLICATION**
The Particle Filter Detector Systsem (PFDS) is a system that can also be deployed for underwater imaging and fluidic environemnt. 
If used for underwater it can be used for habitat  classification,  monitoring tracking of estimated position. During  operation the vehicle (most likely an ROV) captures down looking images at a controlled altitude above the bottom. Direct navigation information is often, but not always, recorded with a ultra short baseline (USBL) acoustic systsem. The presented  methodology provides a different method for georeferencing when USBL is unavailable. The implemented
particle filter utilizes a background bathymetry map and visual odometry as a motion mode.  The particle filter is implemented using the Robot Operating System (ROS) and Orocos Bayesian Filtering Library (BFL). The Grid Map package is used to store and retrieve the bathymetryic data.  Results using data collected shows how to effectively utilize the terrain information and produce drift trajectories which closely match the recorded USBL data. Utilizing this approach allows the system to be deployed with minimal ship-side support while still maintaining the georeferencing critical to the end use of the collected images. But most importantly, because of the broad scope of application this system can be applied in several environment.

Under development:
1. Creation of a specific graphical User Interface (GUI) to interact with ROS as an external plug-in
2. Implementation of additional non-parametric filters for additional in-depth analysis
3. Database integration of recorded information during simulation phase

**How PFDS works and its structure**

PFDS relays on the following ancillary software structure:

1) Database Management System
2) Image Lighting Corrector
3) Particle Detection System
4) Rectification Imaging System
5) Ultrasound Processor

Above mentioned softwares are described more in depth in their own wiki repos.
