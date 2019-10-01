**Welcome to the Particle Filter Detector System**

This repository is a ROS node that is aimed at tracking particle cloud feeding the system with previous information from existing exteroceptive sensors. This is a free and open source package that proved to give encoraging result in the underwater environment. However it will be utilized for  


It has the following dependencies:
1. Qt5 
2. ROS
3. Bayesian Library (the orocos-bfl project)

ABSTRACT - UNDERWATER APPLICATION
The Particle Filter Detector Systsem (PFDS) is a system used for underwater imaging and fuild environemnt. 
Originally developed from a terrain-aided particle filter for localizing a freely drifting  underwater  vehicle, the PFDS can also be extended to blood analysis and blood cell tracking.
If used for underwater on a remotely operated vehicle (ROV) it can be used for habitat  classification,  monitoring tracking of estimated position. During  operation  the vehicle captures down looking images at a controlled altitude above the bottom. Direct navigation information is often, but not always, recorded with a ultra short
baseline  (USBL)  acoustic  systsem. The  presented  methodology  provides  an  al-
ternate method for georeferencing when USBL is unavailable.  The implemented
particle filter utilizes a background bathymetry map and visual odometry as a motion mode.  The particle filter is implemented using the Robot Operating System (ROS) and Orocos Bayesian Filtering Library (BFL). The Grid Map package is used to store and retrieve the bathymetryic data.  Results using data collected shows how to effectively utilize the terrain information and produce drift trajectories which closely match the recorded USBL data. Utilizing the method allows the system to be deployed with minimal ship-side support while still maintaining the georeferencing critical to the end use of the collected images. But most importantly, because of the broad scope of application this system can be applied in several environment.


ABSTRACT - BIOMEDICAL APPLICATION
The PFDS can be used in a multiple ways. A great use of this package will be in the tracking of specific blood cells particles for estimating the position of a: i) cell, ii) cancer cell based on specific cancer markers, iii) fluid flow inside blood vessel. A specific number of exteroceptive sensors will be mounted on the human body such as:
1. A pressure sensor on the body detecting the pressure (and temperature)
2. Blood speed sensor
3. Inertial Measurment Unit system that give rotation of the human body part under analysis
4. Location sensor (e.g. GPS system) to localize the reference frame
5. an a-priori known map from an ultra-sound device that gives the internal part composition of the body
6. Laser device to scan the human body
7. Camera system for stereo-pair use and 3D composition of images

All this information combined together will be able to provide the exact location of the specific cell someone is looking for. ROS will be used for visualization purposes.



