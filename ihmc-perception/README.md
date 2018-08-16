# Stereo-based Visual SLAM using RTAB-Map

## About

A Docker environment has already been created with ROS Kinetic with all the dependencies to launch the RTAB-Map for Stereo-based SLAM.
The instructions below 

## Dependencies

Ubuntu 14.04 or above  
Docker  
NVIDIA Driver

## Instructions to setup the environment

* Install Docker on the Workstation if it is not already installed.
* Clone "IHMC Open Robotics Software" onto the Workstation and checkout the branch "feature/rtab-slam"
* Clone "IHMC Multisense Tools" onto the workstation and checkout the branch "feature/rtab-slam"
* Change directory into "Kinetic3D" inside "ihmc-open-robotics-software/ihmc-perception"
* Use command `nvidia-smi` to get the driver version number for the NVIDIA Graphics Card installed on the workstation
* Download this exact NVIDIA driver version form the NVIDIA website into the Kinetic3D directory.
* Run the command `sudo bash docker.sh build` to build the `kinetic3d:latest` Docker Image 
* Execute the command `xhost +` on the Ubuntu host machine CLI
* Use the command `sudo bash docker.sh run` to launch the Docker container from the `kinetic3d:latest` image
* Once inside the running container, use the command `bash install-rtabmap.sh` to setup the remaining dependencies for NVIDIA driver installation
* Accept and select "Yes" for any X-Server prompts during the process of installation.
* After all the dependencies and drivers have been installed successfully, run the command `source /opt/ros/kinetic/setup.bash` to setup the ROS Kinetic environment.
* Modify the Environment Variables $ROS_MASTER_URI and $ROS_IP according to the network configurations of ROS Master and this Workstation. Otherwise the algorithm will not work.
* Once the MultiSense driver is publishing the left and right camera image and info topics, launch the `stereo_mapping.launch` that exists in inside the Kinetic3D directory, using Roslaunch.

## Instructions to setup the remaining Data Flow

* The following files enable the rest of the data flow:
	* Kinetic3D/filter_chain.launch (Subscribe: /voxel_cloud, Publish: /filter_chain/output)
	* Kinetic3D/voxel_cloud.launch (Subscribe: /filter_chain/output, Publish: )
	* IHMC Perception: MultisensePointCloud2WithSourceReceiver (Handled internally on Valkyrie)
	* IHMC Perception: LidarBasedREAStandaloneLauncher (Robot Awareness Environment generates the planar regions) [Subscribe: ihmc/lidar_scan]

Refer to the diagram below to setup the remaining part of the pipeline to enable the REA to generate Planar Regions

## Data Flow Diagram for the RTAB-Map Visual SLAM Integration with Robot Environment Awareness (REA)  

![RTAB-Map Data Flow Diagram](RTABMap_Data_Flow.png)  

## Final Result Video  

To see the final results check out the YouTube video: https://www.youtube.com/watch?v=fF9qEaRX50I  

## More Documentation for RTAB-Map

Please look at the following two link. There is an amazing level of documentation in them.  
Github Wiki: https://github.com/introlab/rtabmap/wiki  
Documentation: http://introlab.github.io/rtabmap/  


### Author

Bhavyansh Mishra  
