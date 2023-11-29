# Local Enviroment

<br>

<!-- > [!REGISTER]
> Register from here!
> [https://www.jsae.or.jp/jaaic/en/index.php](https://www.jsae.or.jp/jaaic/en/index.php)

<br> -->

 &emsp; Participants will be asked to create a ROS2 package to carry out the scenario, and the following ROS2 package is provided in aichallenge2023-racing/docker/aichallenge/aichallenge_ws/src as sample code to serve as a base for this in this repository The following ROS2 package is provided.  
 &emsp; aichallenge_launch, sim_msgs change is not applied at submission

## Sample Code
 &emsp;aichallenge2023-racing/docker/aichallenge/aichallenge_ws/src directory structure is below
* aichallenge_launch
  * Contains the main launch file aichallenge.launch.xml. All ROS2 nodes are launched from this launch file.
* aichallenge_submit
  * All ROS2 packages implemented by participants should be placed in this directory, as only the contents of this directory will be submitted at the time of submission.
  * Please modify this launch file accordingly to configure it to launch the ROS2 node you have implemented. Please modify this launch file accordingly to configure your ROS2 node to be launched.
  * autoware_launch, autoware_universe_launch
    * The Autoware packages included here have been removed from the Docker image of Autoware. You can modify the behavior of Autoware by editing here.
    * If you want to use the files before the modification, please copy the files from [autoware_launch](https://github.com/autowarefoundation/autoware_launch/tree/awsim-stable), [autoware_universe's launch directory](https://github.com/autowarefoundation/autoware.universe/tree/awsim-stable/launch).
  * autoware_micro Folder for creating a minimum configuration of autoware.
  * dallara_interface A set of packages to adapt the vehicle used in Autonoma.
  * sample_pakcages Sample packages for creating custom packages.
### Steps for Execution

1. Docker Image Build
```
#In the aichallenge2023-racing directory
cd docker/train
bash build_docker.sh
```

2. Docker Container Run
```
#In the aichallenge2023-racing directory
cd docker/train
bash run_container.sh
```

3. Code Build
```
# In the Rocker container
cd /aichallenge
bash build_autoware.sh
 ```
 4. Start AWSIM  
Start AWSIM by referring to [Setup page](../setup/index.html).

5. Sample Code Run
 ```
# In the Rocker container
cd /aichallenge
bash run.sh
```
 &emsp; If setup is successful, rviz will display a point cloud map and begin automatic operation.
 
 ### Customizing Autoware

 Ways in which Autoware can be customized, or new packages be added into Autoware, are explained in the [Customizing Autoware](../customize/index.html) page.
