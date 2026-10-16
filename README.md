# Gazebo Simulation (Gazebo Fortress)

This contains all the files that are used for running the Gazebo simulation of the rover's arm alone. 

## URDF
There are currently three main URDFs of the rover's arm:
* __rover_arm_raw.xacro__: This is the playground, so to speak, while making changes to the URDF. This the raw export from LinkForge (more on that below).
* __rover_arm_sim.xacro__: This is the xacro file used to in the Gazebo simulation with its custom gazebo plugins.
* __rover_arm_real.xacro__: This is the xacro file that will be used on the actual rover, in case we ever need to use any ROS 2 packages that require a URDF to function.

The visuals for all these xacro files are stored in the ```meshes``` folder within this directory and these are also where the default export location of LinkForge should be.


## Running the simulation
To run the simulation, first clone the repository and checkout to the required branch ```gazebo_main``` or ```gazebo_dev```. There are two launch files in the launch directory of the ROS 2 package ```rover_arm_description```:
* __gazebo.launch.py__: This launch file launches gazebo and rviz2. The camera feeds can be visualized via rviz2 as well as through other custom ROS2 nodes.
* __rviz.launch.py__: This launch file launches only rviz2 and the joint_state_publisher_gui in order to verify changes made to the URDF.

## Editing
To edit the URDF of the rover arm or to add new features such as sensors, it must be done using LinkForge, which is a blender plugin. 

After LinkForge is installed, follow the steps below to make your own URDFs or edit the current one. 
* __Step 1__: Pester some mech person to give you whatever parts step file. 
* __Step 2__: Open this step file in Fusion 360 and rearrange all the components and bodies, such that all that remain in the tree structure are the actual links connected via revolute joints.
* __Step 3__: Export these as __binary__ STL files.
* __Step 4__: Open blender and import the STL files one by one, while arranging them as the joints should be. Use the 3D cursor to correct the object origins where you want them to be (Ask gemini for steps). 
* __Step 5__: Click ```N``` to open the sidebar in blender, where you can access LinkForge. Click on one of the links and click ```Create Link from Mesh```, which will make the link. Once it is a link, you can also create the collision, which should usually be kept as the bounding box option.
* __Step 6__: To create joints, click on the __child__ link and scroll down in the LinkForge side bar to see the ```Create Joint``` option, which will allow you to add the joint. 
* __Step 7__: To create a sensor, click on whichever link you want the sensor to be at and click ```Create sensor```, this will immediately add the require Gazebo tags in your URDF, though you may have to make further corrections, as this feature assumes a wrong coordinate frame.

