# __README: Robot Graspit! Project__  
__Carlyn Dougherty (ccd2134@columbia.edu)__


## __FOUNDATION:__ 
__Main Idea:__ Replace [Graspit!](https://graspit-simulator.github.io/) with a similar simulator using Pybullet 

#### __Graspit! Overview:__
* Tool for grasping research: simulator that can accommodate arbitrary hand and robot designs. 
* Uses ROS/Gazebo
* Loads objects/obstacles of arbitrary geometry 
* Collision detection/contact
* Grasp quality metrics, both numeric/visual
* Key Use: development tool -  execute and test various robot control algorithms in simulation for grasp-grasp-dependent needs
* Key Use: computational platform: in addition to real-world action, adds in ability to plan grasps as needed 

<p align="center">
    <img src="https://github.com/carcamdou/cr_grasper/blob/master/rm_images/graspit.png" width="200">
</p>


#### __Why switch to PyBullet:__
* Python lends itself toward quick development 
* Not reliant on ROS/Gazebo or any particular OS
* Easier use of arbitrary URDF/SDF/etc + better solution for arbitrary robotic hands (still w/config, streamlined)
* Bullet dynamics/kinematics/collisions allows for some better opportunities for simulations
* Make as adaptable as possible for ability to adapt later
* Time speedups possible if needed (C++, no GUI, etc)


##Get Started: pick up and adapt this project from here

The attempt was to make this as OS independent as possible as well as remove dependencies from ROS and Gazebo. 
<br>


#####Some reading/researching to situate yourself in this project:

######Resources + Tools:
Python 3.6 [Docs](https://docs.python.org/3/) <br>
PyBullet [Homepage](https://pybullet.org/wordpress/) <br>
* The documentation is limited somewhat to this [quickstart guide](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.2ye70wns7io3)
* Not everything from bullet has been transfered over to pybullet, but the system is built on Bullet Physics (C++, found [here](https://github.com/bulletphysics/bullet3) )
URDF Overview [here](http://wiki.ros.org/urdf/Tutorials)
######Theoretical background/papers:
* Original GraspIt! 
    * Andrew Miller and Peter K. Allen, Graspit!: A Versatile Simulator for Robotic Grasping
    * Gives a good overview of the scope of this project 
    * [link](http://www.cs.columbia.edu/~allen/PAPERS/graspit.final.pdf)
* Columbia Grasp Database
    * Corey Goldfeder, Matei Ciocarlie, Hao Dang and Peter K. Allen: The Columbia Grasp Database
    * Grasp database of good grasps for objects and hands - can use to match best object/grasp to current situation
    * [link](http://www.cs.columbia.edu/~allen/PAPERS/icra_7page_pub.pdf)
* Grasp Planning
    *  C. Ferrari and J. Canny, “Planning optimal grasps”
    * To understand grasp wrench space and the math behind grasp metrics 
    * [link](https://people.eecs.berkeley.edu/~jfc/papers/92/FCicra92.pdf)
* Eigengrasps
    * Matei Ciocarlie, Corey Goldfeder, Peter Allen: Dimensionality reduction for hand-independent dexterous robotic grasping
    * Example of simulated Annealing use 
    * [link](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=4399227)
* Simulated Annealing
    * L. Ingber, “Very fast simulated re-annealing,”
    * The math behind this
    * [link](https://www.sciencedirect.com/science/article/pii/0895717789902021)



## Current Key Features: 
### __Grasp Planning:__
#### _List Planner:_
The hand circles the object and makes grasps at regular intervals on the sphere around the object. The number of intervals around a sphere and also the number of attempts made for each circumnavigation are user specified. 

<p align="center">
    <img src="https://github.com/carcamdou/cr_grasper/blob/master/rm_images/global%20rotations.gif" width="200">
</p>

Hand distance is chosen by moving the fingers splayed until the hand/palm hits the target. This allows for the closest grasp.

<p align="center">                                              
    <img src="https://github.com/carcamdou/cr_grasper/blob/master/rm_images/palm%20to%20object.gif" width="200">
</p>

Currently, the whole hand is rotated around the wrist axis for each grasp a user specified number of times. 


<p align="center">
    <img src="https://github.com/carcamdou/cr_grasper/blob/master/rm_images/wrist%20rotation.gif" width="200">
</p>

### __Grasp Evaluation Metrics:__ 
#### _Gravity:_
Using the Pybullet dynamics, a force in the downward direction is applied to the object. This simulates the influence of gravity and gives one metric for the success of a grasp - the binary evaluation of remaining in the gripper or not.  

<p align="center">
    <img src="https://github.com/carcamdou/cr_grasper/blob/master/rm_images/grav%20check.gif" width="200">
</p>

#### _Distance Moved:_ 
While this is just a rough estimation (further work could be done here) the distance that an object moves from it's original pose (position, orientation) might indicate the replicability of the grasp and therefore give some understanding of how easily it will translate from simulation to the real world. 
#### _Contact Point Evaluations:_ 
##### Grasp Wrench Space: 
For each of the contact points between the object and the robotic hand, force and torque are calculated in 3 dimensions. The combination of these two create a 6 dimensional vector used to create some numerical metric for grasps. To extend the amount of usable vectors, each contact point uses an n-sided pyramid to create n different Force/Torque vectors for each contact point. The dimensions of this pyramid can be adjusted by the user. If there are sufficent contact points between the hand and the object, then no pyramid need be used. 

This is based on Graspit! techniques and more details on that can be found [here.](https://graspit-simulator.github.io/build/html/grasp_quality.html) The friction cones made from each contact point are shown in the image below (also from graspit!).

<p align="center">
    <img src="https://github.com/carcamdou/cr_grasper/blob/master/rm_images/gws.png" width="200">
</p>


##### Volume: 
From the force/torque vectors calculated for each grasp, create a 6 dimensional convex hull. Then take the volume of that convex hull. 
##### Epsilon:
Distance from the centroid of the 6 dimensional hull to the closest vector


## Usage: 
The goal here is to get the most simple form of a grasp planner working first, then move on to more sophisticated methods

__To Use:__
* __Edit + run write_config.py__  
    - file_paths
        - __robot_path__ - relative path from grasper.py to robot hand URDF file
        - __object_path__ - path to object URDF file
        - __object_scale__ - scale of object in URDF file (to adjust to size of hand, should default to 1)
    - grasp_settings
        - __init_grasp_distance__ - how far from the origin should the hand be at the start (just needs to be beyond the length of the object) to attempt to find closest point
        - __speed_find_distance__ - speed the object moves toward the hand to find ideal grasp distance
        - __grasp_distance_margin__ - how far from touching do you want the palm to be when attempting grips
        - __max_grasp_force__ - max force allowed
        - __target_grasp_velocity__ - target velocity for joints when grasping
        - __grasp_time_limit__ - how long given to find a grasp
        - __active_grasp_joints__ - which joints in the hand to use - specified given joint number from Pybullet getJointInfo
        - __num_grasps_per_cycle__ - number of grasps attempted in each rotation around object (evenly spaced attempts)
        - __num_cycles_to_grasp__ - number of rotations (with angles spread between pi/2 and 0 - ie euler theta) 
        - __use_wrist_rotations__ - binary. allows for rotations around the wrist at each grasp attempt
        - __num_wrist_rotations__ - number of rotations around the wrist axis for each grasp location
    - eval_settings
        - __force_pyramid_sides__ - for grasp wrench space: how many sides does the pyramid approximating the friction cone have
        - __force_pyramid_radius__ - for grasp wrench space: what is the radius of the friction cone approximated by the pyramid
    - gui_settings
        - __use_gui__ - not functioning yet. should allow the whole system to work without visualizations to allow for speedups/parallelization
        - __debug_lines__ - shows axis lines for the hand
        - __debug_text__ - shows text on screen to update the user
       
* __Run grasper.py__
    - Should return a list of good grasps specified by a (Position, Orientation, Joint Angle) Tuple. Also returned is the final pose of the object and the grasp quality metrics (volume and epsilson). At the moment, only grasps that pass the gravity check threshold are returned. 

## __Future Work (where to go from here):__  
* More advanced choices for grasps
    *  having variable movement for different fingers
    *  developing a model for a human-like dexterous hand
    *  give the hand some sensitivity for grasping; ie, force sensors for fingertips
    *  Add support for different types of hands - specifically parallel grippers and those with more than 3 fingers

* Replacing the GraspIt grasp planners
    * Eigengrasp Planner (which relies on hand posture space dimensionality reduction)
    * Database Planner (which relies on a huge database of pre-computed grasps to plan grasps for novel objects.)
        * to this aim, a large database of precomputed grasps could be interesting here, particularly if the object/hand pipeline could be sufficiently automated in the collecton of data
* Some robust speedups
    * Not reloading the hand as much as I have done
    * Multithreadding support
    * Certain portions could be written in the C++ Bullet rather than PyBullet if needed
* Integration with real robots
    * testing the real-world results based on the pybullet dynamics 
    * make sure finger movements are fully constricted to the same as the real world - should be defined by URDF but there seems to be a gap



## NB:

#### On Object files: 
All the files in the ObjectURDFs folder are from the PyBullet examples folder. This can be found [here](https://github.com/bulletphysics/bullet3/tree/master/data) on GitHub.


#### On URDFs: (based on needs for  barrett hand)

__If you have a depreciated .xacro:__ update/clean up using this script in all folders with .xacro files (dependancies and all) - specifically the robots + urdf files for barrett_hand
```sh
$ find . -iname "*.xacro" | xargs sed -i 's#<\([/]\?\)\(if\|unless\|include\|arg\|property\|macro\|insert_block\)#<\1xacro:\2#g'
```
__In the robots folder (all the files in the urdf folder depend on this one):__ convert the urdf.xacro to urdf 
```sh
$ source ~/catkin_ws/devel/setup.bash
$ rosrun xacro xacro --inorder name.urdf.xacro > name.urdf
```

