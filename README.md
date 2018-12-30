# __README: Robot Graspit! Project__  
__Carlyn Dougherty (ccd2134@columbia.edu)__


##__FOUNDATION:__ 
__Main Idea:__ Replace [Graspit!](https://graspit-simulator.github.io/) with a similar simulator using Pybullet (https://pybullet.org/wordpress/). 

####__Graspit! Overview:__
* Tool for grasping research: simulator that can accommodate arbitrary hand and robot designs. 
* Uses ROS/Gazebo
* Loads objects/obstacles of arbitrary geometry 
* Collision detection/contact
* Grasp quality metrics, both numeric/visual
* Key Use: development tool -  execute and test various robot control algorithms in simulation for grasp-grasp-dependent needs
* Key Use: computational platform: in addition to real-world action, adds in ability to plan grasps as needed 

####__Why switch to PyBullet:__
* Python lends itself toward quick development 
* Not reliant on ROS/Gazebo or any particular OS
* Easier use of arbitrary URDF/SDF/etc + better solution for arbitrary robotic hands (still w/config, streamlined)
* Bullet dynamics/kinematics/collisions allows for some better opportunities for simulations
* Make as adaptable as possible for ability to adapt later
* Time speedups possible if needed (C++, no GUI, etc)


## Key Features: 
### __Grasp Planning:__
#### _List Planner:_
The hand circles the object and makes grasps at regular intervals on the sphere around the object. The number of intervals around a sphere and also the number of attempts made for each circumnavigation are user specified. 

Hand distance is chosen by moving the fingers splayed until the hand/palm hits the target. This allows for the closest grasp.

Currently, the whole hand is rotated around the wrist axis for each grasp a user specified number of times. 


#### _Future Work:_ 
More advanced choices for grasps. Replacing the Eigengrasp Planner which relies on hand posture space dimensionality reduction and the Database Planner family, which relies on a huge database of pre-computed grasps to plan grasps for novel objects. These two can be found in Graspit!

### __Grasp Evaluation Metrics:__ 
#### _Gravity:_
Using the Pybullet dynamics, a force in the downward direction is applied to the object. This simulates the influence of gravity and gives one metric for the success of a grasp - the binary evaluation of remaining in the gripper or not.  
#### _Distance Moved:_ 
While this is just a rough estimation (further work could be done here) the distance that an object moves from it's original pose (position, orientation) might indicate the replicability of the grasp and therefore give some understanding of how easily it will translate from simulation to the real world. 
#### _Contact Point Evaluations:_ 
##### Grasp Wrench Space: 
For each of the contact points between the object and the robotic hand, force and torque are calculated in 3 dimensions. The combination of these two create a 6 dimensional vector used to create some numerical metric for grasps. To extend the amount of usable vectors, each contact point uses an n-sided pyramid to create n different Force/Torque vectors for each contact point. The dimensions of this pyramid can be adjusted by the user. If there are sufficent contact points between the hand and the object, then no pyramid need be used. 
##### Volume: 
From the force/torque vectors calculated for each grasp, create a 6 dimensional convex hull. Then take the volume of that convex hull. 
##### Epsilon:
Distance from the centroid of the 6 dimensional hull to the closest vector

 
### __Grasp Evaluation Metrics:__ 
#### _Gravity:_



## Usage: 
The goal here is to get the most simple form of a grasp planner working first, then move on to more sophisticated methods

__To Use:__
* __Edit + run write_config__  
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
    - Should return a list of good grasps specified by a (Position, Orientation, Joint Angle) Tuple


##NOTES:

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

