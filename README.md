# __README: Robot Graspit! Project__  
__Carlyn Dougherty (ccd2134@columbia.edu)__


__Main Idea:__ replace [Graspit!](https://graspit-simulator.github.io/) with a similar simulator using Pybullet (https://pybullet.org/wordpress/) to 


## Method #1: List Planner: 
The goal here is to get the most simple form of a grasp planner working first, then move on to more sophisticated methods

__To Use:__
* __Edit + run write_config__  
    - file_paths
        - relative path from grasper.py to hand URDF file
        - path to object URDF file
        - scale of object in URDF file (to adjust to size of hand, should default to 1)
    - grasp_settings
        - __init_grasp_distance__ - how far from the origin should the hand be at the start (just needs to be beyond the length of the object) to attempt to find closest point
        - __speed_find_distance__ - speed the object moves toward the hand to find ideal grasp distance
        - __grasp_distance_margin__ - how far from touching do you want the palm to be when attempting grips
        - __max_grasp_force__ - max force allowed
        - __target_grasp_velocity__ - target velocity for joints when grasping
        - __grasp_time_limit__ - how long given to find a grasp
        - __active_grasp_joints__ - which joints in the hand to use - specified given joint number from Pybullet getJointInfo
        - __num_grasps_per_cycle__ - number of grasps attempted in each rotation around object (evenly spaced attempts)
        - __max_grasp_force__ - number of rotations (with angles spread between pi/2 and 0) 

* __Run grasper.py__
    - Should return a list of good grasps specified by a (Position, Orientation, Joint Angle) Tuple



All the files in the ObjectURDFs folder are from the PyBullet examples folder. Here [on GitHub](https://github.com/bulletphysics/bullet3/tree/master/data).


##### Notes on URDFs: (based on needs for  barrett hand)

__If you have a depreciated .xacro:__ update/clean up using this script in all folders with .xacro files (dependancies and all) - specifically the robots + urdf files for barrett_hand
```sh
$ find . -iname "*.xacro" | xargs sed -i 's#<\([/]\?\)\(if\|unless\|include\|arg\|property\|macro\|insert_block\)#<\1xacro:\2#g'
```
__In the robots folder (because all the ones in the urdf folder depend on this one):__ convert the urdf.xacro to urdf 
```sh
$ source ~/catkin_ws/devel/setup.bash
$ rosrun xacro xacro --inorder name.urdf.xacro > name.urdf
```

