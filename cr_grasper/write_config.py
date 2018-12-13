from configparser import ConfigParser

config = ConfigParser()
config.read('config.ini')
config.add_section('file_paths')
config.set('file_paths', 'robot_path', '../RobotURDFs/barrett_hand_description/urdf/bh.urdf')
config.set('file_paths', 'object_path', '/Users/carlyndougherty/PycharmProjects/cr_grasper/pybullet_examples/bullet3/data/cube_small.urdf')
config.set('file_paths', 'object_scale', '.1')
config.add_section('grasp_settings')
config.set('grasp_settings', 'init_grasp_distance', '.19')
config.set('grasp_settings', 'grasp_distance_margin', '.01')
config.set('grasp_settings', 'max_grasp_force', '.3')
config.set('grasp_settings', 'target_grasp_velocity', '100')
config.set('grasp_settings', 'grasp_time_limit', '2.0')
config.set('grasp_settings', 'active_grasp_joints', '3,6,9')
config.add_section('gui_settings')
config.set('gui_settings', 'use_gui', '1')



with open('config.ini', 'w') as f:
    config.write(f)
