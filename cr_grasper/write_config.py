from configparser import ConfigParser

config = ConfigParser()

###CHANGE NAME TO GET NEW FILE###
config.read('bh_config.ini')
config.add_section('file_paths')
config.set('file_paths', 'robot_path', '/Users/carlyndougherty/PycharmProjects/cr_grasper/RobotURDFs/barrett_hand_description/urdf/bh.urdf')
config.set('file_paths', 'object_path', '/Users/carlyndougherty/PycharmProjects/cr_grasper/ObjectURDFs/cube/cube_small.urdf')
config.set('file_paths', 'object_scale', '1')
config.add_section('grasp_settings')
config.set('grasp_settings', 'init_grasp_distance', '.19')
config.set('grasp_settings', 'speed_find_distance', '1')
config.set('grasp_settings', 'grasp_distance_margin', '.01')
config.set('grasp_settings', 'max_grasp_force', '100')
config.set('grasp_settings', 'target_grasp_velocity', '.5')
config.set('grasp_settings', 'grasp_time_limit', '2.0')
config.set('grasp_settings', 'active_grasp_joints', '3,6,9')
config.set('grasp_settings', 'num_grasps_per_cycle', '6')
config.set('grasp_settings', 'num_cycles_to_grasp', '3')
config.add_section('gui_settings')
config.set('gui_settings', 'use_gui', '1')


###CHANGE NAME TO GET NEW FILE###
with open('bh_config.ini', 'w') as f:
    config.write(f)
