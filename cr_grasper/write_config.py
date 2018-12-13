from configparser import ConfigParser

config = ConfigParser()
config.read('config.ini')
config.add_section('file_paths')
config.set('file_paths', 'robot_path', '../barrett_hand_ros/barrett_hand_description/urdf/bh.urdf')
config.set('file_paths', 'object_path', 'cube_small.urdf')
config.add_section('grasp_settings')
config.set('grasp_settings', 'init_grasp_distance', '.19')
config.set('grasp_settings', 'grasp_distance_margin', '.01')
config.set('grasp_settings', 'max_grasp_force', '.3')
config.set('grasp_settings', 'target_grasp_velocity', '100')
config.set('grasp_settings', 'grasp_time_limit', '2.0')
config.set('grasp_settings', 'active_grasp_joints', '3,6,9')


with open('config.ini', 'w') as f:
    config.write(f)
