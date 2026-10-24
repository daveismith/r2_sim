"""Launch-argument test (no ROS graph): launch_sim.launch.py declares the converter cap."""

import importlib.util
import os

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription

LAUNCH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                      'launch', 'launch_sim.launch.py')


def test_sim_declares_personality_max_speed():
    spec = importlib.util.spec_from_file_location('launch_sim', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    args = {a.name: a for a in module.generate_launch_description().entities
            if isinstance(a, DeclareLaunchArgument)}
    assert ''.join(t.text for t in args['personality_max_speed_rad_s'].default_value) == '2.0'


def test_sim_forwards_wav_backend_and_mock_level_topic():
    spec = importlib.util.spec_from_file_location('launch_sim', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    forwarded = None
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, IncludeLaunchDescription):
            arguments = dict(entity.launch_arguments)
            if 'sound_backend' in arguments:
                forwarded = arguments
    assert forwarded is not None
    assert forwarded['sound_backend'] == 'wav'
    topic = forwarded['dome_level_topic']
    assert len(topic) == 3 and topic[0] == '/' and topic[2] == '/dome/audio/level'
    assert topic[1].variable_name[0].text == 'dome_mock_name'
