"""Servo discovery and start positions (no ROS graph): the dome emulator's rules."""

import importlib.util
import math
import os
import xml.etree.ElementTree as ET

# servo_sim.py is a script (installed as an executable), so load it by path
SERVO_SIM = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                         'r2_sim', 'servo_sim.py')
_spec = importlib.util.spec_from_file_location('servo_sim', SERVO_SIM)
servo_sim = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(servo_sim)
dome_servo_limits = servo_sim.dome_servo_limits
joint_key = servo_sim.joint_key
start_position = servo_sim.start_position

NS = 'xmlns:astromech="https://astromech.co/urdf/1.0"'


def joint(name, parent, child, lower, upper, servo=True, kind='revolute'):
    tag = '<astromech:servo display="x"/>' if servo else ''
    return (f'<joint name="{name}" type="{kind}"><parent link="{parent}"/>'
            f'<child link="{child}"/><limit lower="{lower}" upper="{upper}"/>{tag}</joint>')


LIMIT = math.radians(25)
URDF = ET.fromstring(f'''<robot {NS} name="r">
  {joint('body_panel_joint', 'body', 'body_panel', 0, math.pi / 2)}
  {joint('dome_joint', 'body', 'dome', 0, 0, servo=False, kind='continuous')}
  {joint('pp1_joint', 'dome', 'pp1', 0, math.pi / 2)}
  {joint('hp_front_mount', 'dome', 'hp_front', 0, 0, servo=False, kind='fixed')}
  {joint('hp_front_x_joint', 'hp_front', 'hp_front_gimbal', -LIMIT, LIMIT)}
  {joint('hp_front_y_joint', 'hp_front_gimbal', 'hp_front_body', -LIMIT, LIMIT)}
  {joint('untagged_joint', 'dome', 'other', 0, 1, servo=False)}
</robot>''')


def test_dome_servos_include_descendants_two_deep():
    limits = dome_servo_limits(URDF)
    assert set(limits) == {'pp1_joint', 'hp_front_x_joint', 'hp_front_y_joint'}
    assert limits['hp_front_y_joint'] == (-LIMIT, LIMIT)


def test_body_servos_excluded():
    assert 'body_panel_joint' not in dome_servo_limits(URDF)


def test_start_position():
    assert start_position(-LIMIT, LIMIT) == 0.0
    assert start_position(0.0, math.pi / 2) == 0.0
    assert start_position(0.2, 1.0) == 0.2     # 0 is outside: clamped to the lower limit
    assert start_position(-1.0, -0.2) == -0.2


def test_clamping_at_limits():
    lower, upper = dome_servo_limits(URDF)['hp_front_x_joint']
    assert servo_sim.clamp(1.0, lower, upper) == LIMIT
    assert servo_sim.clamp(-1.0, lower, upper) == -LIMIT
    assert servo_sim.clamp(0.1, lower, upper) == 0.1


def test_joint_key():
    assert joint_key('hp_front_x') == 'hp_front_x_joint'
    assert joint_key('hp_front_x_joint') == 'hp_front_x_joint'
    assert joint_key('pp1') == 'pp1_joint'
