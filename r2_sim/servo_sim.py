#!/usr/bin/env python3
"""Servo joints in simulation.

Mirror (always): /dome/joint_states -> Gazebo, so the servo joints follow whatever the
servo device reports. Each joint goes where it is driven: the dome panels (gz_joints) to
their own Gazebo JointPositionController on /servo/<joint>/cmd_pos, which moves them at
servo speed; the rest (controller_joints) to the servo_pos ros2_control controller on
/servo_pos/commands. Joints it never hears about stay at 0 (closed).

Emulate (emulate:=true): stands in for the dome servo device (r2_domeplayer) with the same
contract: /dome/joint_command in (sensor_msgs/JointState, radians, `pp1` or `pp1_joint`),
clamped to the URDF <limit>, and /dome/joint_states out at publish_rate (and on every
command) carrying the commanded target. Its servos are the <astromech:servo> joints on the
dome, read from /robot_description, the same way the device decides.

A real device on the network may publish only some of the servos (the ones it has
configured). The emulator fills in the rest: it watches /dome/joint_states, and any joint
another publisher has reported within the last claim_timeout seconds is muted here, so
each joint has exactly one source. Its own messages are told apart by header.frame_id ==
"servo_sim" (the device leaves it empty). While muted, a joint's target follows the
device's reported position, so emulation resumes from there if the device stops.
"""

import math
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray, String

SERVO_TAG = '{https://astromech.co/urdf/1.0}servo'
DEVICE_PARENT_LINK = 'dome'
OWN_FRAME_ID = 'servo_sim'   # marks the emulator's own /dome/joint_states messages


def joint_key(name):
    """Accept `pp1` or `pp1_joint`, as the device does."""
    return name if name.endswith('_joint') else f'{name}_joint'


class ServoSim(Node):

    def __init__(self):
        super().__init__('servo_sim')

        self.declare_parameter('controller_joints', [''])
        self.declare_parameter('gz_joints', [''])
        self.declare_parameter('emulate', True)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('claim_timeout', 1.0)

        self.controller_joints = [j for j in self.get_parameter('controller_joints').value if j]
        self.gz_joints = [j for j in self.get_parameter('gz_joints').value if j]
        self.emulate = self.get_parameter('emulate').value
        period = 1.0 / self.get_parameter('publish_rate').value
        self.claim_timeout = self.get_parameter('claim_timeout').value

        # Mirror: last reported position of every servo joint
        self.mirrored = {j: 0.0 for j in self.controller_joints + self.gz_joints}
        self.controller_pub = self.create_publisher(Float64MultiArray, '/servo_pos/commands', 10)
        self.gz_pubs = {
            j: self.create_publisher(Float64, f'/servo/{j}/cmd_pos', 10) for j in self.gz_joints}
        self.create_subscription(JointState, '/dome/joint_states', self.on_dome_states, 10)
        self.create_timer(period, self.publish_commands)

        # Emulator: joint -> (lower, upper) and commanded targets, filled from the URDF
        self.limits = {}
        self.targets = {}
        # (joint, reason) refusals already logged: tools like joint_state_publisher_gui
        # send every joint at 10 Hz, so each refusal is logged only the first time
        self.refused = set()
        # joint -> when another publisher last reported it (wall clock: the device runs
        # on real time, whatever the sim's real-time factor), and the joints muted now
        self.claimed = {}
        self.muted = set()
        if self.emulate:
            self.states_pub = self.create_publisher(JointState, '/dome/joint_states', 10)
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(String, '/robot_description', self.on_description, latched)
            self.create_subscription(JointState, '/dome/joint_command', self.on_command, 10)
            self.create_timer(period, self.publish_states)

        self.get_logger().info(
            f'{len(self.gz_joints)} Gazebo-driven and {len(self.controller_joints)} '
            f'controller-held servo joints, emulating the servo device: {self.emulate}')

    # ---- mirror ----------------------------------------------------------------------

    def on_dome_states(self, msg):
        foreign = msg.header.frame_id != OWN_FRAME_ID
        now = time.monotonic()
        for name, position in zip(msg.name, msg.position):
            key = joint_key(name)
            if not math.isfinite(position):
                continue
            if key in self.mirrored:
                self.mirrored[key] = position
            if foreign:
                self.claimed[key] = now
                if key in self.targets:
                    self.targets[key] = position
        # Forward straight away so Gazebo starts moving without waiting for the timer
        self.publish_commands()

    def publish_commands(self):
        # Also re-sent every period, so a controller or bridge that started late still
        # gets a target
        if self.controller_joints:
            self.controller_pub.publish(
                Float64MultiArray(data=[self.mirrored[j] for j in self.controller_joints]))
        for joint, pub in self.gz_pubs.items():
            pub.publish(Float64(data=self.mirrored[joint]))

    # ---- emulator --------------------------------------------------------------------

    def on_description(self, msg):
        try:
            root = ET.fromstring(msg.data)
        except ET.ParseError as e:
            self.get_logger().error(f'robot_description is not valid XML: {e}')
            return

        limits = {}
        for joint in root.findall('joint'):
            parent = joint.find('parent')
            limit = joint.find('limit')
            if joint.find(SERVO_TAG) is None or parent is None or limit is None:
                continue
            if parent.get('link') != DEVICE_PARENT_LINK:
                continue
            limits[joint.get('name')] = (float(limit.get('lower', 0.0)),
                                         float(limit.get('upper', 0.0)))

        self.limits = limits
        # Start closed (lower limit); keep targets for joints that are still servos
        self.targets = {j: self.targets.get(j, lo) for j, (lo, _) in limits.items()}
        self.get_logger().info(f'emulating {len(limits)} servos: {", ".join(sorted(limits))}')

    def on_command(self, msg):
        for i, name in enumerate(msg.name):
            if i >= len(msg.position) or not math.isfinite(msg.position[i]):
                continue
            key = joint_key(name)
            if key not in self.limits:
                reason = 'unknown joint' if self.limits else 'no joint limits'
                if (key, reason) not in self.refused:
                    self.refused.add((key, reason))
                    self.get_logger().warn(f'{name}: refused, {reason} (logged once)')
                continue
            lower, upper = self.limits[key]
            position = min(max(msg.position[i], lower), upper)
            if position != msg.position[i]:
                self.get_logger().info(
                    f'{name}: {msg.position[i]:.3f} clamped to {position:.3f} '
                    f'[{lower:.3f}, {upper:.3f}]')
            self.targets[key] = position
        # The real servo starts moving as soon as it's commanded, so report the new target
        # straight away rather than on the next periodic publish
        self.publish_states()

    def update_muted(self):
        now = time.monotonic()
        muted = {j for j, t in self.claimed.items()
                 if j in self.targets and now - t < self.claim_timeout}
        if muted - self.muted:
            self.get_logger().info(
                f'another publisher is reporting {", ".join(sorted(muted - self.muted))}: '
                'muting emulation for them')
        if self.muted - muted:
            self.get_logger().info(
                f'no reports for {", ".join(sorted(self.muted - muted))} in '
                f'{self.claim_timeout:.1f} s: emulating them again')
        self.muted = muted

    def publish_states(self):
        self.update_muted()
        joints = [j for j in self.targets if j not in self.muted]
        if not joints:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = OWN_FRAME_ID
        msg.name = joints
        msg.position = [self.targets[j] for j in joints]
        self.states_pub.publish(msg)


def main(args=None):
    # Leave signals to Python: rclpy's own handlers race the shutdown below on SIGINT
    # ("rcl_shutdown already called") and hang on SIGTERM under rmw_zenoh. SIGINT then
    # arrives as KeyboardInterrupt, and SIGTERM ends the process. Spin with a timeout so
    # Python gets to handle SIGINT even when nothing wakes the node (e.g. no /clock yet).
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = ServoSim()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
