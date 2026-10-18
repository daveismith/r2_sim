import os
import yaml

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch_ros.actions import Node
from math import radians


def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    bringup_package_name = 'r2_bringup'
    description_package_name = 'r2_description'
    package_name='r2_sim'
    web_package_name = 'r2_web_interface'
    worlds_package_name = 'gazebo_models_worlds_collection'

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(bringup_package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(bringup_package_name),'launch','joystick.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    twist_mux_params = os.path.join(get_package_share_directory(bringup_package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel')]
        )

    default_world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'obstacles.world'
        #'office_env_large.world'
        )    
    
    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
        )

    # gui:=false runs only the Gazebo server, for machines without a display (the dev
    # container); view the robot in Foxglove instead (sim_foxglove.launch.py).
    # --headless-rendering lets the server's ogre2 sensors render through EGL.
    gui = LaunchConfiguration('gui')

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        choices=['true', 'false'],
        description='Start the Gazebo GUI; false runs the server headless'
        )

    server_args = PythonExpression(["'' if '", gui, "' == 'true' else '-s --headless-rendering '"])

    # Include the Gazebo launch file, provided by the ros_gz_sim package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                    launch_arguments={'gz_args': ['-r -v4 ', server_args, world], 'on_exit_shutdown': 'true'}.items()
             )

    # Run the spawner node from the ros_gz_sim package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'my_bot',
                                   '-z', '0.1'],
                        output='screen')

    ankle_pos_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ankle_pos"],
        output='screen'
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    dome_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["dome_controller"],
    )

    dome_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["dome_state_broadcaster"],
    )

    servo_pos_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["servo_pos"],
    )

    # Servo joints: servo_sim mirrors /dome/joint_states into Gazebo (dome panels through
    # their per-joint controllers, the rest through servo_pos), and with servo_device:=sim
    # also emulates the dome servo device. With servo_device:=real the real device on the
    # network provides /dome/joint_states instead.
    servo_device = LaunchConfiguration('servo_device')

    servo_device_arg = DeclareLaunchArgument(
        'servo_device',
        default_value='sim',
        choices=['sim', 'real'],
        description='sim: emulate the dome servos a real device on the network is not publishing '
                    '(all of them without a device); real: no emulation, the device drives them'
        )

    sim_controllers = os.path.join(get_package_share_directory(package_name),'config','sim_controllers.yaml')
    with open(sim_controllers) as f:
        controller_joints = yaml.safe_load(f)['servo_pos']['ros__parameters']['joints']

    dome_servos = os.path.join(get_package_share_directory(package_name),'config','dome_servos.yaml')
    with open(dome_servos) as f:
        gz_joints = list(yaml.safe_load(f))

    # Commands for the dome panels' Gazebo JointPositionControllers (see gz_ros2_control.xacro)
    servo_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="servo_bridge",
        arguments=[f'/servo/{j}/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double' for j in gz_joints]
    )

    servo_sim = Node(
        package=package_name,
        executable="servo_sim",
        output='screen',
        parameters=[{
            'controller_joints': controller_joints,
            'gz_joints': gz_joints,
            'emulate': PythonExpression(["'", servo_device, "' == 'sim'"]),
            'use_sim_time': True,
        }]
    )

    dome_teleop = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(bringup_package_name),'launch','dome_teleop.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    joint_states = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(bringup_package_name),'launch','joint_states.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    web_interface = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(web_package_name),'launch','web_interface.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'robot_mode': 'sim'}.items()
    )

    angle = 18
    rads = radians(angle)

    set_angles = ExecuteProcess(
        cmd=[
            "ros2", "topic", "pub", "--once",
            "/ankle_pos/commands", "std_msgs/msg/Float64MultiArray",
            f"{{data: [-{rads}, -{rads}, {rads * 2}, -{rads}]}}"  # 30 deg in radians; add more values if you listed more joints
        ],
        output="screen",
    )


    bridge_params = os.path.join(get_package_share_directory(package_name),'config','gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"]
    )



    # Start controllers in the same order as the real robot: the gz_ros2_control
    # controller_manager comes up once the model is spawned, then joint_broad,
    # then the remaining controllers.
    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_broad_spawner],
        )
    )

    delayed_controller_spawners = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_broad_spawner,
            on_exit=[
                diff_drive_spawner,
                dome_controller_spawner,
                ankle_pos_spawner,
                servo_pos_spawner,
                dome_state_broadcaster_spawner,
            ],
        )
    )

    # Only command the leg pose once ankle_pos is active
    delayed_set_angles = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=ankle_pos_spawner,
            on_exit=[set_angles],
        )
    )


    # Launch them all!
    return LaunchDescription([
        rsp,
        joystick,
        twist_mux,
        world_arg,
        gui_arg,
        servo_device_arg,
        gazebo,
        spawn_entity,
        delayed_joint_broad_spawner,
        delayed_controller_spawners,
        delayed_set_angles,
        joint_states,
        servo_sim,
        dome_teleop,
        web_interface,
        ros_gz_bridge,
        ros_gz_image_bridge,
        servo_bridge,
    ])
