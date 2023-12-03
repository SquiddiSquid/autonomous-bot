from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import SetEnvironmentVariable
from os.path import join as Path

def generate_launch_description():
    raido_bot_package = get_package_share_path('raido-bot_description')
    ign_pkg_sim = get_package_share_path('ros_ign_gazebo')

    default_model_path = raido_bot_package / 'urdf/raido-bot.urdf'
    default_rviz_config_path = raido_bot_package / 'rviz/config.yaml.rviz'
    default_world_path = Path(raido_bot_package, 'worlds' , 'my_world.sdf')
    default_meshes_path = Path(raido_bot_package, '..')

    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path), description='Path to robot urdf file')
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path), description='Path to rviz config file')

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]), value_type=str)

    env_var = SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', default_meshes_path)

    ign_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=Path(ign_pkg_sim, 'launch', 'ign_gazebo.launch.py')
        ),
        launch_arguments={"gz_args": "-r " + default_world_path}.items()
    )

    #ros_gz_bridge = Node(
    #    package="ros_gz_bridge",
    #    executable="parameter_bridge",
    #    arguments=[
    #            "/model/diffbot/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist"
    #        ],
    #    remappings=[("model/diffbot/cmd_vel", "/cmd_vel")],
    #    output="screen"
    #   ),

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')]
    )

    raido_drive_node = Node(
        package='raido-bot_description',
        executable='raido_drive',
        output='screen'
    )

    return LaunchDescription([
        env_var,
        model_arg,
        #rviz_arg,

        #ros_gz_bridge,
        joint_state_publisher_gui_node,
        ign_sim,
        robot_state_publisher_node,
        #rviz_node,
        raido_drive_node
    ])