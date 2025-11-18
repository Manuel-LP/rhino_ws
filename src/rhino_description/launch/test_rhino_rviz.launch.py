import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FileContent
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    ld = LaunchDescription()
    pkg_share = FindPackageShare(package='rhino_description').find('rhino_description')
    default_model_path = os.path.join(pkg_share, 'description/rhino.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/rhino.rviz')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    #urdf= FileContent( PathJoinSubstitution([FindPackageShare('rhino_description', 'description/rhino.urdf')]))
    urdf = FileContent(PathJoinSubstitution([FindPackageShare('rhino_description'), 'description/rhino.urdf']))
    gui_arg = DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')
    ld.add_action(gui_arg)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}],
    )
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('jsp_gui'))
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    SerialToJointState_node = Node(
        package='rhino_description',
        executable='SerialToJointState',
        name='SerialToJointState',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': urdf}],
        arguments=[urdf]
    )
    JointControl_node= Node(
        package='rhino_description',
        executable='JointControl',
        name='JointControl',
        output='screen',
        #parameters=[{'use_sim_time': use_sim_time, 'robot_description': urdf}],
        #arguments=[urdf]
    )
    NewControl_node = Node(
        package='rhino_description',
        executable='NewControl',
        name='NewControl',
        #parameters=[{'use_sim_time': use_sim_time, 'robot_description': urdf}],
        #arguments=[urdf]
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot xacro file'),
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        DeclareLaunchArgument(
                                'use_sim_time',
                                default_value='false',
                                description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument (   name='jsp_gui',
                                default_value='true',
                                choices=['true', 'false'],
                                description='Flag to enable joint_state_publisher_gui'),
        #joint_state_publisher_gui_node,
        robot_state_publisher_node,
        #SerialToJointState_node,
        rviz_node,
        #JointControl_node,
        #NewControl_node
    ])
