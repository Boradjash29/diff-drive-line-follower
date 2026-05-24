import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    package_name = 'diff_line_following_robot'
    pkg_share = get_package_share_directory(package_name)
    
    # Process the URDF file
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}
    
    # Gazebo launch
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    world_path = os.path.join(pkg_share, 'worlds', 'advanced.world')
    
    # Set gazebo models path so it can find beginner_track
    os.environ["GAZEBO_MODEL_PATH"] = os.path.join(pkg_share, 'models')
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )
    
    # Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
    
    # Spawn Entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'line_following_robot', '-x', '-6.5', '-y', '0.0', '-z', '0.1'],
        output='screen'
    )
    
    # Line Detector Node
    line_detector_node = Node(
        package=package_name,
        executable='line_detector_node.py',
        output='screen'
    )
    
    # Controller Node
    controller_node = Node(
        package=package_name,
        executable='controller_node.py',
        output='screen'
    )
    
    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        line_detector_node,
        controller_node
    ])
