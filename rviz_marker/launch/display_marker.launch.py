from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_path
import os

class StreamlitAction(ExecuteProcess):
    def __init__(self):
        super().__init__(
            cmd=['python3', '-m', 'streamlit', 'run', '/home/neo/Robotics/ros2_ws/src/GUI.py'],
            name='streamlit_gui',
            output='screen'
        )

class GazeboAction(ExecuteProcess):
    def __init__(self):
        super().__init__(
            cmd=['ros2','launch','elevator_sim','elevator_sim.launch.py'],
            name='Gazebo_Action',
            output='screen'
        )


def generate_launch_description():
    rviz_config_path = os.path.join(get_package_share_path('rviz_marker'),
                                    'rviz', 'rviz_config.rviz')
    
    marker_node = Node(
        package="rviz_marker",
        executable="display_marker"
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path]
    )

    map_publisher_node = Node(
        package="map_publisher",
        executable="map_publisher_node"
    )

    path_planner_node = Node(
        package="PathPlanners",
        executable="planner_test",
    )

    agent_node_1 = Node(
        package="agent",
        executable="service_test",
        arguments=["agent_1","0", "0", "0"],
    )

    agent_node_2 = Node(
        package="agent",
        executable="service_test",
        arguments=["agent_2","2", "0", "0"],
    )

    agent_node_3 = Node(
        package="agent",
        executable="service_test",
        arguments=["agent_3","4", "0", "0"],
    )
    agent_node_4 = Node(
        package="agent",
        executable="service_test",
        arguments=["agent_4","6", "0", "0"],
    )

    # Add the Streamlit GUI action
    streamlit_gui = StreamlitAction()

    #gazebo_init =GazeboAction()
    
    return LaunchDescription([
        marker_node,
        rviz2_node,
        map_publisher_node,
        path_planner_node,
        agent_node_1,
        agent_node_2,
        agent_node_3,
        agent_node_4,
        streamlit_gui,
        #gazebo_init,
    ])