from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            namespace= "shipsim1", package='shipsim', executable='shipsim_node', output='screen'),
        launch_ros.actions.Node(
            namespace= "shipsim2", package='shipsim', executable='shipsim_node', output='screen'),
    ])
