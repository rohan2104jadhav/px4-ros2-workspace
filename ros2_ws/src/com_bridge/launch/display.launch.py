import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='com_bridge',
            executable='drone_flight',
            name='drone_flight',
            output='screen',)
    ])