from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get shared directories
    robot_desc_share = get_package_share_directory('mujoco_ros2_interface')

    # Construct full model path
    model_path = os.path.join(
        robot_desc_share,
        'mjcf/panda_mujoco/world.xml'
    )

    # Define node
    mujoco_node = Node(
        package='mujoco_ros2_interface',
        executable='mujoco_node',
        name='mujoco_node',
        output='screen',
        parameters=[{
            'model_path': model_path
        }]
    )

    # Optional: print the resolved model path (for debugging)
    print(f"[Launch] Using model path: {model_path}")

    # Return the launch description
    return LaunchDescription([mujoco_node])
