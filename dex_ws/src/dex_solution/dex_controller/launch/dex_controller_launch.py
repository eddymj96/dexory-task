import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    dex_controller_dir = get_package_share_directory("dex_controller")
    dex_controller_params_file = os.path.join(dex_controller_dir, "params", "dex_controller_params.yaml")
    
    param_substitutions = {"use_sim_time": "false", "autostart": "true"}

    print(param_substitutions)
    
    dex_controller_configured_params = RewrittenYaml(
    source_file=dex_controller_params_file,
    root_key="",
    param_rewrites=param_substitutions,
    convert_types=True,
    )
  
   

    return LaunchDescription([
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[dex_controller_configured_params],
            arguments=["--ros-args", "--log-level", "info"]
        )
    ])