import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # --- 1. Definições de Variáveis ---
    package_name = 'robotics_class'
    pkg_share_dir = get_package_share_directory(package_name)
    params_file_path = os.path.join(pkg_share_dir, 'params', 'default.yaml')

    # --- 2. Declaração de Ações ---
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true'
    )

    # 3. Nós de navegação (os 4 subsistemas da Prática 3)
    # Planner Server (navegador)
    start_planner_server_cmd = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file_path]
    )
    
    # Controller Server (controlador)
    start_controller_server_cmd = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file_path],
        remappings=[('cmd_vel', '/jetauto/cmd_vel')]
    )

    # Behavior Server (comportamentos de navegação)
    start_behavior_server_cmd = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file_path]
    )

    # BT Navigator (árvore de comportamentos)
    start_bt_navigator_cmd = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file_path]
    )

    # Lifecycle Manager para os novos nós de navegação
    start_lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': [
                'planner_server',
                'controller_server',
                'behavior_server',
                'bt_navigator'
            ]
        }]
    )
    
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    
    # Adicione apenas os nós da Prática 3 ao LaunchDescription
    ld.add_action(start_planner_server_cmd)
    ld.add_action(start_controller_server_cmd)
    ld.add_action(start_behavior_server_cmd)
    ld.add_action(start_bt_navigator_cmd)
    ld.add_action(start_lifecycle_manager_cmd)

    return ld
# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node

# def generate_launch_description():
#     package_name = 'robotics_class'
#     pkg_share_dir = get_package_share_directory(package_name)
#     params_file_path = os.path.join(pkg_share_dir, 'params', 'default.yaml')

#     declare_use_sim_time_cmd = DeclareLaunchArgument(
#         'use_sim_time',
#         default_value='true',
#         description='Use simulation clock if true'
#     )

#     # Nós do Navigation2
#     start_bt_navigator_cmd = Node(
#         package='nav2_bt_navigator',
#         executable='bt_navigator',
#         name='bt_navigator',
#         output='screen',
#         parameters=[params_file_path]
#     )

#     start_controller_server_cmd = Node(
#         package='nav2_controller',
#         executable='controller_server',
#         name='controller_server',
#         output='screen',
#         parameters=[params_file_path]
#     )

#     start_planner_server_cmd = Node(
#         package='nav2_planner',
#         executable='planner_server',
#         name='planner_server',
#         output='screen',
#         parameters=[params_file_path]
#     )

#     start_behavior_server_cmd = Node(
#         package='nav2_behaviors',
#         executable='behavior_server',
#         name='behavior_server',
#         output='screen',
#         parameters=[params_file_path]
#     )

#     start_global_costmap_cmd = Node(
#         package='nav2_costmap_2d',
#         executable='nav2_costmap_2d',  # ← CORRIGIDO
#         name='global_costmap',
#         output='screen',
#         parameters=[params_file_path],
#         namespace='global_costmap'
#     )

#     start_local_costmap_cmd = Node(
#         package='nav2_costmap_2d',
#         executable='nav2_costmap_2d',  # ← CORRIGIDO
#         name='local_costmap',
#         output='screen',
#         parameters=[params_file_path],
#         namespace='local_costmap'
#     )
    
#     start_lifecycle_manager_cmd = Node(
#         package='nav2_lifecycle_manager',
#         executable='lifecycle_manager',
#         name='lifecycle_manager_navigation',
#         output='screen',
#         parameters=[{
#             'autostart': True,
#             'node_names': [
#                 'controller_server',
#                 'planner_server',
#                 'behavior_server',
#                 'bt_navigator',
#                 'global_costmap',     
#                 'local_costmap'       
#             ]
#         }]
#     )

#     ld = LaunchDescription()
#     ld.add_action(declare_use_sim_time_cmd)
#     ld.add_action(start_bt_navigator_cmd)
#     ld.add_action(start_controller_server_cmd)
#     ld.add_action(start_planner_server_cmd)
#     ld.add_action(start_behavior_server_cmd)
#     ld.add_action(start_global_costmap_cmd)
#     ld.add_action(start_local_costmap_cmd)
#     ld.add_action(start_lifecycle_manager_cmd)

#     return ld