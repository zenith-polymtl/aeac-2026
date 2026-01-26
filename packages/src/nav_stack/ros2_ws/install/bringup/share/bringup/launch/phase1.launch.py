from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Fichier mission
    mission_file = PathJoinSubstitution([
        FindPackageShare('bringup'),
        'config',
        'trajectoirePhase1.json'
    ])

    return LaunchDescription([

        # -----------------------------
        # Serveur GPS -> Local
        # -----------------------------
        Node(
            package='mission',
            executable='gps_to_local',      # gps_to_local.py
            name='gps_to_local',
            output='screen',
            parameters=[
                {"lat_ref": -35.3633516},
                {"lon_ref": 149.1652413},
                {"alt_ref": 10.0}
            ]
        ),

        # -----------------------------
        # Mission principale
        # -----------------------------
        Node(
            package='mission',
            executable='laps',
            name='laps',
            output='screen',
            parameters=[
                {"mission_file": mission_file}
            ]
        )
    ])

