from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()

    # init = Node(
    #     package="nav_stack",
    #     executable="init",
    #     name="init",
    # )
    
    # convert = Node(
    #     package="nav_stack",
    #     executable="convert",
    #     name="convert",
    # )

    sim = Node(
        package = 'remote_controller_interface',
        executable = 'rc_simulator',
        name = 'rc_simulator'
    )
    
    remote_interface = Node(
        package = 'remote_controller_interface',
        executable = 'remote_controller_interface',
        name = 'remote_controller_interface'
    )


    ld.add_action(sim)
    ld.add_action(remote_interface)
    # ld.add_action(init)
    # ld.add_action(convert)
    
    return ld