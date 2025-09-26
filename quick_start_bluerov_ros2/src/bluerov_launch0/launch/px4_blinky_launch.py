import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace

from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import AnyLaunchDescriptionSource


# Fonction appelé par ros2 launch pour avoir la liste des nodes à lancer

def generate_launch_description():

    config_blinky = os.path.join(
        get_package_share_directory('bluerov_launch0'),
        'config', # repertoire (dans le dossier "share/(...)" qui a été generé et non dans le "src")
        'parametre_blinky.yaml' # nom du fichier .yaml
    )

    
    launch_mavros_blinky = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("mavros"), "launch"),
                "/px4.launch",
            ]
        ),
        launch_arguments={
            'fcu_url': 'udp://192.168.4.100:14550@192.168.4.2:14550',
            'tgt_system':'3',
            'tgt_component':'1',
            "fcu_protocol":"v2.0",
            "gcs_url":"", 
            
            "/mavros/conn/timesync_rate":"0.0",
        }.items()

    )

    launch_mavros_with_namespace_blinky = GroupAction(
     actions=[
         PushRosNamespace('blinky'),
         launch_mavros_blinky,
      ]
   )


    node_control_blinky = Node(
        package="bluerov_control0", 
        namespace='blinky',
        executable="control_bluerov", 
        name="control_blinky", 
        output="screen",
        emulate_tty= True,
        parameters=[config_blinky]

    )

    #"""

    node_ihm_blinky = Node(
        package="bluerov_ihm0", 
        namespace='blinky',
        executable="ihm", 
        name="ihm_blinky", 
        output="screen",
        emulate_tty= True,
        parameters=[config_blinky]
    )


    node_tracking_blinky = Node(
        package="bluerov_tracking0", 
        namespace='blinky',
        executable="tracking_node", 
        name="tracking_blinky", 
        output="screen",
        emulate_tty= True,
        parameters=[config_blinky]
    )


    ######## Nodes commun #######


    node_joy = Node(
        package="joy", 
        executable="joy_node", 
        name="joy_node", 
        output="screen"
    )


    # retour de la fonction avec la liste des nodes à lancer
    
    return LaunchDescription([
    
    	#launch_mavros_blinky,
        launch_mavros_with_namespace_blinky,
        node_control_blinky,
        node_ihm_blinky,
    	node_joy,
        node_tracking_blinky,


    ])
    
    
