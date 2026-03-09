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


    launch_mavros_with_namespace_blinky = Node(
        package="bluerov_control0",
        namespace='blinky', 
        executable="Mini_MAVROS", 
        name="Mini_MAVROS", 
        output="screen",
        emulate_tty= True,
        parameters=[{
            'adresse_udp': 'udp:192.168.4.100:14550', 
            #'adresse_udp': 'udp:0.0.0.0:14550',
            'tgt_system':'3',
            'tgt_component':'1', # dois toujours rester = à 1
            'system_id':'3', #  SYSID de MAVROS (la station GCS)

             }
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


    node_affichage_blinky = Node(
        package="bluerov_tracking0", 
        namespace='blinky',
        executable="affichage_node", 
        name="affichage_blinky", 
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


    node_joy_switch = Node(
        package="bluerov_add", 
        executable="joy_switch", 
        name="joy_switch", 
        output="screen",
        parameters=[config_blinky]
    )


    # retour de la fonction avec la liste des nodes à lancer
    
    return LaunchDescription([
    
    	#launch_mavros_blinky,
        launch_mavros_with_namespace_blinky,
        node_control_blinky,
        node_ihm_blinky,
    	node_joy,
        node_joy_switch,
        node_tracking_blinky,
        node_affichage_blinky ,

    ])
    
    
