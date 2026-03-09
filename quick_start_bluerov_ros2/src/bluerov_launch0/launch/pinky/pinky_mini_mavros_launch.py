import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace

from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import AnyLaunchDescriptionSource


# Fonction appelé par ros2 launch pour avoir la liste des nodes à lancer

def generate_launch_description():

    config_pinky = os.path.join(
        get_package_share_directory('bluerov_launch0'),
        'config', # repertoire (dans le dossier "share/(...)" qui a été generé et non dans le "src")
        'parametre_pinky.yaml' # nom du fichier .yaml
    )

    #"""

    launch_mavros_with_namespace = Node(
        package="bluerov_control0",
        namespace='pinky', 
        executable="Mini_MAVROS", 
        name="Mini_MAVROS", 
        output="screen",
        emulate_tty= True,
        parameters=[{
            'adresse_udp': 'udp:192.168.3.100:14550', 
            #'adresse_udp': 'udp:0.0.0.0:14550',
            'tgt_system':'2',
            'tgt_component':'1', # dois toujours rester = à 1
            'system_id':'2', #  SYSID de MAVROS (la station GCS)

             }
        ]

    )



    node_control_pinky = Node(
        package="bluerov_control0", 
        namespace='pinky',
        executable="control_bluerov", 
        name="control_pinky", 
        output="screen",
        emulate_tty= True,
        parameters=[config_pinky]

    )

    #"""

    node_ihm_pinky = Node(
        package="bluerov_ihm0", 
        namespace='pinky',
        executable="ihm", 
        name="ihm_pinky", 
        output="screen",
        emulate_tty= True,
        parameters=[config_pinky]
    )


    node_tracking_pinky_cam1 = Node(
        package="bluerov_tracking0", 
        namespace='pinky',
        executable="tracking_node", 
        name="tracking_pinky_cam1", 
        output="screen",
        emulate_tty= True,
        parameters=[config_pinky]
    )

    node_tracking_pinky_cam2 = Node(
        package="bluerov_tracking0", 
        namespace='pinky',
        executable="tracking_node", 
        name="tracking_pinky_cam2", 
        output="screen",
        emulate_tty= True,
        parameters=[config_pinky]
    )
    
    
    node_affichage_pinky = Node(
        package="bluerov_tracking0", 
        namespace='pinky',
        executable="affichage_node", 
        name="affichage_pinky", 
        output="screen",
        emulate_tty= True,
        parameters=[config_pinky]
    )

    # ADAPTER LE PACKAGE SONAR
    """
    node_ping1d_pinky = Node(
        package="bluerov_sonar0", 
        namespace='pinky',
        executable="ping1d_rustine", 
        name="ping1d_pinky", 
        output="screen",
        emulate_tty= True,
        parameters=[config_pinky]
    )
    #"""
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
        parameters=[config_pinky]
    )


    # retour de la fonction avec la liste des nodes à lancer
    
    return LaunchDescription([
    
    	#launch_mavros,
        launch_mavros_with_namespace,
        node_control_pinky,
        node_ihm_pinky,
    	node_joy,
        node_joy_switch,
        node_tracking_pinky_cam1,
        node_tracking_pinky_cam2,
        node_affichage_pinky ,
        #node_ping1d_pinky,

    ])
    
    
