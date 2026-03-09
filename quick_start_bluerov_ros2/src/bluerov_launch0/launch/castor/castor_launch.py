import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace

from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import AnyLaunchDescriptionSource


# Fonction appelé par ros2 launch pour avoir la liste des nodes à lancer

def generate_launch_description():

    config_castor = os.path.join(
        get_package_share_directory('bluerov_launch0'),
        'config', # repertoire (dans le dossier "share/(...)" qui a été generé et non dans le "src")
        'parametre_castor.yaml' # nom du fichier .yaml
    )

    
    launch_castor = Node(
        package="bluerov_control0",
        namespace='castor', 
        executable="Mini_MAVROS", 
        name="Mini_MAVROS", 
        output="screen",
        emulate_tty= True,
        parameters=[{
            'adresse_udp': 'udp:192.168.21.100:14550', #'adresse_udp': 'udp:192.168.2.1:14550',
            #'adresse_udp': 'udp:0.0.0.0:14550',
            'tgt_system':'21',
            'tgt_component':'1', # dois toujours rester = à 1
            'system_id':'11', #  SYSID de MAVROS (la station GCS)

             }
        ]

    )

    node_control_castor = Node(
        package="bluerov_control0", 
        namespace='castor',
        executable="control_bluerov", 
        name="control_castor", 
        output="screen",
        emulate_tty= True,
        parameters=[config_castor]

    )

    #"""

    node_ihm_castor = Node(
        package="bluerov_ihm0", 
        namespace='castor',
        executable="ihm", 
        name="ihm_castor", 
        output="screen",
        emulate_tty= True,
        parameters=[config_castor]
    )


    node_tracking_castor = Node(
        package="bluerov_tracking0", 
        namespace='castor',
        executable="tracking_node", 
        name="tracking_castor", 
        output="screen",
        emulate_tty= True,
        parameters=[config_castor]
    )


    node_affichage_castor = Node(
        package="bluerov_tracking0", 
        namespace='castor',
        executable="affichage_node", 
        name="affichage_castor", 
        output="screen",
        emulate_tty= True,
        parameters=[config_castor]
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
    
        launch_castor,
        node_control_castor,
        node_ihm_castor,
    	node_joy,
        node_tracking_castor,
        node_affichage_castor,

    ])
    
    
