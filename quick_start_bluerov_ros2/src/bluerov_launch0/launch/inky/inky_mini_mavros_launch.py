import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace

from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import AnyLaunchDescriptionSource


# Function called by ros2 launch to get the list of nodes to launch

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('bluerov_launch0'),
        'config', 
        'parametre_inky.yaml' # nom du fichier .yaml
    )

    #"""


    launch_mavros = Node(
        package="bluerov_control0",
        namespace='inky', 
        executable="Mini_MAVROS", 
        name="Mini_MAVROS", 
        output="screen",
        emulate_tty= True,
        parameters=[{
            'adresse_udp': 'udp:192.168.2.1:14550', 
            #'adresse_udp': 'udp:0.0.0.0:14550',
            'tgt_system':'1',
            'tgt_component':'1', # dois toujours rester = à 1
            'system_id':'1', #  SYSID de MAVROS (la station GCS)

             }
        ]

    )




    node_control_inky = Node(
        package="bluerov_control0", 
        namespace='inky',
        executable="control_bluerov", 
        name="control_inky", 
        output="screen",
        emulate_tty= True,
        parameters=[config]

    )

    #"""

    node_ihm_inky = Node(
        package="bluerov_ihm0", 
        namespace='inky',
        executable="ihm", 
        name="ihm_inky", 
        output="screen",
        emulate_tty= True,
        parameters=[config]
    )


    node_tracking_inky = Node(
        package="bluerov_tracking0", 
        namespace='inky',
        executable="tracking_node", 
        name="tracking_inky", 
        output="screen",
        emulate_tty= True,
        parameters=[config]
    )


    node_affichage_inky = Node(
        package="bluerov_tracking0", 
        namespace='inky',
        executable="affichage_node", 
        name="affichage_inky", 
        output="screen",
        emulate_tty= True,
        parameters=[config]
    )

    """ ADAPTER LE PACKAGE
    node_ping1d_inky = Node(
        package="bluerov_sonar2", 
        namespace='inky',
        executable="ping1d", 
        name="ping1d_inky", 
        output="screen",
        emulate_tty= True,
        parameters=[config]
    )
    """
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
        parameters=[config]
    )


    # retour de la fonction avec la liste des nodes à lancer
    
    return LaunchDescription([
    
    	launch_mavros,
        #launch_mavros_with_namespace,
        node_control_inky,
        node_ihm_inky,
    	node_joy,
        node_joy_switch,
        node_tracking_inky,
        node_affichage_inky ,
        #node_ping1d_inky,

    ])
    
    
