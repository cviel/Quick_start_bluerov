import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace

from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import AnyLaunchDescriptionSource

from launch.substitutions import LaunchConfiguration

from launch_ros.parameter_descriptions import ParameterValue


# Fonction appelé par ros2 launch pour avoir la liste des nodes à lancer

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('bluerov_launch0'),
        'config', # repertoire (dans le dossier "share/(...)" qui a été generé et non dans le "src")
        'parametre_multi_rovs.yaml' # nom du fichier .yaml
    )

    rov_id = LaunchConfiguration('rov_id') # on va charger une variable dans le launch parent (nous sommes dans un launch enfant)
    nb_ROV = LaunchConfiguration('nb_ROV') # on va charger une variable dans le launch parent (nous sommes dans un launch enfant)


    ######## Nodes pinky #######
    

    launch_mavros_with_namespace_pinky = Node(
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
        parameters=[config, {
            "ID": ParameterValue(rov_id, value_type=float),
            "nb_ROV": ParameterValue(nb_ROV, value_type=float),
        }]

    )

    node_tracking_pinky_cam1 = Node(
        package="bluerov_tracking0", 
        namespace='pinky',
        executable="tracking_node", 
        name="tracking_pinky_cam1", 
        output="screen",
        emulate_tty= True,
        parameters=[config, {
            "ID": ParameterValue(rov_id, value_type=float),
            "nb_ROV": ParameterValue(nb_ROV, value_type=float),
        }]
    )

    node_tracking_pinky_cam2 = Node(
        package="bluerov_tracking0", 
        namespace='pinky',
        executable="tracking_node", 
        name="tracking_pinky_cam2", 
        output="screen",
        emulate_tty= True,
        parameters=[config, {
            "ID": ParameterValue(rov_id, value_type=float),
            "nb_ROV": ParameterValue(nb_ROV, value_type=float),
        }]
    )
    


    # retour de la fonction avec la liste des nodes à lancer
    
    return LaunchDescription([
    
    	launch_mavros_with_namespace_pinky,
        node_control_pinky,
        node_tracking_pinky_cam1,
        node_tracking_pinky_cam2

    ])
    
    
