import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace

from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import AnyLaunchDescriptionSource

from launch.launch_description_sources import PythonLaunchDescriptionSource

# Fonction appelé par ros2 launch pour avoir la liste des nodes à lancer


    
def lire_noms(fichier_txt):
    liste_noms = []
    with open(fichier_txt, "r") as f:
        for ligne in f:
            ligne = ligne.strip()              # enlever espaces / retour ligne
            if not ligne:                      # ignorer lignes vides
                continue
            if ligne.startswith("#"):          # ignorer commentaires
                continue
            liste_noms.append(ligne)           # ajouter le nom
            
    return liste_noms 


def generate_launch_description():

    
    config = os.path.join(
        get_package_share_directory('bluerov_launch0'),
        'config', # repertoire (dans le dossier "share/(...)" qui a été generé et non dans le "src")
        'parametre_multi_rovs.yaml' # nom du fichier .yaml
    )


    ######## Launch des nodes ROVs #########

    # list des ROVs que l'on veut utiliser
    """
    list_ROV = [
        #'inky',
        #'blinky',
        'pinky',
        #'polux',
    ]
    """
    ROOT_DIR = os.path.expanduser('~') # chemin vers le fichier text a lire
    filename =  ROOT_DIR + "/quick_start_bluerov_ros2/src/bluerov_launch0/launch/multi/" 
                            
    list_ROV = lire_noms(filename+'list_ROV.txt')

        
    ########### Launch des ROVs ###################

    nb_ROV = len(list_ROV)  # nombre de ROVs
    num_ROV = 0.0
    list_launch = []
    list_name_ROV = {} # liste des noms de ROV pour les affichages IHM et video


    for i in range(nb_ROV):

        nom_rov = list_ROV[i]

        if not(nom_rov == 'affichage_multi') :

            print("rov used = ", nom_rov)
            num_ROV = num_ROV+1.0
            name = {'ROV_name_'+str(int(num_ROV)):nom_rov}
            list_name_ROV.update(name) # on ajoute le nom a la liste

            rov_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('bluerov_launch0'),
                        'launch',
                        'launch_'+nom_rov+'_multi.py'           # <-- ici le nom exact de ton launch
                    )
                ),
                # arguments optionnels si ton launch en accepte
                launch_arguments={'rov_id': str(num_ROV), #1.0
                                'nb_ROV' : str(nb_ROV)
                                }.items()  # Numero du ROV : A mettre à jour en fonction des ROVs utilise
            )

            list_launch.append(rov_launch)

    # on recupere aussi une variable pour afficher toutes les cam. en miniature ou non
    if 'affichage_multi' in list_ROV:
        affichage_multi = 1
    else:
        affichage_multi = 0
    nb_ROV = nb_ROV-affichage_multi # on retire la variable en trop
    

    ##########################################
    ######## Nodes commun #######
    list_name_ROV.update({"nb_ROV":  float(nb_ROV)})
    list_name_ROV.update({"affichage_multi":  float(affichage_multi)})
    print("list_name_ROV = ", list_name_ROV)


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
        parameters=[config, {"nb_ROV":  float(nb_ROV)}]
    )

    node_ihm_multi = Node(
        package="bluerov_ihm0", 
        executable="multi_ihm_node", 
        name="ihm_general", 
        output="screen",
        emulate_tty= True,
        parameters=[list_name_ROV]
    )
    node_affichage_multi_light = Node(
        package="bluerov_tracking0", 
        executable="multi_affichage_light_node", 
        name="affichage_general", 
        output="screen",
        emulate_tty= True,
        parameters=[config,list_name_ROV]
    )


    # retour de la fonction avec la liste des nodes à lancer
    
    return LaunchDescription(
        list_launch +
        [
    	node_joy,
        node_joy_switch,
        
        node_affichage_multi_light,
        node_ihm_multi,
        
    ])
    
    
