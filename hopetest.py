import time
import sys

# Ajouter le chemin d'accès à l'API ZMQ de CoppeliaSim
sys.path.append('/opt/CoppeliaSim_Edu_V4_7_0_rev4_Ubuntu22_04/programming/zmqRemoteApi/python')
import coppeliasim_zmqremoteapi_client as zmqRemoteApi

def log_message(sim, message):
    """Fonction pour envoyer un message à la barre d'état de CoppeliaSim."""
    sim.addStatusbarMessage(message)
    print(f"Log: {message}")  # Affiche le message dans la console du terminal

def list_scripts(sim):
    """Fonction pour lister tous les scripts dans la scène."""
    # Obtenir tous les objets de type script
    script_handles = sim.getObjects(sim.sim_get_object_type('Script'))  # Corrected here

    scripts = {}
    for handle in script_handles:
        name = sim.getObjectName(handle)
        scripts[handle] = name

    return scripts

def main():
    # Connexion à CoppeliaSim via l'API ZMQ
    client = zmqRemoteApi.RemoteAPIClient()
    sim = client.getObject('sim')

    # Démarrer la simulation
    sim.startSimulation()

    # Enregistrer un message de démarrage dans les logs
    log_message(sim, "Robot UR5 est prêt à être manipulé.")

    # Obtenir la poignée du robot UR5
    robot_handle = sim.getObject('/UR5')

    # Obtenir les poignées des joints
    joint_handles = [
        sim.getObject('/UR5/UR5_joint1'),
        sim.getObject('/UR5/UR5_joint2'),
        sim.getObject('/UR5/UR5_joint3'),
        sim.getObject('/UR5/UR5_joint4'),
        sim.getObject('/UR5/UR5_joint5'),
        sim.getObject('/UR5/UR5_joint6')
    ]

    # Positions cibles pour chaque joint
    target_positions = [0.5, -0.5, 0.3, -0.3, 0.2, -0.2]

    # Déplacer chaque joint à sa position cible
    for joint, target_position in zip(joint_handles, target_positions):
        sim.setJointTargetPosition(joint, target_position)
        time.sleep(1)  # Attendre 1 seconde avant de passer au joint suivant

    # Ramener chaque joint à sa position initiale (supposons qu'elle est à 0)
    initial_positions = [0, 0, 0, 0, 0, 0]

    for joint, initial_position in zip(joint_handles, initial_positions):
        sim.setJointTargetPosition(joint, initial_position)
        time.sleep(0.4)  # Attendre 1 seconde avant de passer au joint suivant

    # Arrêter la simulation
    sim.stopSimulation()

    log_message(sim, "Manipulation du robot terminée et retour aux positions initiales.")

    # Lancer le script spécifique créé dans la scène
    script_handle = sim.getObject('/Script_1_')  # Remplacez par le nom réel du script
    script_functions = client.getScriptFunctions(script_handle)
    script_functions.runSelf()  # Exécutez le script
    log_message(sim, f"Le script '{sim.getObjectAlias(script_handle)}' a été exécuté avec succès.")



if __name__ == "__main__":
    main()
