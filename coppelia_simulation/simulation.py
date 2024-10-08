# import sys
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import logging

# Configuration du logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(message)s')
logger = logging.getLogger(__name__)


class MovementSubscriber(Node):
    def __init__(self, sim, script_handle):
        super().__init__('movement_subscriber')
        self.sim = sim
        self.script_handle = script_handle
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'movement_commands',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        # Assume msg contains the coordinates [x, y, z]
        coordinates = list(msg.data)  # Convert to Python list
        logging.info(f"Received target coordinates: {coordinates}")

        try:
            # Convert coordinates (x, y, z) to joint angles using inverse kinematics
            joint_angles = self.calculate_inverse_kinematics(coordinates)
            result = self.sim.callScriptFunction('moveArm', self.script_handle, joint_angles)
            logging.info(f"Robot moved to position: {coordinates}, result: {result}")
        except Exception as e:
            logging.error(f"Error during movement execution: {e}")

    def calculate_inverse_kinematics(self, coordinates):
        x, y, z = coordinates
        # Logique simplifiée pour la cinématique inverse (ajuste ceci selon ton robot)
        joint1 = x
        joint2 = y
        joint3 = z
        joint4 = 0  # Ajoute plus si nécessaire
        joint5 = 0
        joint6 = 0
        return [joint1, joint2, joint3, joint4, joint5, joint6]


'''All functions related to CoppeliaSim manipuation'''
class CoppeliaSimulation:
    def __init__(self):
        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')

        logger.info("Connexion à Coppelia établie")

    def run_simulation(self):
        # Charger la scène
        scene_path = '/home/ken/Documents/BinPicking/Environment4BinPicking.ttt'
        self.sim.loadScene(scene_path)
        logger.info(f"Scène chargée : {scene_path}")

        # Trouver le script
        script_name = '/Script0'
        script_handle = self.find_script_by_name(script_name)

        if script_handle is not None:
            logger.info(f"Script trouvé : {script_name}")

        else:
            logger.error(f"Script {script_name} non trouvé")

        # Démarrer la simulation
        self.sim.startSimulation()
        logger.info("Simulation démarrée")

        # Attendre que la simulation démarre complètement
        time.sleep(1)

        # Manipuler le robot après le démarrage de la simulation
        self.manipulate_robot()

        # Prendre la valeur de i via un prompt utilisateur
        user_input = int(input("Entrez une valeur pour i: "))

        # Appeler la fonction test_fx après le démarrage de la simulation
        if script_handle is not None:
            self.call_test_fx(script_handle, user_input)

        # Démarrer ROS2 pour écouter sur le topic des commandes de mouvement
        rclpy.init()
        subscriber = MovementSubscriber(self.sim, script_handle)
        try:
            rclpy.spin(subscriber)  # Ecouter le topic ROS2
        except KeyboardInterrupt:
            logger.info("Interruption ROS2 Node")
        finally:
            subscriber.destroy_node()
            rclpy.shutdown()

        # Arrêter la simulation
        self.stop_simulation()

    def manipulate_robot(self):
        """Fonction pour manipuler le robot UR5"""
        logger.info("Robot UR5 est prêt à être manipulé.")

        # Obtenir la poignée du robot UR5
        #robot_handle = self.sim.getObject('/UR5')

        # Obtenir les poignées des joints
        joint_handles = [
            self.sim.getObject('/UR5/UR5_joint1'),
            self.sim.getObject('/UR5/UR5_joint2'),
            self.sim.getObject('/UR5/UR5_joint3'),
            self.sim.getObject('/UR5/UR5_joint4'),
            self.sim.getObject('/UR5/UR5_joint5'),
            self.sim.getObject('/UR5/UR5_joint6')
        ]

        # Positions cibles pour chaque joint
        target_positions = [0.5, -0.5, 0.3, -0.3, 0.2, -0.2]

        # Déplacer chaque joint à sa position cible
        for joint, target_position in zip(joint_handles, target_positions):
            self.sim.setJointTargetPosition(joint, target_position)
            time.sleep(1)  # Attendre 1 seconde avant de passer au joint suivant

        # Ramener chaque joint à sa position initiale
        initial_positions = [0, 0, 0, 0, 0, 0]

        for joint, initial_position in zip(joint_handles, initial_positions):
            self.sim.setJointTargetPosition(joint, initial_position)
            time.sleep(0.4)  # Attendre 0.4 seconde avant de passer au joint suivant

        logger.info("Manipulation du robot terminée et retour aux positions initiales.")

    def get_robot_pose(self):
        try:
            result = self.sim.callScriptFunction('getRobotPose', self.script_handle)
            position = result[0]  # Position: [x, y, z]
            orientation = result[1]  # Quaternion: [qx, qy, qz, qw]
            logging.info(f"Robot position: {position}, orientation: {orientation}")
            return position, orientation
        except Exception as e:
            logging.error(f"Error getting robot pose: {e}")

    def call_test_fx(self, script_handle, i):
        """Appelle la fonction test_fx dans le script CoppeliaSim"""
        try:
            result = self.sim.callScriptFunction('test_fx', script_handle, [i])
            logger.info(f"Appel de test_fx avec i = {i}, résultat = {result}")
        except Exception as e:
            logger.error(f"Erreur lors de l'appel de test_fx: {e}")

    def find_script_by_name(self, script_name):
        """Trouver le handle du script"""
        try:
            script_handle = self.sim.getScript(self.sim.scripttype_childscript, script_name)
            if script_handle != -1:
                return script_handle
        except Exception as e:
            logger.error(f"Erreur lors de la recherche du script {script_name}: {e}")
        return None

    def stop_simulation(self):
        """Arrêter la simulation"""
        try:
            if self.sim.getSimulationState() != self.sim.simulation_stopped:
                self.sim.stopSimulation()
                logger.info("Simulation arrêtée")
            else:
                logger.info("La simulation était déjà arrêtée")
        except Exception as e:
            logger.error(f"Erreur lors de l'arrêt de la simulation : {e}")


def main():
    coppelia_sim = CoppeliaSimulation()

    try:
        coppelia_sim.run_simulation()
    except Exception as e:
        logger.error(f"Erreur lors de l'exécution de la simulation : {e}")
    finally:
        coppelia_sim.stop_simulation()


if __name__ == '__main__':
    main()
