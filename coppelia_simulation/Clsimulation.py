# import sys
import time
import rclpy
from rclpy.node import Node
# from std_msgs.msg import Float64MultiArray
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import logging
from geometry_msgs.msg import Point
import numpy as np
# import random
from ur5_kinematics import ur5_inverse_kinematics_with_orientation

# Configuration du logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(message)s')
logger = logging.getLogger(__name__)




class MovementSubscriber(Node):
    def __init__(self, sim, script_handle):
        super().__init__('movement_subscriber')
        self.sim = sim
        self.script_handle = script_handle
        self.subscription = self.create_subscription(
            Point,
            'sandbox_positions',
            self.listener_callback,
            10)
        self.subscription

        # Définir les limites de la SandBox
        self.sandbox_min = np.array([0, 0, -0.2])
        self.sandbox_max = np.array([1.05, 0.60, 0.35])

        # Définir la position de base du robot par rapport à la SandBox
        self.robot_base = np.array([0.5, 0.3, 0.3])  # Ajustez ces valeurs selon votre configuration


    def listener_callback(self, msg):
        # World coordinates
        # world_coords = self.sandbox_to_world_coordinates(msg.x, msg.y, msg.z)

        # Orientation: Gripper facing down (rotation around Z-axis)
        gripper_orientation = -np.pi / 4  # 90 degrees downward

        # Compute joint angles with the correct orientation
        # joint_angles = ur5_inverse_kinematics_with_orientation(*world_coords, gripper_orientation)

        try:
            # result = self.sim.callScriptFunction('moveArm', self.script_handle, joint_angles)
            result = self.sim.callScriptFunction('moveToPositionAndCreateDummy', self.script_handle, msg.x, msg.y, msg.z)
            self.get_logger().info(f"Robot moved to position: ({msg.x}, {msg.y}, {msg.z}), result: {result}")
        except Exception as e:
            self.get_logger().error(f"Error during movement execution: {e}")


    def reset_robot_position(self):
        """Function to reset UR5 to its initial position"""
        joint_handles = [
            self.sim.getObject('/UR5/UR5_joint1'),
            self.sim.getObject('/UR5/UR5_joint2'),
            self.sim.getObject('/UR5/UR5_joint3'),
            self.sim.getObject('/UR5/UR5_joint4'),
            self.sim.getObject('/UR5/UR5_joint5'),
            self.sim.getObject('/UR5/UR5_joint6')
        ]

        initial_positions = [0, 0, 0, 0, 0, 0]  # Set the exact initial joint angles
        for joint, initial_position in zip(joint_handles, initial_positions):
            self.sim.setJointTargetPosition(joint, initial_position)
            time.sleep(0.5)  # Gradual reset

        logger.info("Robot reset to initial position.")

    def sandbox_to_world_coordinates(self, x, y, z):
        # Convertir les coordonnées de la SandBox en coordonnées du monde
        sandbox_coords = np.array([x, y, z])
        world_coords = self.robot_base + sandbox_coords

        # S'assurer que les coordonnées sont dans les limites de la SandBox
        world_coords = np.clip(world_coords, self.sandbox_min, self.sandbox_max)

        return world_coords


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
        # user_input = int(input("Entrez une valeur pour i: "))

        # Appeler la fonction test_fx après le démarrage de la simulation
        if script_handle is not None:
            # self.call_test_fx(script_handle, user_input)
            a = 0

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
        # target_positions = [np.pi/2, -np.pi/4, 0, np.pi/2, 3*np.pi/4, np.pi/2]

        # Déplacer chaque joint à sa position cible
        for joint, target_position in zip(joint_handles, target_positions):
            self.sim.setJointTargetPosition(joint, target_position)
            time.sleep(0.8)  # Attendre 1 seconde avant de passer au joint suivant

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

    def reset_robot_position(self, joint_handles):
        initial_positions = [0, 0, 0, 0, 0, 0]  # Adjust these to the initial joint angles
        for joint, initial_position in zip(joint_handles, initial_positions):
            self.sim.setJointTargetPosition(joint, initial_position)
            time.sleep(0.4)  # Ensure gradual movement back to initial position
        logger.info("Robot reset to initial position.")

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
