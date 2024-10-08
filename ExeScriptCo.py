#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import sys
import os
import time
import logging

try:
    from zm_coppeliasim_remote_api import client as zc
except ImportError:
    print("Erreur : Impossible d'importer zm_coppeliasim_remote_api. Assurez-vous qu'il est installé.")
    sys.exit(1)

# Configuration du logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class CoppeliaSimNode(Node):
    def __init__(self):
        super().__init__('coppeliasim_node')
        self.client = None
        
    def connect_to_coppeliasim(self):
        try:
            self.client = zc.RemoteAPIClient()
            logger.info("Connexion à CoppeliaSim établie avec succès.")
        except Exception as e:
            logger.error(f"Erreur lors de la connexion à CoppeliaSim : {e}")
            sys.exit(1)

    def load_scene(self, scene_path):
        try:
            self.client.sim.loadScene(scene_path)
            logger.info(f"Scène '{scene_path}' chargée avec succès.")
        except Exception as e:
            logger.error(f"Erreur lors du chargement de la scène : {e}")
            sys.exit(1)

    def start_simulation(self):
        try:
            self.client.sim.startSimulation()
            logger.info("Simulation démarrée.")
        except Exception as e:
            logger.error(f"Erreur lors du démarrage de la simulation : {e}")
            sys.exit(1)

    def stop_simulation(self):
        try:
            self.client.sim.stopSimulation()
            logger.info("Simulation arrêtée.")
        except Exception as e:
            logger.error(f"Erreur lors de l'arrêt de la simulation : {e}")

    def execute_script(self, script_name):
        try:
            script_handle = self.client.sim.getScriptHandle(script_name)
            if script_handle != -1:
                self.client.sim.callScriptFunction("sysCall_init", script_handle)
                logger.info(f"Script '{script_name}' exécuté avec succès.")
            else:
                logger.error(f"Script '{script_name}' non trouvé dans la scène.")
        except Exception as e:
            logger.error(f"Erreur lors de l'exécution du script : {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CoppeliaSimNode()

    try:
        # Connexion à CoppeliaSim
        node.connect_to_coppeliasim()

        # Chargement de la scène
        scene_path = os.path.join(os.getcwd(), "scenes", "Environment4BinPicking.ttt")
        node.load_scene(scene_path)

        # Démarrage de la simulation
        node.start_simulation()

        # Exécution du script
        node.execute_script("Script_1")

        # Attente de quelques secondes pour permettre à la simulation de s'exécuter
        time.sleep(10)

    except Exception as e:
        logger.error(f"Une erreur est survenue : {e}")

    finally:
        # Arrêt de la simulation
        node.stop_simulation()

        # Fermeture de la connexion
        if node.client:
            node.client.close()
            logger.info("Connexion à CoppeliaSim fermée.")

        # Arrêt du nœud ROS
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
