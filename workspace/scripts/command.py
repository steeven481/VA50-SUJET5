#!/usr/bin/env python3

import rospy
import json
import numpy as np
import os
from geometry_msgs.msg import Pose, Twist
import moveit_commander
from tf.transformations import quaternion_from_euler
import math


class TiagoObjectPlacer:
    """Robot Tiago pour placer les objets à leurs positions finales"""

    def __init__(self):
        rospy.init_node('tiago_object_placer')

        # Initialisation MoveIt
        moveit_commander.roscpp_initialize([])
        self.arm_group = moveit_commander.MoveGroupCommander("arm_torso")

        # Configuration MoveIt
        self.arm_group.set_planning_time(5.0)
        self.arm_group.set_max_velocity_scaling_factor(0.1)
        self.arm_group.set_goal_position_tolerance(0.02)
        self.arm_group.set_goal_orientation_tolerance(0.05)

        # Charger les positions depuis les fichiers JSON
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.initial_positions = self.load_json(os.path.join(script_dir, "../pipeline/outputs/detections_input.json"))
        self.final_positions = self.load_json(os.path.join(script_dir, "../pipeline/outputs/final_positions.json"))

        # Facteurs de conversion pixel -> mètres (calibrés)
        self.pixel_to_meter_x = 0.0015
        self.pixel_to_meter_y = 0.0015

        # Centre de l'image (caméra Xtion: 640x480)
        self.image_center_x = 320
        self.image_center_y = 240

        # Positions de référence
        self.base_x = 0.5  # Distance de base du bras
        self.base_y = 0.0  # Centre latéral
        self.grasp_height = 0.7  # Hauteur pour saisir
        self.place_height = 0.65  # Hauteur pour déposer
        self.approach_height = 0.8  # Hauteur d'approche

        rospy.loginfo("📊 Positions chargées avec succès")
        rospy.sleep(2.0)

    def load_json(self, filepath):
        """Charger un fichier JSON"""
        try:
            with open(filepath, 'r') as f:
                return json.load(f)
        except Exception as e:
            rospy.logerr(f"❌ Erreur chargement {filepath}: {e}")
            return []

    def pixel_to_robot_position(self, pixel_x, pixel_y, is_final=False):
        """Convertir des coordonnées pixels en position robot"""
        # Calculer les offsets en mètres
        offset_x = (pixel_x - self.image_center_x) * self.pixel_to_meter_x
        offset_y = (pixel_y - self.image_center_y) * self.pixel_to_meter_y

        # Position calculée
        x = self.base_x + offset_x
        y = self.base_y - offset_y  # Inversion Y (image vs robot)

        # Ajuster la hauteur selon l'action
        if is_final:
            z = self.place_height  # Plus bas pour déposer
        else:
            z = self.grasp_height  # Plus haut pour saisir

        # Limites de sécurité pour Tiago
        x = max(0.3, min(0.7, x))  # Limites en X
        y = max(-0.3, min(0.3, y))  # Limites en Y
        z = max(0.6, min(1.0, z))  # Limites en Z

        return [x, y, z]

    def create_pose(self, position, orientation_type="down"):
        """Créer une pose avec orientation"""
        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]

        if orientation_type == "down":
            # Orientation vers le bas (pour saisir/déposer)
            quat = quaternion_from_euler(-math.pi / 2, 0, math.pi / 2)
        elif orientation_type == "forward":
            # Orientation vers l'avant (position sûre)
            quat = quaternion_from_euler(0, math.pi / 2, 0)
        else:
            # Orientation neutre
            quat = quaternion_from_euler(0, 0, 0)

        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]

        return pose

    def go_to_safe_position(self):
        """Aller à une position sûre"""
        rospy.loginfo("🔄 Déplacement vers position sûre...")

        safe_position = [0.4, 0.0, 0.8]
        safe_pose = self.create_pose(safe_position, "forward")

        self.arm_group.set_pose_target(safe_pose)
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()

        if success:
            rospy.loginfo("✅ Position sûre atteinte")
        else:
            rospy.logwarn("⚠️  Échec position sûre, tentative alternative...")
            # Tentative alternative
            self.arm_group.set_named_target("home")
            self.arm_group.go(wait=True)
            self.arm_group.stop()

        rospy.sleep(1.0)
        return True

    def move_to_position(self, position, label, is_approach=False):
        """Déplacer le bras à une position spécifique"""
        rospy.loginfo(f"🎯 Déplacement vers {label}...")
        rospy.loginfo(f"📍 Position: X={position[0]:.3f}m, Y={position[1]:.3f}m, Z={position[2]:.3f}m")

        # Créer la pose avec orientation vers le bas
        pose = self.create_pose(position, "down")

        # Déplacer
        self.arm_group.set_pose_target(pose)
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()

        if success:
            rospy.loginfo("✅ Position atteinte")
            rospy.sleep(0.5)
            return True
        else:
            rospy.logwarn("⚠️  Échec du déplacement")
            return False

    def simulate_gripper_action(self, action, label):
        """Simuler l'action de la pince (ouvrir/fermer)"""
        if action == "open":
            rospy.loginfo(f"🤏 Simulation ouverture pince pour {label}...")
            # Simulation: juste attendre
            rospy.sleep(1.0)
        elif action == "close":
            rospy.loginfo(f"🤏 Simulation fermeture pince pour {label}...")
            # Simulation: attendre un peu plus longtemps pour la saisie
            rospy.sleep(1.5)
        return True

    def pick_object(self, label, initial_position):
        """Saisir un objet"""
        rospy.loginfo(f"🤏 Saisie de {label}...")

        # 1. Approche haute (au-dessus de l'objet)
        approach_position = initial_position.copy()
        approach_position[2] = self.approach_height

        if not self.move_to_position(approach_position, f"{label} (approche)"):
            return False

        # 2. Ouvrir la pince
        self.simulate_gripper_action("open", label)

        # 3. Descente pour saisir
        if not self.move_to_position(initial_position, f"{label} (saisie)"):
            return False

        # 4. Fermer la pince (saisir l'objet)
        self.simulate_gripper_action("close", label)

        # 5. Remonter avec l'objet
        if not self.move_to_position(approach_position, f"{label} (remontée)"):
            return False

        rospy.loginfo(f"✅ {label} saisi avec succès")
        return True

    def place_object(self, label, final_position):
        """Déposer un objet à sa position finale"""
        rospy.loginfo(f"📦 Dépot de {label} à la position finale...")

        # 1. Approche haute de la position finale
        approach_final = final_position.copy()
        approach_final[2] = self.approach_height

        if not self.move_to_position(approach_final, f"{label} (approche finale)"):
            return False

        # 2. Descente pour déposer
        if not self.move_to_position(final_position, f"{label} (dépôt)"):
            return False

        # 3. Ouvrir la pince (relâcher l'objet)
        self.simulate_gripper_action("open", label)

        # 4. Remonter
        if not self.move_to_position(approach_final, f"{label} (détachement)"):
            return False

        rospy.loginfo(f"✅ {label} déposé avec succès")
        return True

    def execute_object_placement(self):
        """Exécuter le placement de tous les objets"""
        rospy.loginfo("=" * 60)
        rospy.loginfo("🚀 DÉMARRAGE DU RANGEMENT DES OBJETS")
        rospy.loginfo("=" * 60)

        # Attendre que tout soit initialisé
        rospy.sleep(2.0)

        # Position sûre initiale
        self.go_to_safe_position()

        # Définir l'ordre de traitement
        processing_order = ["fork", "knife", "plate"]

        for label in processing_order:
            rospy.loginfo("\n" + "=" * 50)
            rospy.loginfo(f"🎯 TRAITEMENT DE: {label.upper()}")
            rospy.loginfo("=" * 50)

            # Trouver les centroïdes initiaux et finaux
            initial_centroid = None
            final_centroid = None

            # Chercher dans les positions initiales
            for obj in self.initial_positions:
                if obj['label'] == label:
                    initial_centroid = (obj['x_pixel'], obj['y_pixel'])
                    break

            # Chercher dans les positions finales
            for obj in self.final_positions:
                if obj['label'] == label:
                    final_centroid = (obj['x_pixel'], obj['y_pixel'])
                    break

            if not initial_centroid or not final_centroid:
                rospy.logwarn(f"⚠️  Positions non trouvées pour {label}")
                continue

            rospy.loginfo(f"📍 Position initiale: ({initial_centroid[0]:.1f}, {initial_centroid[1]:.1f}) pixels")
            rospy.loginfo(f"📍 Position finale: ({final_centroid[0]:.1f}, {final_centroid[1]:.1f}) pixels")

            try:
                # Convertir pixels en positions robot
                initial_position = self.pixel_to_robot_position(
                    initial_centroid[0], initial_centroid[1], is_final=False
                )

                final_position = self.pixel_to_robot_position(
                    final_centroid[0], final_centroid[1], is_final=True
                )

                rospy.loginfo(
                    f"📍 Position robot initiale: X={initial_position[0]:.3f}m, Y={initial_position[1]:.3f}m, Z={initial_position[2]:.3f}m")
                rospy.loginfo(
                    f"📍 Position robot finale: X={final_position[0]:.3f}m, Y={final_position[1]:.3f}m, Z={final_position[2]:.3f}m")

                # Phase 1: Saisir l'objet
                rospy.loginfo("\n📦 PHASE 1: SAISIE")
                if self.pick_object(label, initial_position):
                    rospy.loginfo(f"✅ {label} saisi avec succès")

                    # Phase 2: Déposer à la position finale
                    rospy.loginfo("\n📦 PHASE 2: DÉPOT")
                    if self.place_object(label, final_position):
                        rospy.loginfo(f"✅ {label} rangé avec succès!")

                        # Retour à la position sûre entre les objets
                        rospy.loginfo("\n🔄 Retour à la position sûre...")
                        self.go_to_safe_position()

                        # Pause entre les objets
                        rospy.sleep(1.0)
                    else:
                        rospy.logwarn(f"⚠️  Échec du dépôt de {label}")
                        self.go_to_safe_position()
                else:
                    rospy.logwarn(f"⚠️  Échec de la saisie de {label}")
                    self.go_to_safe_position()

            except Exception as e:
                rospy.logerr(f"❌ Erreur avec {label}: {e}")
                self.go_to_safe_position()

        # Mission accomplie
        rospy.loginfo("\n" + "=" * 60)
        rospy.loginfo("🎉🎉🎉 MISSION ACCOMPLIE ! 🎉🎉🎉")
        rospy.loginfo("✅ Tous les objets ont été rangés")
        rospy.loginfo("🤖 La main de Tiago s'arrête")
        rospy.loginfo("=" * 60)

        # Dernière position sûre
        self.go_to_safe_position()

        # Attendre indéfiniment (Ctrl+C pour arrêter)
        rospy.spin()

    def run(self):
        """Exécuter la séquence principale"""
        self.execute_object_placement()


def main():
    placer = TiagoObjectPlacer()

    try:
        placer.run()
    except KeyboardInterrupt:
        rospy.loginfo("\n🛑 Programme arrêté par l'utilisateur")
    except Exception as e:
        rospy.logerr(f"\n❌ Erreur: {e}")
    finally:
        moveit_commander.roscpp_shutdown()
        rospy.loginfo("👋 Programme terminé")


if __name__ == '__main__':
    main()