#!/usr/bin/env python3

import rospy
import json
import numpy as np
import cv2
import os
from geometry_msgs.msg import Pose, Twist, PoseStamped
import moveit_commander
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import quaternion_from_euler, quaternion_multiply
import math
from geometry_msgs.msg import PointStamped
import tf


class TiagoPositionWithDepth:
    def __init__(self, target_object="plate", json_path="../pipeline/outputs/detections_input.json"):
        rospy.init_node('tiago_position_with_depth')

        # Initialisation MoveIt
        moveit_commander.roscpp_initialize([])
        self.arm_group = moveit_commander.MoveGroupCommander("arm_torso")
        self.arm_group.set_planning_time(15.0)
        self.arm_group.set_max_velocity_scaling_factor(0.1)
        self.arm_group.set_goal_position_tolerance(0.02)
        self.arm_group.set_goal_orientation_tolerance(0.05)

        # Configuration TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Publisher pour la base mobile
        self.cmd_vel_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)

        # Paramètres
        self.target_object = target_object
        self.detections_path = json_path

        # Dossiers de données
        self.data_path = os.path.join(os.path.expanduser('~'), 'ros_ws', 'data')
        self.depth_path = os.path.join(self.data_path, 'depth_images')
        self.calib_path = os.path.join(self.data_path, 'calibration')

        # Charger les données
        self.detections = self.load_detections()
        self.camera_calib = self.load_camera_calibration()
        self.depth_image = self.load_latest_depth()

        rospy.loginfo(f"🎯 Cible: {target_object}")
        rospy.loginfo(f"📁 {len(self.detections)} objets détectés")

        rospy.sleep(2.0)

    def load_detections(self):
        """Charger les détections depuis le JSON"""
        with open(self.detections_path, 'r') as f:
            return json.load(f)

    def load_camera_calibration(self):
        """Charger la calibration de la caméra"""
        calib_file = os.path.join(self.calib_path, 'camera_calibration.json')
        if os.path.exists(calib_file):
            with open(calib_file, 'r') as f:
                return json.load(f)
        else:
            return {
                'fx': 525.0,
                'fy': 525.0,
                'cx': 320.0,
                'cy': 240.0,
                'width': 640,
                'height': 480
            }

    def load_latest_depth(self):
        """Charger la dernière image de profondeur sauvegardée"""
        if not os.path.exists(self.depth_path):
            rospy.logerr(f"❌ Dossier profondeur non trouvé: {self.depth_path}")
            return None

        npz_files = [f for f in os.listdir(self.depth_path) if f.endswith('.npz')]
        if not npz_files:
            rospy.logerr("❌ Aucune image de profondeur trouvée")
            return None

        latest_file = max(npz_files, key=lambda f: os.path.getmtime(os.path.join(self.depth_path, f)))
        npz_path = os.path.join(self.depth_path, latest_file)

        try:
            data = np.load(npz_path)
            depth_image = data['depth']
            rospy.loginfo(f"✅ Profondeur chargée: {latest_file} ({depth_image.shape})")
            return depth_image
        except Exception as e:
            rospy.logerr(f"❌ Erreur chargement profondeur: {e}")
            return None

    def find_target(self):
        """Trouver l'objet cible dans les détections"""
        for obj in self.detections:
            if obj['label'] == self.target_object:
                return obj
        return self.detections[0]

    def get_depth_at_pixel(self, pixel_x, pixel_y):
        """Obtenir la profondeur à un pixel donné"""
        if self.depth_image is None:
            rospy.logwarn("⚠️  Aucune image de profondeur chargée")
            return None

        x = int(pixel_x)
        y = int(pixel_y)

        if 0 <= x < self.depth_image.shape[1] and 0 <= y < self.depth_image.shape[0]:
            depth = self.depth_image[y, x]

            if depth > 0 and not np.isnan(depth):
                # Moyenne locale pour plus de robustesse
                y_min = max(0, y - 3)
                y_max = min(self.depth_image.shape[0], y + 3)
                x_min = max(0, x - 3)
                x_max = min(self.depth_image.shape[1], x + 3)

                neighborhood = self.depth_image[y_min:y_max, x_min:x_max]
                valid_depths = neighborhood[neighborhood > 0]

                if len(valid_depths) > 0:
                    return np.median(valid_depths)
                else:
                    return depth
            else:
                rospy.logwarn(f"⚠️  Profondeur invalide à ({x}, {y}): {depth}")
                return None
        else:
            rospy.logerr(f"❌ Coordonnées hors limites: ({x}, {y})")
            return None

    def pixel_to_3d_camera(self, pixel_x, pixel_y, depth):
        """Convertir pixel + profondeur -> 3D dans le repère caméra"""
        if depth is None:
            return None

        fx = self.camera_calib['fx']
        fy = self.camera_calib['fy']
        cx = self.camera_calib['cx']
        cy = self.camera_calib['cy']

        # Conversion standard
        x = (pixel_x - cx) * depth / fx
        y = (pixel_y - cy) * depth / fy
        z = depth

        return (x, y, z)

    def transform_to_robot_base(self, point_camera):
        """Transformer un point du repère caméra au repère base du robot"""
        if point_camera is None:
            return None

        # Créer un PointStamped dans le repère caméra
        point_stamped = PointStamped()
        point_stamped.header.frame_id = "xtion_rgb_optical_frame"
        point_stamped.header.stamp = rospy.Time.now()
        point_stamped.point.x = point_camera[0]
        point_stamped.point.y = point_camera[1]
        point_stamped.point.z = point_camera[2]

        try:
            # Transformation vers base_footprint
            transform = self.tf_buffer.lookup_transform(
                "base_footprint",
                "xtion_rgb_optical_frame",
                rospy.Time(0),
                rospy.Duration(5.0)
            )

            point_base = tf2_geometry_msgs.do_transform_point(point_stamped, transform)
            return point_base.point

        except Exception as e:
            rospy.logerr(f"❌ Erreur TF: {e}")
            return None

    def calculate_robot_position(self):
        """Calculer la position robot en utilisant la profondeur"""
        target_obj = self.find_target()
        rospy.loginfo(f"🎯 Traitement: {target_obj['label']}")

        # Obtenir la profondeur au pixel
        depth = self.get_depth_at_pixel(target_obj['x_pixel'], target_obj['y_pixel'])

        if depth is None:
            rospy.logwarn("⚠️  Utilisation de la profondeur par défaut")
            depth = 1.0  # Valeur par défaut en mètres

        rospy.loginfo(f"📏 Profondeur mesurée: {depth:.3f} m")

        # Conversion pixel -> 3D caméra
        point_camera = self.pixel_to_3d_camera(
            target_obj['x_pixel'],
            target_obj['y_pixel'],
            depth
        )

        if point_camera:
            rospy.loginfo(
                f"📍 Coordonnées caméra: ({point_camera[0]:.3f}, {point_camera[1]:.3f}, {point_camera[2]:.3f})")

            # Transformation vers base robot
            point_base = self.transform_to_robot_base(point_camera)

            if point_base:
                rospy.loginfo(f"📍 Coordonnées robot: ({point_base.x:.3f}, {point_base.y:.3f}, {point_base.z:.3f})")

                # Retourner la position (15cm au-dessus de l'objet)
                z_offset = 0.15
                return [point_base.x, point_base.y, point_base.z + z_offset]

        # Fallback: conversion sans profondeur
        rospy.logwarn("⚠️  Utilisation de la méthode de conversion sans profondeur")
        return self.pixel_to_robot_fallback(target_obj['x_pixel'], target_obj['y_pixel'], target_obj['label'])

    def pixel_to_robot_fallback(self, pixel_x, pixel_y, label):
        """Conversion de secours sans profondeur"""
        pixel_to_meter = 0.0015
        center_x, center_y = 320, 240

        base_x, base_y, base_z = 0.6, 0.0, 0.8

        offset_x = (pixel_x - center_x) * pixel_to_meter
        offset_y = (pixel_y - center_y) * pixel_to_meter

        x = base_x + offset_x
        y = base_y + offset_y

        # Ajustement basé sur le type d'objet
        if label == "plate":
            z = base_z
        elif label == "knife":
            z = base_z - 0.05
        else:  # fork ou autre
            z = base_z - 0.03

        # Limites de sécurité strictes pour Tiago
        x = max(0.4, min(0.75, x))  # X limité à 0.75 max
        y = max(-0.3, min(0.3, y))  # Y limité
        z = max(0.6, min(1.0, z))  # Z limité

        return [x, y, z]

    def create_pose(self, position, orientation_type="down"):
        """Créer une pose avec différentes orientations possibles"""
        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]

        if orientation_type == "down":
            # Orientation vers le bas (pour saisir)
            quat = quaternion_from_euler(-math.pi / 2, 0, math.pi / 2)
        elif orientation_type == "forward":
            # Orientation vers l'avant (pour approche)
            quat = quaternion_from_euler(0, math.pi / 2, 0)
        elif orientation_type == "side":
            # Orientation sur le côté
            quat = quaternion_from_euler(0, 0, math.pi / 2)
        else:
            # Orientation par défaut
            quat = quaternion_from_euler(0, 0, 0)

        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]

        return pose

    def go_to_safe_position(self):
        """Aller à une position sûre et atteignable"""
        rospy.loginfo("🔄 Déplacement vers position sûre...")

        # Essayer d'abord une position simple
        try:
            safe_pose = self.create_pose([0.4, 0.0, 0.7], "forward")
            self.arm_group.set_pose_target(safe_pose)
            success = self.arm_group.go(wait=True)
            self.arm_group.stop()

            if success:
                rospy.loginfo("✅ Position sûre atteinte")
                rospy.sleep(1.0)
                return True

        except Exception as e:
            rospy.logwarn(f"⚠️  Échec position sûre: {e}")

        return False

    def lift_arm_for_movement(self):
        """Lever le bras modérément pour éviter la table"""
        rospy.loginfo("⬆️  Levage modéré du bras...")

        # Position modérée (pas trop haute)
        lift_pose = self.create_pose([0.4, 0.0, 0.75], "forward")

        self.arm_group.set_pose_target(lift_pose)
        self.arm_group.set_max_velocity_scaling_factor(0.05)  # Très lent
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.set_max_velocity_scaling_factor(0.1)  # Restaurer

        if success:
            rospy.loginfo("✅ Bras levé")
            rospy.sleep(1.0)
            return True
        else:
            rospy.logwarn("⚠️  Échec levage, continuation prudemment")
            return False

    def move_robot_toward_target(self, target_distance_x):
        """Déplacer le robot vers la cible"""
        rospy.loginfo("🚗 Déplacement robot...")

        # Calculer la distance à parcourir
        current_distance = target_distance_x
        desired_distance = 0.65  # Distance optimale
        distance_to_move = current_distance - desired_distance

        # Limiter le déplacement
        if distance_to_move <= 0.1:
            rospy.loginfo("✅ Déjà assez proche")
            return 0.0

        if distance_to_move > 0.4:
            distance_to_move = 0.4
            rospy.loginfo(f"⚠️  Déplacement limité à {distance_to_move:.3f}m")

        rospy.loginfo(f"📏 Distance à parcourir: {distance_to_move:.3f}m")

        # Déplacement très lent
        speed = 0.04
        move_time = distance_to_move / speed

        twist = Twist()
        twist.linear.x = speed

        start_time = rospy.Time.now().to_sec()
        rate = rospy.Rate(10)

        while (rospy.Time.now().to_sec() - start_time) < move_time:
            if rospy.is_shutdown():
                break
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

        # Arrêt progressif
        twist.linear.x = 0.0
        for _ in range(5):
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

        rospy.sleep(2.0)  # Attente stabilisation
        rospy.loginfo("✅ Déplacement terminé")

        return distance_to_move

    def plan_and_execute_pose(self, position, orientation_type="down", max_attempts=3):
        """Essayer de planifier et exécuter avec plusieurs tentatives"""
        pose = self.create_pose(position, orientation_type)

        for attempt in range(max_attempts):
            rospy.loginfo(f"🔄 Tentative {attempt + 1}/{max_attempts}")

            # Essayer différentes orientations si nécessaire
            if attempt == 1:
                pose = self.create_pose(position, "forward")
            elif attempt == 2:
                pose = self.create_pose(position, "side")

            self.arm_group.set_pose_target(pose)

            # Essayer de planifier
            plan = self.arm_group.plan()

            if plan[0]:
                rospy.loginfo("✅ Plan trouvé, exécution...")
                success = self.arm_group.execute(plan[1], wait=True)
                self.arm_group.stop()

                if success:
                    rospy.loginfo("✅ Pose atteinte")
                    rospy.sleep(1.0)
                    return True
                else:
                    rospy.logwarn("⚠️  Échec exécution du plan")
            else:
                rospy.logwarn("⚠️  Aucun plan trouvé")

        return False

    def approach_target_safely(self, target_position):
        """Approcher la cible de manière sûre et progressive"""
        rospy.loginfo("🎯 Approche sûre de la cible...")

        # 1. Position haute d'approche
        high_approach = target_position.copy()
        high_approach[2] = max(target_position[2] + 0.2, 0.9)  # 20cm au-dessus
        high_approach[0] = min(high_approach[0], 0.75)  # Limiter X

        rospy.loginfo(f"📍 Approche haute: z={high_approach[2]:.3f}m")

        if not self.plan_and_execute_pose(high_approach, "forward"):
            rospy.logwarn("⚠️  Échec approche haute, tentative directe...")
            # Essayer directement la position finale
            return self.plan_and_execute_pose(target_position, "down")

        rospy.sleep(1.0)

        # 2. Position intermédiaire
        mid_approach = target_position.copy()
        mid_approach[2] = max(target_position[2] + 0.1, 0.8)  # 10cm au-dessus

        rospy.loginfo(f"📍 Approche intermédiaire: z={mid_approach[2]:.3f}m")

        if not self.plan_and_execute_pose(mid_approach, "forward"):
            rospy.logwarn("⚠️  Échec approche intermédiaire")

        rospy.sleep(0.5)

        # 3. Position finale (très lentement)
        rospy.loginfo(f"📍 Descente finale: z={target_position[2]:.3f}m")
        self.arm_group.set_max_velocity_scaling_factor(0.05)
        success = self.plan_and_execute_pose(target_position, "down")
        self.arm_group.set_max_velocity_scaling_factor(0.1)

        return success

    def run(self):
        """Exécuter la séquence principale"""
        rospy.loginfo("🚀 Démarrage positionnement Tiago")
        rospy.sleep(2.0)

        # 1. Position sûre initiale
        self.go_to_safe_position()
        rospy.sleep(1.0)

        # 2. Calculer position cible
        target_position = self.calculate_robot_position()
        rospy.loginfo(
            f"🎯 Position calculée: X={target_position[0]:.3f}, Y={target_position[1]:.3f}, Z={target_position[2]:.3f}")

        # 3. Ajuster les limites
        # Limiter strictement X pour Tiago
        target_position[0] = max(0.35, min(0.75, target_position[0]))
        # Limiter Y
        target_position[1] = max(-0.25, min(0.25, target_position[1]))
        # Limiter Z
        target_position[2] = max(0.65, min(0.95, target_position[2]))

        rospy.loginfo(
            f"🎯 Position ajustée: X={target_position[0]:.3f}, Y={target_position[1]:.3f}, Z={target_position[2]:.3f}")

        # 4. Vérifier si besoin de déplacer le robot
        if target_position[0] > 0.7:  # Si trop loin
            rospy.loginfo("📏 Cible trop éloignée, déplacement nécessaire...")

            # Lever le bras
            self.lift_arm_for_movement()

            # Déplacer le robot
            distance_moved = self.move_robot_toward_target(target_position[0])

            # Ajuster la position estimée
            target_position[0] = max(0.35, target_position[0] - distance_moved)
            rospy.loginfo(f"🎯 Nouvelle position: X={target_position[0]:.3f}")

        # 5. Approche sûre de la cible
        rospy.loginfo("🎯 Approche de la cible...")

        success = self.approach_target_safely(target_position)

        if success:
            rospy.loginfo("✅✅✅ SUCCÈS: Position atteinte avec précision!")
            rospy.loginfo("🔒 Maintien de la position... (Ctrl+C pour arrêter)")

            # Maintenir la position indéfiniment
            # MoveIt garde automatiquement la position atteinte
            try:
                rospy.spin()  # Attendre indéfiniment
            except KeyboardInterrupt:
                rospy.loginfo("🔄 Interruption utilisateur, retour à la position sûre...")
                # Optionnel: retour à la position sûre si désiré
                # self.go_to_safe_position()

        else:
            rospy.logerr("❌❌❌ ÉCHEC: Impossible d'atteindre la cible")

            # Dernière tentative avec position très prudente
            rospy.loginfo("🔄 Dernière tentative position prudente...")
            safe_pos = [0.5, target_position[1], 0.75]
            if self.plan_and_execute_pose(safe_pos, "forward"):
                rospy.loginfo("✅ Position alternative atteinte")
                rospy.sleep(5.0)

            # Retour à la position sûre (optionnel)
            rospy.loginfo("🔄 Retour à la position sûre...")
            self.go_to_safe_position()


def main():
    target_object = "knife"  # "fork", "plate", "knife"
    json_path = "../pipeline/outputs/detections_input.json"

    controller = TiagoPositionWithDepth(target_object, json_path)

    try:
        controller.run()
    except KeyboardInterrupt:
        rospy.loginfo("Interruption utilisateur")
        # Optionnel: ajouter un retour à la position sûre ici si besoin
    except Exception as e:
        rospy.logerr(f"❌ Erreur: {e}")
    finally:
        moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    main()