#!/usr/bin/env python3

import rospy
import json
import numpy as np
import os
from geometry_msgs.msg import Pose, Twist, PoseStamped
import moveit_commander
from tf.transformations import quaternion_from_euler
import math
import time

class TiagoObjectPlacer:
    """Robot Tiago pour pousser les objets dans la scène"""
    
    def __init__(self):
        rospy.init_node('tiago_object_placer')
        
        # Initialisation MoveIt
        rospy.loginfo("🤖 Initialisation de MoveIt...")
        moveit_commander.roscpp_initialize([])
        
        # Groupes de contrôle
        self.arm_group = moveit_commander.MoveGroupCommander("arm_torso")
        self.arm_group.set_planning_time(15.0)
        self.arm_group.set_max_velocity_scaling_factor(0.15)
        self.arm_group.set_max_acceleration_scaling_factor(0.1)
        
        # Publisher pour la base mobile
        self.cmd_vel_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)
        
        # Charger les positions
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.initial_positions = self.load_json(os.path.join(script_dir, "../pipeline/outputs/detections_input.json"))
        self.final_positions = self.load_json(os.path.join(script_dir, "../pipeline/outputs/final_positions.json"))
        
        # Afficher les positions
        self.display_positions()
        
        # Configuration de la scène (image 640x480)
        self.scene_width = 640
        self.scene_height = 480
        self.scene_center_x = self.scene_width // 2  # 320
        self.scene_center_y = self.scene_height // 2  # 240
        
        # Dimensions réelles de la zone de travail
        self.workspace_width = 0.8   # 80cm de large
        self.workspace_depth = 0.6   # 60cm de profondeur
        
        # Position du robot par rapport à la scène
        # Le robot est devant la scène, le centre de la scène est devant lui
        self.robot_base_x = 0.5      # Distance au centre de la scène
        self.robot_base_y = 0.0      # Centré latéralement
        
        # Facteurs de conversion pixel -> mètre
        self.pixel_to_meter_x = self.workspace_width / self.scene_width
        self.pixel_to_meter_y = self.workspace_depth / self.scene_height
        
        # Hauteurs calibrées
        self.table_height = 0.82      # Hauteur de la table
        self.safe_height = 1.0        # Hauteur sûre au-dessus de la table
        self.approach_height = 0.85   # Hauteur d'approche
        self.push_height = 0.80       # Hauteur de poussée
        
        # Vitesses
        self.base_speed = 0.03
        self.arm_speed_slow = 0.1
        self.arm_speed_normal = 0.2
        
        rospy.loginfo(f"📐 Facteurs de conversion: {self.pixel_to_meter_x:.6f} m/px, {self.pixel_to_meter_y:.6f} m/px")
        rospy.sleep(2.0)
    
    def display_positions(self):
        """Afficher les positions chargées"""
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("📊 POSITIONS DANS LA SCÈNE:")
        rospy.loginfo("="*60)
        
        rospy.loginfo("🎯 Positions initiales (dans l'image):")
        for obj in self.initial_positions:
            rospy.loginfo(f"  • {obj['label']}: ({obj['x_pixel']:.1f}, {obj['y_pixel']:.1f})")
        
        rospy.loginfo("\n🎯 Positions finales (dans la même image):")
        for obj in self.final_positions:
            rospy.loginfo(f"  • {obj['label']}: ({obj['x_pixel']:.1f}, {obj['y_pixel']:.1f})")
        rospy.loginfo("="*60)
    
    def load_json(self, filepath):
        """Charger un fichier JSON"""
        try:
            with open(filepath, 'r') as f:
                return json.load(f)
        except Exception as e:
            rospy.logerr(f"❌ Erreur chargement {filepath}: {e}")
            return []
    
    def scene_pixel_to_robot_position(self, pixel_x, pixel_y, height_offset=0.04):
        """
        Convertir les pixels de la scène en position robot
        La scène est vue de dessus, le robot est au bas de l'image
        """
        # Calculer le décalage par rapport au centre de la scène
        offset_x = (pixel_x - self.scene_center_x) * self.pixel_to_meter_x
        offset_y = (pixel_y - self.scene_center_y) * self.pixel_to_meter_y
        
        # Position robot:
        # - X: plus on va vers le haut de l'image (pixel_y petit), plus c'est loin du robot
        # - Y: plus on va vers la droite de l'image (pixel_x grand), plus c'est à droite du robot
        robot_x = self.robot_base_x + offset_y  # Y image -> X robot (profondeur)
        robot_y = self.robot_base_y - offset_x  # X image -> Y robot (latéral, inversé)
        
        # Hauteur = hauteur table + offset
        robot_z = self.table_height + height_offset
        
        # Limites de sécurité
        robot_x = max(0.3, min(0.8, robot_x))
        robot_y = max(-0.3, min(0.3, robot_y))
        robot_z = max(self.table_height + 0.02, min(1.1, robot_z))
        
        rospy.logdebug(f"📐 Pixel({pixel_x:.1f}, {pixel_y:.1f}) -> Robot({robot_x:.3f}, {robot_y:.3f}, {robot_z:.3f})")
        
        return [robot_x, robot_y, robot_z]
    
    def create_pose(self, position, orientation_type="push"):
        """Créer une pose avec orientation"""
        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
        
        if orientation_type == "push":
            # Orientation pour pousser (vers le bas et légèrement vers l'avant)
            quat = quaternion_from_euler(-1.57, 0.2, 0)  # -90° + petit angle avant
        elif orientation_type == "survey":
            # Orientation pour survoler (plus horizontal)
            quat = quaternion_from_euler(-1.0, 0.5, 0)
        elif orientation_type == "transport":
            # Orientation pour transport
            quat = quaternion_from_euler(0, 1.57, 0)
        else:
            quat = quaternion_from_euler(0, 0, 0)
        
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        
        return pose
    
    def move_base_forward(self, distance=0.2):
        """Avancer la base du robot"""
        rospy.loginfo(f"🚗 Avancée de {distance:.2f}m")
        
        twist = Twist()
        twist.linear.x = self.base_speed
        
        move_time = distance / self.base_speed
        start_time = rospy.Time.now().to_sec()
        
        while (rospy.Time.now().to_sec() - start_time) < move_time:
            if rospy.is_shutdown():
                break
            self.cmd_vel_pub.publish(twist)
            rospy.sleep(0.1)
        
        # Arrêt
        twist.linear.x = 0.0
        for _ in range(5):
            self.cmd_vel_pub.publish(twist)
            rospy.sleep(0.1)
        
        rospy.sleep(1.0)
        return True
    
    def go_to_safe_position(self):
        """Aller à une position sûre"""
        rospy.loginfo("🔼 Position sûre")
        
        safe_pos = [0.5, 0.0, self.safe_height]
        safe_pose = self.create_pose(safe_pos, "transport")
        
        self.arm_group.set_max_velocity_scaling_factor(0.1)
        self.arm_group.set_pose_target(safe_pose)
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        
        if success:
            rospy.loginfo("✅ Position sûre atteinte")
        else:
            rospy.logwarn("⚠️  Échec position sûre")
        
        rospy.sleep(0.5)
        return success
    
    def execute_vertical_movement(self, start_pos, end_pos, description):
        """Exécuter un mouvement vertical contrôlé"""
        rospy.loginfo(f"📏 {description}: {start_pos[2]:.3f}m -> {end_pos[2]:.3f}m")
        
        # Créer un chemin avec waypoints pour un mouvement doux
        waypoints = []
        
        # Point intermédiaire
        mid_height = (start_pos[2] + end_pos[2]) / 2
        mid_pos = [start_pos[0], start_pos[1], mid_height]
        
        waypoints.append(self.create_pose(mid_pos, "push"))
        waypoints.append(self.create_pose(end_pos, "push"))
        
        # Exécuter
        self.arm_group.set_max_velocity_scaling_factor(0.05)
        
        try:
            (plan, fraction) = self.arm_group.compute_cartesian_path(
                waypoints, 0.01, 0.0, True
            )
            
            if fraction > 0.5:
                self.arm_group.execute(plan, wait=True)
                return True
            else:
                rospy.logwarn(f"⚠️  Mouvement vertical limité (fraction: {fraction:.2f})")
                return False
        except Exception as e:
            rospy.logerr(f"❌ Erreur mouvement vertical: {e}")
            return False
    
    def execute_push_movement(self, start_pixel, end_pixel, label):
        """Exécuter le mouvement de poussée d'un objet dans la scène"""
        rospy.loginfo(f"\n🤖 POUSSÉE DE {label.upper()}")
        rospy.loginfo(f"📍 Scene start: ({start_pixel[0]:.1f}, {start_pixel[1]:.1f})")
        rospy.loginfo(f"📍 Scene end: ({end_pixel[0]:.1f}, {end_pixel[1]:.1f})")
        
        # Convertir les positions de la scène en positions robot
        approach_start = self.scene_pixel_to_robot_position(
            start_pixel[0], start_pixel[1], 0.08
        )
        
        push_start = self.scene_pixel_to_robot_position(
            start_pixel[0], start_pixel[1], 0.04
        )
        
        push_end = self.scene_pixel_to_robot_position(
            end_pixel[0], end_pixel[1], 0.04
        )
        
        approach_end = self.scene_pixel_to_robot_position(
            end_pixel[0], end_pixel[1], 0.08
        )
        
        rospy.loginfo(f"🤖 Robot start: ({push_start[0]:.3f}, {push_start[1]:.3f})")
        rospy.loginfo(f"🤖 Robot end: ({push_end[0]:.3f}, {push_end[1]:.3f})")
        
        # 1. Aller à la position d'approche
        rospy.loginfo("1️⃣  Approche de l'objet")
        self.arm_group.set_max_velocity_scaling_factor(self.arm_speed_slow)
        self.arm_group.set_pose_target(self.create_pose(approach_start, "survey"))
        if not self.arm_group.go(wait=True):
            rospy.logwarn("⚠️  Échec approche")
            return False
        
        # 2. Descente vers l'objet
        rospy.loginfo("2️⃣  Descente vers l'objet")
        if not self.execute_vertical_movement(approach_start, push_start, "Descente"):
            rospy.logwarn("⚠️  Échec descente")
            return False
        
        # 3. Poussée vers la position finale
        rospy.loginfo("3️⃣  Poussée vers position finale")
        
        # Calculer la distance et diviser en segments si nécessaire
        distance = math.sqrt(
            (push_end[0] - push_start[0])**2 + 
            (push_end[1] - push_start[1])**2
        )
        
        if distance > 0.15:  # Si plus de 15cm, diviser
            num_segments = max(2, int(distance / 0.08))
            rospy.loginfo(f"📏 Poussée en {num_segments} segments ({distance:.3f}m)")
            
            for i in range(num_segments + 1):
                t = i / num_segments
                inter_x = push_start[0] + t * (push_end[0] - push_start[0])
                inter_y = push_start[1] + t * (push_end[1] - push_start[1])
                inter_pos = [inter_x, inter_y, push_start[2]]
                
                self.arm_group.set_max_velocity_scaling_factor(0.03)
                self.arm_group.set_pose_target(self.create_pose(inter_pos, "push"))
                if not self.arm_group.go(wait=True):
                    rospy.logwarn(f"⚠️  Échec segment {i+1}")
                    break
                rospy.sleep(0.1)
        else:
            # Poussée directe
            self.arm_group.set_max_velocity_scaling_factor(0.04)
            self.arm_group.set_pose_target(self.create_pose(push_end, "push"))
            if not self.arm_group.go(wait=True):
                rospy.logwarn("⚠️  Échec poussée directe")
                return False
        
        # 4. Remontée
        rospy.loginfo("4️⃣  Remontée")
        if not self.execute_vertical_movement(push_end, approach_end, "Remontée"):
            rospy.logwarn("⚠️  Échec remontée")
        
        rospy.loginfo(f"✅ {label} poussé avec succès")
        return True
    
    def find_object_positions(self, label):
        """Trouver les positions initiale et finale d'un objet dans la scène"""
        init_pos = None
        final_pos = None
        
        for obj in self.initial_positions:
            if obj['label'].lower() == label.lower():
                init_pos = (obj['x_pixel'], obj['y_pixel'])
                break
        
        for obj in self.final_positions:
            if obj['label'].lower() == label.lower():
                final_pos = (obj['x_pixel'], obj['y_pixel'])
                break
        
        return init_pos, final_pos
    
    def calibrate_scene(self):
        """Calibrer la relation entre la scène image et le monde réel"""
        rospy.loginfo("\n🔧 CALIBRATION DE LA SCÈNE")
        
        # Aller à une position centrale haute
        center_pos = self.scene_pixel_to_robot_position(
            self.scene_center_x, self.scene_center_y, 0.1
        )
        
        rospy.loginfo(f"📍 Centre scène: pixel({self.scene_center_x}, {self.scene_center_y})")
        rospy.loginfo(f"📍 Position robot: ({center_pos[0]:.3f}, {center_pos[1]:.3f}, {center_pos[2]:.3f})")
        
        self.arm_group.set_pose_target(self.create_pose(center_pos, "survey"))
        self.arm_group.go(wait=True)
        
        rospy.loginfo("✅ Calibration prête")
        rospy.sleep(1.0)
        
        # Retour en sécurité
        self.go_to_safe_position()
    
    def run_placement(self):
        """Exécuter le placement des objets dans la scène"""
        rospy.loginfo("\n" + "="*70)
        rospy.loginfo("🎬 DÉMARRAGE DU RANGEMENT DANS LA SCÈNE")
        rospy.loginfo("="*70)
        
        # Attendre
        rospy.sleep(2.0)
        
        # ÉTAPE 1: Position initiale sûre
        rospy.loginfo("\n📋 ÉTAPE 1: POSITION INITIALE")
        self.go_to_safe_position()
        
        # ÉTAPE 2: Avancer le robot
        rospy.loginfo("\n📋 ÉTAPE 2: APPROCHE")
        self.move_base_forward(0.15)
        
        # ÉTAPE 3: Calibration
        rospy.loginfo("\n📋 ÉTAPE 3: CALIBRATION")
        self.calibrate_scene()
        
        # ÉTAPE 4: Traitement des objets
        rospy.loginfo("\n📋 ÉTAPE 4: RANGEMENT DES OBJETS")
        
        # Ordre de traitement optimisé
        objects_to_process = ["fork", "knife", "plate"]
        successful_objects = []
        
        for label in objects_to_process:
            rospy.loginfo("\n" + "="*50)
            rospy.loginfo(f"🎯 TRAITEMENT: {label.upper()}")
            rospy.loginfo("="*50)
            
            # Trouver les positions dans la scène
            init_pixel, final_pixel = self.find_object_positions(label)
            
            if not init_pixel or not final_pixel:
                rospy.logwarn(f"⚠️  Positions manquantes pour {label}")
                continue
            
            # Aller à une position sûre avant chaque objet
            self.go_to_safe_position()
            rospy.sleep(0.5)
            
            # Exécuter la poussée
            try:
                if self.execute_push_movement(init_pixel, final_pixel, label):
                    successful_objects.append(label)
                    rospy.loginfo(f"✅ {label} rangé avec succès!")
                else:
                    rospy.logwarn(f"⚠️  Échec partiel pour {label}")
                
                # Pause entre les objets
                rospy.sleep(0.5)
                
            except Exception as e:
                rospy.logerr(f"❌ Erreur avec {label}: {e}")
                self.go_to_safe_position()
                rospy.sleep(1.0)
        
        # Résumé
        rospy.loginfo("\n" + "="*70)
        rospy.loginfo("📊 RÉSUMÉ DE LA MISSION")
        rospy.loginfo("="*70)
        rospy.loginfo(f"✅ Objets réussis: {len(successful_objects)}/{len(objects_to_process)}")
        rospy.loginfo(f"📋 Liste: {', '.join(successful_objects) if successful_objects else 'Aucun'}")
        
        if len(successful_objects) > 0:
            rospy.loginfo("🎉 MISSION ACCOMPLIE !")
        else:
            rospy.loginfo("⚠️  Mission terminée avec des difficultés")
        
        # Position finale sûre
        rospy.loginfo("\n📋 POSITION FINALE")
        self.go_to_safe_position()
        
        # Petit recul
        twist = Twist()
        twist.linear.x = -0.02
        for _ in range(50):
            self.cmd_vel_pub.publish(twist)
            rospy.sleep(0.1)
        
        rospy.loginfo("🤖 Système prêt")
        rospy.spin()
    
    def run(self):
        """Exécuter la séquence principale"""
        try:
            self.run_placement()
        except KeyboardInterrupt:
            rospy.loginfo("\n🛑 Arrêt par l'utilisateur")
            self.go_to_safe_position()
        except Exception as e:
            rospy.logerr(f"\n💥 Erreur: {e}")
            self.go_to_safe_position()
        finally:
            moveit_commander.roscpp_shutdown()

def main():
    rospy.loginfo("\n" + "="*70)
    rospy.loginfo("🤖 TIAGo - SYSTÈME DE RANGEMENT DANS LA SCÈNE")
    rospy.loginfo("="*70)
    
    placer = TiagoObjectPlacer()
    
    try:
        placer.run()
    except Exception as e:
        rospy.logerr(f"💥 Erreur fatale: {e}")
    
    rospy.loginfo("👋 Programme terminé")

if __name__ == '__main__':
    main()