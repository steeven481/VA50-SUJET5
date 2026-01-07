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
    """Robot Tiago pour déplacer les objets sur la table"""
    
    def __init__(self):
        rospy.init_node('tiago_object_placer')
        
        # Initialisation MoveIt
        rospy.loginfo("🤖 Initialisation de MoveIt...")
        moveit_commander.roscpp_initialize([])
        
        # Groupes de contrôle
        self.arm_group = moveit_commander.MoveGroupCommander("arm_torso")
        self.arm_group.set_planning_time(20.0)
        self.arm_group.set_max_velocity_scaling_factor(0.1)
        self.arm_group.set_max_acceleration_scaling_factor(0.08)
        
        # Configurer le planificateur pour mieux éviter les obstacles
        self.arm_group.set_num_planning_attempts(10)
        self.arm_group.set_goal_position_tolerance(0.02)
        self.arm_group.set_goal_orientation_tolerance(0.1)
        
        # Publisher pour la base mobile
        self.cmd_vel_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)
        
        # Charger les positions
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.initial_positions = self.load_json(os.path.join(script_dir, "../pipeline/outputs/detections_input.json"))
        self.final_positions = self.load_json(os.path.join(script_dir, "../pipeline/outputs/final_positions.json"))
        
        # Afficher les positions
        self.display_positions()
        
        # Configuration de la scène
        # L'image de détection est 640x480 (de la caméra xtion)
        # L'image générée est 384x384
        # On utilise les positions des pixels par rapport à l'image générée (384x384)
        self.input_img_width = 640
        self.input_img_height = 480
        self.generated_img_width = 384
        self.generated_img_height = 384
        
        # Positions Gazebo réelles depuis le fichier world:
        # Table: position (0.839, -0.011) rotée de -90° environ
        # Surface de table à z ≈ 0.74m
        # Plate: (0.872, -0.164, 0.742)
        # Fork: (0.797, 0.123, 0.740)
        # Knife: (0.961, 0.023, 0.695)
        
        # Zone de travail sur la table (calibrée depuis les positions Gazebo)
        # La table est centrée autour de x=0.85m, y=0.0m
        self.table_center_x = 0.85   # Centre X de la table dans le repère robot
        self.table_center_y = 0.0    # Centre Y de la table
        self.table_width = 0.50      # Largeur effective de la zone de travail (Y)
        self.table_depth = 0.40      # Profondeur effective (X)
        
        # Hauteurs calibrées précisément depuis Gazebo
        self.table_surface_z = 0.74   # Hauteur exacte de la surface de table
        self.safe_height_z = 1.05     # Hauteur sûre bien au-dessus de la table
        self.approach_height_z = 0.90 # Hauteur d'approche sécuritaire
        self.push_height_z = 0.78     # Hauteur pour pousser (juste au-dessus des objets)
        
        # Vitesses sécuritaires
        self.base_speed = 0.02
        self.arm_speed_slow = 0.05
        self.arm_speed_normal = 0.1
        
        # Paramètres de mouvement
        self.cartesian_path_min_fraction = 0.8  # Fraction minimale pour accepter un chemin cartésien
        self.min_movement_distance = 0.01       # Distance minimale pour déclencher un mouvement (en m)
        self.push_offset_distance = 0.05        # Distance de recul avant de pousser un objet (en m)
        self.segment_distance_threshold = 0.15  # Distance au-delà de laquelle on segmente le mouvement
        self.segment_length = 0.10              # Longueur de chaque segment de mouvement
        
        rospy.loginfo(f"📐 Zone de travail: centre=({self.table_center_x:.2f}, {self.table_center_y:.2f}), taille=({self.table_depth:.2f}x{self.table_width:.2f})m")
        rospy.sleep(2.0)
    
    def display_positions(self):
        """Afficher les positions chargées"""
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("📊 POSITIONS DANS LA SCÈNE:")
        rospy.loginfo("="*60)
        
        rospy.loginfo("🎯 Positions initiales (pixels):")
        for obj in self.initial_positions:
            rospy.loginfo(f"  • {obj['label']}: ({obj['x_pixel']:.1f}, {obj['y_pixel']:.1f})")
        
        rospy.loginfo("\n🎯 Positions finales (pixels):")
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
    
    def pixel_to_world_position(self, pixel_x, pixel_y, img_width, img_height, target_z=None):
        """
        Convertir les coordonnées pixel en coordonnées monde (repère robot base_footprint).
        
        Le système de coordonnées:
        - L'image a (0,0) en haut à gauche
        - pixel_x augmente vers la droite -> Y robot diminue (vers la droite du robot)
        - pixel_y augmente vers le bas -> X robot augmente (vers l'avant)
        
        On mappe les pixels sur la zone de travail calibrée sur la table.
        """
        if target_z is None:
            target_z = self.push_height_z
        
        # Normaliser les pixels (0 à 1)
        norm_x = pixel_x / img_width   # 0=gauche, 1=droite
        norm_y = pixel_y / img_height  # 0=haut, 1=bas
        
        # Mapper sur la zone de travail de la table
        # X robot (avant/arrière): plus pixel_y est grand (bas de l'image), plus proche du robot
        # Pour une vue du dessus avec tête baissée: haut de l'image = loin, bas = proche
        robot_x = self.table_center_x + self.table_depth * (0.5 - norm_y)
        
        # Y robot (gauche/droite): pixel_x augmente vers la droite
        # Droite de l'image = droite du robot = Y négatif
        robot_y = self.table_center_y + self.table_width * (0.5 - norm_x)
        
        robot_z = target_z
        
        rospy.loginfo(f"📐 Pixel({pixel_x:.1f}, {pixel_y:.1f}) -> World({robot_x:.3f}, {robot_y:.3f}, {robot_z:.3f})")
        
        return [robot_x, robot_y, robot_z]
    
    def get_object_world_position_initial(self, label):
        """Obtenir la position monde initiale d'un objet."""
        for obj in self.initial_positions:
            if obj['label'].lower() == label.lower():
                return self.pixel_to_world_position(
                    obj['x_pixel'], obj['y_pixel'],
                    self.input_img_width, self.input_img_height
                )
        return None
    
    def get_object_world_position_final(self, label):
        """Obtenir la position monde finale d'un objet."""
        for obj in self.final_positions:
            if obj['label'].lower() == label.lower():
                return self.pixel_to_world_position(
                    obj['x_pixel'], obj['y_pixel'],
                    self.generated_img_width, self.generated_img_height
                )
        return None
    
    def create_pose(self, position, orientation_type="push"):
        """Créer une pose avec orientation appropriée pour la manipulation."""
        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
        
        if orientation_type == "push":
            # Orientation pour pousser: gripper pointant vers le bas, légèrement incliné vers l'avant
            # Roll=-90° (tourner autour de X), Pitch=0, Yaw=0
            quat = quaternion_from_euler(-1.57, 0.0, 0.0)
        elif orientation_type == "survey":
            # Orientation pour survoler/observer
            quat = quaternion_from_euler(-1.57, 0.0, 0.0)
        elif orientation_type == "transport":
            # Orientation pour transport sécuritaire (gripper vers l'avant)
            quat = quaternion_from_euler(0, 0, 0)
        elif orientation_type == "side_push_left":
            # Pousser vers la gauche du robot (Y positif)
            quat = quaternion_from_euler(-1.57, 0.0, 1.57)
        elif orientation_type == "side_push_right":
            # Pousser vers la droite du robot (Y négatif)
            quat = quaternion_from_euler(-1.57, 0.0, -1.57)
        else:
            quat = quaternion_from_euler(0, 0, 0)
        
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        
        return pose
    
    def move_base_forward(self, distance=0.1):
        """Avancer la base du robot de manière sécuritaire."""
        rospy.loginfo(f"🚗 Avancée de {distance:.2f}m")
        
        twist = Twist()
        twist.linear.x = self.base_speed
        
        move_time = abs(distance / self.base_speed)
        start_time = rospy.Time.now().to_sec()
        
        while (rospy.Time.now().to_sec() - start_time) < move_time:
            if rospy.is_shutdown():
                break
            self.cmd_vel_pub.publish(twist)
            rospy.sleep(0.1)
        
        # Arrêt complet
        twist.linear.x = 0.0
        for _ in range(10):
            self.cmd_vel_pub.publish(twist)
            rospy.sleep(0.1)
        
        rospy.sleep(1.0)
        return True
    
    def move_to_home_position(self):
        """Aller à une position de repos sécuritaire."""
        rospy.loginfo("🏠 Retour position de repos...")
        
        try:
            # Utiliser une pose nommée si disponible, sinon position haute sécuritaire
            self.arm_group.set_max_velocity_scaling_factor(0.1)
            
            # Configuration sécuritaire des joints pour position de repos
            # Ces valeurs sont calibrées pour le robot TIAGo
            self.arm_group.set_joint_value_target({
                'torso_lift_joint': 0.25,
                'arm_1_joint': 0.2,
                'arm_2_joint': -1.34,
                'arm_3_joint': -0.2,
                'arm_4_joint': 1.94,
                'arm_5_joint': -1.57,
                'arm_6_joint': 1.37,
                'arm_7_joint': 0.0
            })
            
            success = self.arm_group.go(wait=True)
            self.arm_group.stop()
            
            if success:
                rospy.loginfo("✅ Position de repos atteinte")
            else:
                rospy.logwarn("⚠️ Échec position de repos, essai alternatif...")
                # Position alternative haute
                safe_pos = [0.35, 0.0, self.safe_height_z]
                safe_pose = self.create_pose(safe_pos, "transport")
                self.arm_group.set_pose_target(safe_pose)
                success = self.arm_group.go(wait=True)
                self.arm_group.stop()
            
            rospy.sleep(0.5)
            return success
            
        except Exception as e:
            rospy.logerr(f"❌ Erreur position repos: {e}")
            return False
    
    def go_to_safe_position(self):
        """Aller à une position sûre au-dessus de la table."""
        rospy.loginfo("🔼 Déplacement vers position sûre...")
        
        # Position sûre au-dessus du centre de la table
        safe_pos = [self.table_center_x - 0.15, 0.0, self.safe_height_z]
        safe_pose = self.create_pose(safe_pos, "survey")
        
        self.arm_group.set_max_velocity_scaling_factor(0.1)
        self.arm_group.set_pose_target(safe_pose)
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        
        if success:
            rospy.loginfo("✅ Position sûre atteinte")
        else:
            rospy.logwarn("⚠️ Échec position sûre, essai avec joints...")
            # Alternative: utiliser home position
            return self.move_to_home_position()
        
        rospy.sleep(0.5)
        return success
    
    def move_arm_cartesian(self, start_pos, end_pos, speed_factor=0.05):
        """Exécuter un mouvement cartésien linéaire entre deux positions."""
        rospy.loginfo(f"📏 Mouvement: ({start_pos[0]:.3f},{start_pos[1]:.3f},{start_pos[2]:.3f}) -> ({end_pos[0]:.3f},{end_pos[1]:.3f},{end_pos[2]:.3f})")
        
        self.arm_group.set_max_velocity_scaling_factor(speed_factor)
        
        waypoints = []
        waypoints.append(self.create_pose(end_pos, "push"))
        
        try:
            (plan, fraction) = self.arm_group.compute_cartesian_path(
                waypoints, 
                0.01,  # eef_step: résolution du chemin
                0.0,   # jump_threshold: désactivé
                True   # avoid_collisions
            )
            
            if fraction > self.cartesian_path_min_fraction:
                rospy.loginfo(f"✅ Chemin planifié ({fraction*100:.0f}%)")
                self.arm_group.execute(plan, wait=True)
                self.arm_group.stop()
                return True
            else:
                rospy.logwarn(f"⚠️ Chemin partiel ({fraction*100:.0f}%), essai mouvement direct...")
                # Fallback: mouvement point-à-point
                self.arm_group.set_pose_target(self.create_pose(end_pos, "push"))
                success = self.arm_group.go(wait=True)
                self.arm_group.stop()
                return success
                
        except Exception as e:
            rospy.logerr(f"❌ Erreur mouvement cartésien: {e}")
            return False
    
    def push_object_to_target(self, label, start_world, end_world):
        """
        Pousser un objet de sa position initiale vers sa position finale.
        
        Séquence:
        1. Aller au-dessus de la position initiale (hauteur sûre)
        2. Descendre à la hauteur de poussée
        3. Pousser vers la position finale
        4. Remonter à la hauteur sûre
        """
        rospy.loginfo(f"\n🎯 DÉPLACEMENT DE {label.upper()}")
        rospy.loginfo(f"   De: ({start_world[0]:.3f}, {start_world[1]:.3f})")
        rospy.loginfo(f"   Vers: ({end_world[0]:.3f}, {end_world[1]:.3f})")
        
        # Calculer la direction et la distance du mouvement
        dx = end_world[0] - start_world[0]
        dy = end_world[1] - start_world[1]
        distance = math.sqrt(dx*dx + dy*dy)
        rospy.loginfo(f"   Distance: {distance:.3f}m")
        
        if distance < self.min_movement_distance:
            rospy.loginfo(f"✅ {label} est déjà à la bonne position!")
            return True
        
        # Point de départ légèrement avant l'objet (pour avoir de l'élan)
        # On recule dans la direction opposée au mouvement
        if distance > 0:
            offset_x = -self.push_offset_distance * (dx / distance)
            offset_y = -self.push_offset_distance * (dy / distance)
        else:
            offset_x = offset_y = 0
        
        # Positions clés du mouvement
        approach_start = [start_world[0] + offset_x, start_world[1] + offset_y, self.approach_height_z]
        push_start = [start_world[0] + offset_x, start_world[1] + offset_y, self.push_height_z]
        push_end = [end_world[0], end_world[1], self.push_height_z]
        lift_end = [end_world[0], end_world[1], self.approach_height_z]
        
        # ÉTAPE 1: Aller au-dessus de la position de départ
        rospy.loginfo("  1️⃣ Approche au-dessus de l'objet...")
        approach_pose = self.create_pose(approach_start, "push")
        self.arm_group.set_max_velocity_scaling_factor(0.1)
        self.arm_group.set_pose_target(approach_pose)
        if not self.arm_group.go(wait=True):
            rospy.logwarn("⚠️ Échec approche, nouvel essai...")
            rospy.sleep(1.0)
            if not self.arm_group.go(wait=True):
                rospy.logerr("❌ Impossible d'atteindre la position d'approche")
                return False
        self.arm_group.stop()
        rospy.sleep(0.5)
        
        # ÉTAPE 2: Descendre à la hauteur de poussée
        rospy.loginfo("  2️⃣ Descente vers l'objet...")
        if not self.move_arm_cartesian(approach_start, push_start, 0.03):
            rospy.logerr("❌ Échec descente")
            return False
        rospy.sleep(0.3)
        
        # ÉTAPE 3: Pousser vers la position finale
        rospy.loginfo("  3️⃣ Poussée vers la position finale...")
        
        # Si la distance est grande, diviser en segments
        if distance > self.segment_distance_threshold:
            num_segments = max(2, int(distance / self.segment_length))
            rospy.loginfo(f"     Mouvement en {num_segments} segments")
            
            current_pos = push_start.copy()
            for i in range(num_segments):
                t = (i + 1) / num_segments
                next_pos = [
                    push_start[0] + t * (push_end[0] - push_start[0]),
                    push_start[1] + t * (push_end[1] - push_start[1]),
                    self.push_height_z
                ]
                
                if not self.move_arm_cartesian(current_pos, next_pos, 0.03):
                    rospy.logwarn(f"⚠️ Segment {i+1} échoué, poursuite...")
                
                current_pos = next_pos
                rospy.sleep(0.2)
        else:
            # Mouvement direct pour courtes distances
            if not self.move_arm_cartesian(push_start, push_end, 0.03):
                rospy.logwarn("⚠️ Poussée partielle")
        
        rospy.sleep(0.3)
        
        # ÉTAPE 4: Remonter
        rospy.loginfo("  4️⃣ Remontée...")
        if not self.move_arm_cartesian(push_end, lift_end, 0.05):
            rospy.logwarn("⚠️ Échec remontée, tentative directe...")
            lift_pose = self.create_pose(lift_end, "push")
            self.arm_group.set_pose_target(lift_pose)
            self.arm_group.go(wait=True)
            self.arm_group.stop()
        
        rospy.loginfo(f"✅ {label} déplacé avec succès!")
        return True
    
    def find_object_positions(self, label):
        """Trouver les positions initiale et finale d'un objet."""
        init_world = self.get_object_world_position_initial(label)
        final_world = self.get_object_world_position_final(label)
        return init_world, final_world
    
    def run_placement(self):
        """Exécuter le placement des objets pour dresser la table."""
        rospy.loginfo("\n" + "="*70)
        rospy.loginfo("🍽️  DÉMARRAGE DU DRESSAGE DE TABLE")
        rospy.loginfo("="*70)
        
        # Attendre que tout soit initialisé
        rospy.sleep(3.0)
        
        # ÉTAPE 1: Position initiale sûre
        rospy.loginfo("\n📋 ÉTAPE 1: POSITION INITIALE")
        if not self.move_to_home_position():
            rospy.logwarn("⚠️ Impossible d'atteindre la position initiale")
        
        # ÉTAPE 2: Avancer légèrement le robot vers la table (optionnel)
        # rospy.loginfo("\n📋 ÉTAPE 2: APPROCHE DE LA TABLE")
        # self.move_base_forward(0.10)
        
        # ÉTAPE 3: Traitement des objets
        # Ordre optimisé: d'abord les couverts (plus petits), puis le plat
        # Ceci évite de bousculer les couverts quand on déplace le plat
        rospy.loginfo("\n📋 ÉTAPE 2: RANGEMENT DES OBJETS")
        
        objects_to_process = ["fork", "knife", "plate"]
        successful_objects = []
        failed_objects = []
        
        for label in objects_to_process:
            rospy.loginfo("\n" + "="*50)
            rospy.loginfo(f"🎯 TRAITEMENT: {label.upper()}")
            rospy.loginfo("="*50)
            
            # Trouver les positions
            init_world, final_world = self.find_object_positions(label)
            
            if init_world is None or final_world is None:
                rospy.logwarn(f"⚠️ Positions manquantes pour {label}")
                failed_objects.append(label)
                continue
            
            # Aller à une position sûre avant chaque objet
            rospy.loginfo(f"🔄 Préparation pour {label}...")
            self.go_to_safe_position()
            rospy.sleep(1.0)
            
            # Exécuter le déplacement
            try:
                if self.push_object_to_target(label, init_world, final_world):
                    successful_objects.append(label)
                    rospy.loginfo(f"✅ {label} rangé avec succès!")
                else:
                    failed_objects.append(label)
                    rospy.logwarn(f"⚠️ Échec pour {label}")
                
                # Pause entre les objets
                rospy.sleep(1.0)
                
            except Exception as e:
                rospy.logerr(f"❌ Erreur avec {label}: {e}")
                failed_objects.append(label)
                self.go_to_safe_position()
                rospy.sleep(1.0)
        
        # Résumé final
        rospy.loginfo("\n" + "="*70)
        rospy.loginfo("📊 RÉSUMÉ DU DRESSAGE DE TABLE")
        rospy.loginfo("="*70)
        rospy.loginfo(f"✅ Objets réussis: {len(successful_objects)}/{len(objects_to_process)}")
        if successful_objects:
            rospy.loginfo(f"   {', '.join(successful_objects)}")
        if failed_objects:
            rospy.loginfo(f"❌ Objets échoués: {', '.join(failed_objects)}")
        
        if len(successful_objects) == len(objects_to_process):
            rospy.loginfo("\n🎉 TABLE PARFAITEMENT DRESSÉE !")
        elif len(successful_objects) > 0:
            rospy.loginfo("\n⚠️ Table partiellement dressée")
        else:
            rospy.loginfo("\n❌ Échec du dressage de table")
        
        # Position finale sûre
        rospy.loginfo("\n📋 ÉTAPE FINALE: RETOUR POSITION DE REPOS")
        self.move_to_home_position()
        
        rospy.loginfo("\n🤖 Système prêt")
        
        # Garder le noeud actif pour permettre de voir le résultat
        rospy.sleep(5.0)
    
    def run(self):
        """Exécuter la séquence principale."""
        try:
            self.run_placement()
        except KeyboardInterrupt:
            rospy.loginfo("\n🛑 Arrêt par l'utilisateur")
            self.move_to_home_position()
        except Exception as e:
            rospy.logerr(f"\n💥 Erreur: {e}")
            import traceback
            traceback.print_exc()
            self.move_to_home_position()
        finally:
            moveit_commander.roscpp_shutdown()

def main():
    rospy.loginfo("\n" + "="*70)
    rospy.loginfo("🤖 TIAGo - SYSTÈME DE DRESSAGE DE TABLE")
    rospy.loginfo("="*70)
    
    placer = TiagoObjectPlacer()
    
    try:
        placer.run()
    except Exception as e:
        rospy.logerr(f"💥 Erreur fatale: {e}")
        import traceback
        traceback.print_exc()
    
    rospy.loginfo("👋 Programme terminé")

if __name__ == '__main__':
    main()