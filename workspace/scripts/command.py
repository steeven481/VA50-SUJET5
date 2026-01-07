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
    """Robot Tiago pour pousser les objets à leurs positions finales"""
    def __init__(self):
        rospy.init_node('tiago_object_placer')
        
        # Initialisation MoveIt
        moveit_commander.roscpp_initialize([])
        self.arm_group = moveit_commander.MoveGroupCommander("arm_torso")
        
        # Configuration MoveIt
        self.arm_group.set_planning_time(10.0)
        self.arm_group.set_max_velocity_scaling_factor(0.1)
        self.arm_group.set_goal_position_tolerance(0.03)
        self.arm_group.set_goal_orientation_tolerance(0.1)
        
        # Publisher pour la base mobile
        self.cmd_vel_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)
        
        # Charger les positions
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.initial_positions = self.load_json(os.path.join(script_dir, "../pipeline/outputs/detections_input.json"))
        self.final_positions = self.load_json(os.path.join(script_dir, "../pipeline/outputs/final_positions.json"))
        
        # Facteurs de conversion
        self.pixel_to_meter_x = 0.0015
        self.pixel_to_meter_y = 0.0015
        
        # Centre de l'image
        self.image_center_x = 320
        self.image_center_y = 240
        
        # Positions ajustées
        self.base_x = 0.5
        self.base_y = 0.0
        self.push_height = 0.7        # Hauteur pour pousser les objets
        self.approach_height = 0.85   # Hauteur d'approche (légèrement plus haute)
        self.safe_height = 0.95       # Hauteur de sécurité au-dessus de la table
        self.transport_height = 1.05  # Hauteur pour déplacement du robot
        
        rospy.loginfo("📊 Positions chargées")
        rospy.sleep(2.0)
    
    def load_json(self, filepath):
        """Charger un fichier JSON"""
        try:
            with open(filepath, 'r') as f:
                return json.load(f)
        except Exception as e:
            rospy.logerr(f"❌ Erreur chargement {filepath}: {e}")
            return []
    
    def move_base_toward_table(self, distance=0.2):
        """Faire avancer le robot vers la table"""
        rospy.loginfo(f"🚗 AVANCÉE VERS LA TABLE: {distance}m...")
        
        # Commande de vitesse très lente
        twist = Twist()
        twist.linear.x = 0.05  # Très lent pour plus de précision
        
        # Calculer le temps de déplacement
        move_time = distance / 0.05
        
        # Déplacer le robot
        start_time = rospy.Time.now().to_sec()
        rate = rospy.Rate(10)
        
        rospy.loginfo(f"⏱️  Déplacement pendant {move_time:.1f} secondes")
        
        while (rospy.Time.now().to_sec() - start_time) < move_time:
            if rospy.is_shutdown():
                break
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
        
        # Arrêter progressivement
        twist.linear.x = 0.0
        for _ in range(5):
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
        
        rospy.sleep(2.0)  # Attendre que le robot s'arrête
        rospy.loginfo("✅ Robot avancé vers la table")
    
    def pixel_to_robot_position(self, pixel_x, pixel_y, height_type="push"):
        """Convertir pixels en position robot avec hauteur appropriée"""
        # Calculer les offsets
        offset_x = (pixel_x - self.image_center_x) * self.pixel_to_meter_x
        offset_y = (pixel_y - self.image_center_y) * self.pixel_to_meter_y
        
        # Position calculée
        x = self.base_x + offset_x
        y = self.base_y - offset_y  # Inversion Y
        
        # Déterminer la hauteur en fonction du type
        if height_type == "push":
            z = self.push_height
        elif height_type == "approach":
            z = self.approach_height
        elif height_type == "transport":
            z = self.transport_height
        else:
            z = self.safe_height
        
        # Limites de sécurité
        x = max(0.4, min(0.8, x))
        y = max(-0.3, min(0.3, y))
        z = max(0.6, min(1.1, z))  # Augmenté la limite supérieure
        
        return [x, y, z]
    
    def create_pose(self, position, orientation_type="down"):
        """Créer une pose avec orientation"""
        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
        
        if orientation_type == "down":
            # Orientation vers le bas
            quat = quaternion_from_euler(-math.pi/2, 0, math.pi/2)
        elif orientation_type == "forward":
            # Orientation vers l'avant
            quat = quaternion_from_euler(0, math.pi/2, 0)
        elif orientation_type == "horizontal":
            # Orientation horizontale pour déplacement
            quat = quaternion_from_euler(0, math.pi/2, math.pi/2)
        else:
            quat = quaternion_from_euler(0, 0, 0)
        
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        
        return pose
    
    def go_to_high_position(self):
        """Aller à une position haute pour déplacer le robot"""
        rospy.loginfo("🔼 Position haute pour déplacement...")
        
        high_position = [0.5, 0.0, self.transport_height]
        high_pose = self.create_pose(high_position, "horizontal")  # Orientation horizontale
        
        try:
            self.arm_group.set_pose_target(high_pose)
            success = self.arm_group.go(wait=True)
            self.arm_group.stop()
            
            if success:
                rospy.loginfo("✅ Position haute atteinte")
            else:
                rospy.logwarn("⚠️  Échec position haute")
                # Essayer une position alternative
                self.go_to_safe_position()
        except Exception as e:
            rospy.logwarn(f"⚠️  Erreur position haute: {e}")
            self.go_to_safe_position()
        
        rospy.sleep(1.0)
        return True
    
    def go_to_safe_position(self):
        """Aller à une position sûre au-dessus de la table"""
        rospy.loginfo("🔄 Position sûre...")
        
        safe_position = [0.5, 0.0, self.safe_height]
        safe_pose = self.create_pose(safe_position, "horizontal")
        
        try:
            self.arm_group.set_pose_target(safe_pose)
            success = self.arm_group.go(wait=True)
            self.arm_group.stop()
            
            if success:
                rospy.loginfo("✅ Position sûre atteinte")
            else:
                rospy.logwarn("⚠️  Échec position sûre")
        except Exception as e:
            rospy.logwarn(f"⚠️  Erreur position sûre: {e}")
        
        rospy.sleep(1.0)
        return True
    
    def move_to_position(self, position, label):
        """Déplacer le bras à une position"""
        rospy.loginfo(f"🎯 Déplacement vers {label}...")
        rospy.loginfo(f"📍 Position: ({position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f})")
        
        # Orientation vers le bas pour pousser
        pose = self.create_pose(position, "down")
        
        self.arm_group.set_pose_target(pose)
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        
        if success:
            rospy.loginfo("✅ Position atteinte")
            rospy.sleep(0.5)
            return True
        else:
            rospy.logwarn("⚠️  Échec du déplacement")
            # En cas d'échec, retourner à une position sûre
            self.go_to_safe_position()
            return False
    
    def push_object(self, label, start_position, end_position):
        """Pousser un objet d'une position à une autre"""
        rospy.loginfo(f"🤏 Poussée de {label}...")
        
        # 1. Approche de la position de départ (plus haute)
        approach_start = start_position.copy()
        approach_start[2] = self.approach_height
        
        if not self.move_to_position(approach_start, f"{label} (approche)"):
            rospy.loginfo("🔄 Retour à la position sûre...")
            self.go_to_safe_position()
            return False
        
        # 2. Descente pour pousser (légèrement seulement)
        if not self.move_to_position(start_position, f"{label} (contact)"):
            rospy.loginfo("🔼 Relevage après échec...")
            self.go_to_safe_position()
            return False
        
        # 3. Poussée vers la position finale
        rospy.loginfo(f"🎯 Poussée de {label} vers position finale...")
        
        if not self.move_to_position(end_position, f"{label} (poussée)"):
            rospy.loginfo("🔼 Relevage après échec de poussée...")
            self.go_to_safe_position()
            return False
        
        # 4. Relever le bras à une hauteur de sécurité
        rospy.loginfo(f"🔼 Relevage après poussée de {label}...")
        self.go_to_safe_position()
        
        rospy.loginfo(f"✅ {label} poussé avec succès")
        return True
    
    def execute_object_placement(self):
        """Exécuter le placement de tous les objets"""
        rospy.loginfo("=" * 60)
        rospy.loginfo("🚀 DÉMARRAGE DU RANGEMENT PAR POUSSÉE")
        rospy.loginfo("=" * 60)
        
        # Attendre l'initialisation
        rospy.sleep(2.0)
        
        # ÉTAPE 1: Position haute avant de déplacer le robot
        rospy.loginfo("\n📋 ÉTAPE 1: POSITION HAUTE POUR DÉPLACEMENT")
        self.go_to_high_position()
        
        # ÉTAPE 2: Avancer le robot vers la table
        rospy.loginfo("\n📋 ÉTAPE 2: AVANCÉE DU ROBOT")
        self.move_base_toward_table(0.25)  # Avancer de 25cm
        
        # ÉTAPE 3: Position sûre au-dessus de la table
        rospy.loginfo("\n📋 ÉTAPE 3: POSITION SÛRE AU-DESSUS DE LA TABLE")
        self.go_to_safe_position()
        
        # ÉTAPE 4: Ranger les objets
        rospy.loginfo("\n📋 ÉTAPE 4: RANGEMENT DES OBJETS")
        
        # Ordre de traitement
        processing_order = ["fork", "knife", "plate"]
        
        for label in processing_order:
            rospy.loginfo("\n" + "=" * 50)
            rospy.loginfo(f"🎯 TRAITEMENT DE: {label.upper()}")
            rospy.loginfo("=" * 50)
            
            # Trouver les positions
            initial_centroid = None
            final_centroid = None
            
            for obj in self.initial_positions:
                if obj['label'] == label:
                    initial_centroid = (obj['x_pixel'], obj['y_pixel'])
                    break
            
            for obj in self.final_positions:
                if obj['label'] == label:
                    final_centroid = (obj['x_pixel'], obj['y_pixel'])
                    break
            
            if not initial_centroid or not final_centroid:
                rospy.logwarn(f"⚠️  Positions non trouvées pour {label}")
                continue
            
            rospy.loginfo(f"📍 Initial: ({initial_centroid[0]:.1f}, {initial_centroid[1]:.1f})")
            rospy.loginfo(f"📍 Final: ({final_centroid[0]:.1f}, {final_centroid[1]:.1f})")
            
            try:
                # Convertir en positions robot
                start_position = self.pixel_to_robot_position(
                    initial_centroid[0], initial_centroid[1], "push"
                )
                
                end_position = self.pixel_to_robot_position(
                    final_centroid[0], final_centroid[1], "push"
                )
                
                rospy.loginfo(f"📍 Début: X={start_position[0]:.3f}m, Y={start_position[1]:.3f}m")
                rospy.loginfo(f"📍 Fin: X={end_position[0]:.3f}m, Y={end_position[1]:.3f}m")
                
                # Pousser l'objet
                if self.push_object(label, start_position, end_position):
                    rospy.loginfo(f"✅ {label} rangé avec succès!")
                    
                    # Pause entre les objets
                    rospy.sleep(0.5)
                else:
                    rospy.logwarn(f"⚠️  Échec du rangement de {label}")
                    
                    # Pause plus longue après un échec
                    rospy.sleep(1.0)
                    
            except Exception as e:
                rospy.logerr(f"❌ Erreur avec {label}: {e}")
                self.go_to_safe_position()
                rospy.sleep(1.0)
        
        # Mission terminée
        rospy.loginfo("\n" + "=" * 60)
        rospy.loginfo("🎉🎉🎉 MISSION ACCOMPLIE ! 🎉🎉🎉")
        rospy.loginfo("✅ Tous les objets ont été poussés à leur place")
        rospy.loginfo("🤖 Retour à la position haute de sécurité")
        rospy.loginfo("=" * 60)
        
        # Dernière position haute
        self.go_to_high_position()
        
        # Attendre
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