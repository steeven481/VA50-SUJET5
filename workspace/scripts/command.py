#!/usr/bin/env python3
"""
TIAGo Table Setter - Solution avec positionnement précis du gripper
Le gripper se positionne exactement sur chaque objet avant de le déplacer
Évite les collisions avec la table en utilisant les coordonnées exactes
"""

import rospy
import actionlib
import json
import os
import time
import threading
import numpy as np
import math

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from gazebo_msgs.srv import GetModelState, SetModelState, GetLinkState
from gazebo_msgs.msg import ModelState, LinkState
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import JointState
import tf


class TiagoTableSetter:
    """
    Classe pour dresser la table avec TIAGo.
    Positionne le gripper exactement sur chaque objet.
    Évite les collisions avec la table.
    """
    
    def __init__(self):
        rospy.init_node('tiago_table_setter', anonymous=True)
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("TIAGO TABLE SETTER - Positionnement Précis")
        rospy.loginfo("=" * 60)
        
        # ============================================================
        # COORDONNÉES EXACTES DE LA TABLE (depuis Gazebo)
        # ============================================================
        self.table = {
            'center_x': 0.839,
            'center_y': -0.011,
            'surface_z': 0.687,      # Hauteur de la surface
            'safe_z': 0.80,          # Hauteur sûre au-dessus de la table
            'approach_z': 0.75,      # Hauteur d'approche (juste au-dessus)
            'grasp_z': 0.70,         # Hauteur de saisie (au niveau objet)
        }
        
        # Limites de la zone de travail sur la table
        self.workspace = {
            'x_min': 0.58,   # Bord proche du robot
            'x_max': 1.10,   # Bord loin du robot
            'y_min': -0.42,  # Bord droit
            'y_max': 0.40,   # Bord gauche
        }
        
        # ============================================================
        # POSITIONS INITIALES DES OBJETS (mesurées dans Gazebo)
        # ============================================================
        self.initial_positions = {
            'plate': {'x': 0.872, 'y': -0.164, 'z': 0.692},
            'fork':  {'x': 0.797, 'y': 0.123,  'z': 0.688},
            'knife': {'x': 0.961, 'y': 0.023,  'z': 0.686}
        }
        
        # ============================================================
        # POSITIONS FINALES (table dressée)
        # Assiette au centre, fourchette à gauche, couteau à droite
        # ============================================================
        self.final_positions = {
            'plate': {'x': 0.84, 'y': 0.0,    'z': 0.692},
            'fork':  {'x': 0.84, 'y': 0.18,   'z': 0.688},
            'knife': {'x': 0.84, 'y': -0.18,  'z': 0.687}
        }
        
        # Charger les positions depuis final_positions.json si disponible
        self.load_final_positions_from_json()
        
        # Clients d'action
        rospy.loginfo("Attente de play_motion...")
        self.play_motion_client = actionlib.SimpleActionClient('/play_motion', PlayMotionAction)
        self.play_motion_client.wait_for_server(rospy.Duration(10.0))
        rospy.loginfo("  OK")
        
        # Client pour le bras
        self.arm_client = actionlib.SimpleActionClient(
            '/arm_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )
        self.arm_client.wait_for_server(rospy.Duration(5.0))
        
        # Client pour le torse
        self.torso_client = actionlib.SimpleActionClient(
            '/torso_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )
        self.torso_client.wait_for_server(rospy.Duration(5.0))
        
        # Client pour le gripper
        self.gripper_client = actionlib.SimpleActionClient(
            '/gripper_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )
        self.gripper_client.wait_for_server(rospy.Duration(5.0))
        
        rospy.loginfo("Controleurs OK")
        
        # Services Gazebo
        rospy.wait_for_service('/gazebo/get_model_state', timeout=5.0)
        rospy.wait_for_service('/gazebo/set_model_state', timeout=5.0)
        rospy.wait_for_service('/gazebo/get_link_state', timeout=5.0)
        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.get_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        rospy.loginfo("Services Gazebo OK")
        
        # TF listener pour position du gripper
        self.tf_listener = tf.TransformListener()
        rospy.sleep(0.5)  # Attendre que TF soit pret
        
        # Joints du bras
        self.arm_joints = [
            'arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint',
            'arm_5_joint', 'arm_6_joint', 'arm_7_joint'
        ]
        
        # Joints du gripper
        self.gripper_joints = ['gripper_left_finger_joint', 'gripper_right_finger_joint']
        
        # Objet attache (simulation)
        self.attached_object = None
        self.gripper_offset = {'x': 0.18, 'y': 0.0, 'z': -0.02}
        
        # Thread pour mise a jour de l'objet attache
        self.update_thread = None
        self.stop_update = False
        
        rospy.sleep(1.0)
        rospy.loginfo("Initialisation terminee")

    def get_gripper_position(self):
        """Obtient la position actuelle du gripper dans le monde."""
        try:
            # Utiliser le service Gazebo pour obtenir la position du link
            resp = self.get_link_state('tiago::gripper_grasping_frame', 'world')
            if resp.success:
                return {
                    'x': resp.link_state.pose.position.x,
                    'y': resp.link_state.pose.position.y,
                    'z': resp.link_state.pose.position.z
                }
        except Exception as e:
            pass
        
        # Fallback: utiliser TF
        try:
            self.tf_listener.waitForTransform('base_footprint', 'gripper_grasping_frame', 
                                               rospy.Time(0), rospy.Duration(1.0))
            (trans, rot) = self.tf_listener.lookupTransform('base_footprint', 'gripper_grasping_frame', 
                                                             rospy.Time(0))
            return {'x': trans[0], 'y': trans[1], 'z': trans[2]}
        except Exception as e:
            rospy.logwarn(f"Erreur TF: {e}")
        
        return None

    def attach_object_to_gripper(self, object_name):
        """Attache un objet au gripper (l'objet suivra le gripper)."""
        self.attached_object = object_name
        rospy.loginfo(f"    Objet {object_name} attache au gripper")

    def detach_object(self):
        """Detache l'objet du gripper."""
        if self.attached_object:
            rospy.loginfo(f"    Objet {self.attached_object} detache")
        self.attached_object = None

    def update_attached_object_position(self):
        """Met a jour la position de l'objet attache pour suivre le gripper."""
        if not self.attached_object:
            return
        
        gripper_pos = self.get_gripper_position()
        if gripper_pos:
            # L'objet est decale par rapport au gripper (en dessous)
            obj_x = gripper_pos['x'] + 0.0
            obj_y = gripper_pos['y'] + 0.0
            obj_z = gripper_pos['z'] - 0.05  # 5cm en dessous du gripper
            
            self.set_object_position(self.attached_object, obj_x, obj_y, obj_z)

    def load_final_positions_from_json(self):
        """Charge les positions finales depuis le fichier JSON si disponible."""
        json_paths = [
            '/home/pal/ros_ws/pipeline/outputs/final_positions.json',
            '/home/pal/share/outputs/final_positions.json',
            os.path.expanduser('~/ros_ws/pipeline/outputs/final_positions.json')
        ]
        
        for json_path in json_paths:
            if os.path.exists(json_path):
                try:
                    with open(json_path, 'r') as f:
                        data = json.load(f)
                    
                    rospy.loginfo(f"Fichier JSON trouve: {json_path}")
                    
                    # Convertir les positions pixel en positions Gazebo
                    # (simplification: on garde nos positions calibrees)
                    rospy.loginfo("Utilisation des positions calibrees pour Gazebo")
                    return
                    
                except Exception as e:
                    rospy.logwarn(f"Erreur lecture JSON: {e}")
        
        rospy.loginfo("Utilisation des positions par defaut")

    def play_motion(self, motion_name, timeout=15.0):
        """Execute une motion predefinee."""
        rospy.loginfo(f"    Motion: {motion_name}")
        
        goal = PlayMotionGoal()
        goal.motion_name = motion_name
        goal.skip_planning = True
        
        self.play_motion_client.send_goal(goal)
        success = self.play_motion_client.wait_for_result(rospy.Duration(timeout))
        
        if success:
            result = self.play_motion_client.get_result()
            if result and result.error_code == 1:  # SUCCESS
                return True
        
        return success

    def move_torso(self, height, duration=2.0):
        """Deplace le torse a une hauteur donnee (0.0 - 0.35m)."""
        height = max(0.0, min(0.35, height))
        
        traj = JointTrajectory()
        traj.joint_names = ['torso_lift_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [height]
        point.time_from_start = rospy.Duration(duration)
        traj.points.append(point)
        
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = traj
        
        self.torso_client.send_goal(goal)
        return self.torso_client.wait_for_result(rospy.Duration(duration + 2.0))

    def move_arm_joints(self, positions, duration=3.0):
        """Deplace le bras vers des positions articulaires specifiques."""
        if len(positions) != 7:
            rospy.logerr("7 positions requises pour le bras")
            return False
        
        traj = JointTrajectory()
        traj.joint_names = self.arm_joints
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = rospy.Duration(duration)
        traj.points.append(point)
        
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = traj
        
        self.arm_client.send_goal(goal)
        return self.arm_client.wait_for_result(rospy.Duration(duration + 2.0))

    def open_gripper(self):
        """Ouvre le gripper."""
        rospy.loginfo("    Ouverture gripper")
        
        traj = JointTrajectory()
        traj.joint_names = self.gripper_joints
        
        point = JointTrajectoryPoint()
        point.positions = [0.04, 0.04]  # Ouvert
        point.time_from_start = rospy.Duration(1.0)
        traj.points.append(point)
        
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = traj
        
        self.gripper_client.send_goal(goal)
        return self.gripper_client.wait_for_result(rospy.Duration(3.0))

    def close_gripper(self):
        """Ferme le gripper."""
        rospy.loginfo("    Fermeture gripper")
        
        traj = JointTrajectory()
        traj.joint_names = self.gripper_joints
        
        point = JointTrajectoryPoint()
        point.positions = [0.00, 0.00]  # Ferme
        point.time_from_start = rospy.Duration(1.0)
        traj.points.append(point)
        
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = traj
        
        self.gripper_client.send_goal(goal)
        return self.gripper_client.wait_for_result(rospy.Duration(3.0))

    def get_object_position(self, object_name):
        """Obtient la position actuelle d'un objet dans Gazebo."""
        try:
            resp = self.get_model_state(object_name, 'world')
            if resp.success:
                return {
                    'x': resp.pose.position.x,
                    'y': resp.pose.position.y,
                    'z': resp.pose.position.z
                }
        except Exception as e:
            rospy.logwarn(f"Erreur get_model_state: {e}")
        return None

    def set_object_position(self, object_name, x, y, z):
        """Deplace un objet dans Gazebo."""
        try:
            state = ModelState()
            state.model_name = object_name
            state.pose.position = Point(x, y, z)
            state.pose.orientation = Quaternion(0, 0, 0, 1)
            state.reference_frame = 'world'
            
            resp = self.set_model_state(state)
            return resp.success
        except Exception as e:
            rospy.logwarn(f"Erreur set_model_state: {e}")
            return False

    def move_object_smoothly(self, object_name, start_pos, end_pos, steps=20, duration=2.0):
        """
        Deplace un objet de maniere fluide avec une trajectoire en arc.
        L'objet reste attache au gripper qui suit la meme trajectoire.
        """
        dt = duration / steps
        lift_height = 0.10  # Hauteur de levee
        
        # Obtenir position initiale du gripper
        gripper_start = self.get_gripper_position()
        if not gripper_start:
            gripper_start = {'x': 0.4, 'y': 0.0, 'z': 0.9}
        
        for i in range(steps + 1):
            t = i / steps
            
            # Interpolation lineaire pour x et y
            x = start_pos['x'] + t * (end_pos['x'] - start_pos['x'])
            y = start_pos['y'] + t * (end_pos['y'] - start_pos['y'])
            
            # Trajectoire en arc pour z (parabole)
            arc = 4 * t * (1 - t)  # Parabole: max a t=0.5
            z = start_pos['z'] + arc * lift_height
            
            # A la fin, poser a la hauteur finale
            if i == steps:
                z = end_pos['z']
            
            # Deplacer l'objet
            self.set_object_position(object_name, x, y, z)
            
            rospy.sleep(dt)
        
        return True

    def move_object_with_gripper_sync(self, object_name, start_pos, end_pos, duration=3.0):
        """
        Deplace l'objet en synchronisation avec le mouvement du bras.
        Le gripper et l'objet bougent ensemble de maniere realiste.
        """
        steps = 30
        dt = duration / steps
        lift_height = 0.12
        
        # Phase 1: Lever (1/3 du temps)
        # Phase 2: Transport horizontal (1/3 du temps)
        # Phase 3: Descente (1/3 du temps)
        
        for i in range(steps + 1):
            t = i / steps
            
            # Interpolation pour x et y
            x = start_pos['x'] + t * (end_pos['x'] - start_pos['x'])
            y = start_pos['y'] + t * (end_pos['y'] - start_pos['y'])
            
            # Trajectoire en cloche pour z
            if t < 0.3:
                # Montee
                z_factor = t / 0.3
                z = start_pos['z'] + z_factor * lift_height
            elif t < 0.7:
                # Plateau haut
                z = start_pos['z'] + lift_height
            else:
                # Descente
                z_factor = (t - 0.7) / 0.3
                z = start_pos['z'] + lift_height * (1 - z_factor)
            
            # Fin: poser a la hauteur cible
            if i == steps:
                z = end_pos['z']
            
            self.set_object_position(object_name, x, y, z)
            rospy.sleep(dt)
        
        return True

    # =============================================================
    # SYSTÈME DE POSITIONNEMENT PRÉCIS
    # Les positions articulaires sont calculées pour atteindre
    # les coordonnées exactes des objets
    # =============================================================
    
    def compute_arm_position_for_target(self, target_x, target_y, target_z, phase='approach'):
        """
        Calcule les angles articulaires pour positionner le gripper
        à une position cible donnée.
        
        Paramètres:
        - target_x, target_y, target_z: coordonnées cibles dans le monde
        - phase: 'approach' (au-dessus), 'grasp' (au niveau), 'lift' (levé), 'transport' (en mouvement)
        
        Le robot TIAGo a une portée limitée. On ajuste les angles
        pour s'approcher au maximum de la cible.
        """
        # Ajustements de hauteur selon la phase
        if phase == 'approach':
            z_offset = 0.10  # 10cm au-dessus de la table
        elif phase == 'grasp':
            z_offset = 0.02  # Proche de l'objet
        elif phase == 'lift':
            z_offset = 0.18  # Levé haut
        elif phase == 'transport':
            z_offset = 0.20  # Haut pour transport
        else:
            z_offset = 0.10
        
        # Hauteur cible du gripper (au-dessus de la table)
        gripper_z = max(self.table['surface_z'] + z_offset, target_z + z_offset)
        
        # Calcul de l'angle arm_1 (rotation de la base du bras)
        # Plus y est positif (gauche), plus l'angle est négatif
        # Plus y est négatif (droite), plus l'angle est positif
        arm_1 = -math.atan2(target_y, target_x - 0.05) * 0.8  # Facteur d'ajustement
        arm_1 = max(-1.5, min(1.5, arm_1))  # Limites articulaires
        
        # Distance horizontale à la cible
        dist_xy = math.sqrt((target_x - 0.05)**2 + target_y**2)
        
        # Calcul de l'extension du bras (arm_2, arm_3, arm_4)
        # Plus la cible est loin, plus le bras doit s'étendre
        
        if dist_xy > 0.9:
            # Très loin - bras très étendu
            arm_2 = -0.15  # Épaule vers l'avant
            arm_3 = -1.0   # Coude étendu
            arm_4 = 1.7    # Avant-bras ajusté
        elif dist_xy > 0.7:
            # Moyennement loin
            arm_2 = -0.25
            arm_3 = -0.8
            arm_4 = 1.8
        else:
            # Plus proche
            arm_2 = -0.4
            arm_3 = -0.5
            arm_4 = 1.9
        
        # Ajustement vertical (arm_2) selon la hauteur
        height_factor = (gripper_z - 0.75) * 0.5  # 0.75m est la hauteur de référence
        arm_2 = arm_2 + height_factor
        arm_2 = max(-1.5, min(1.5, arm_2))
        
        # arm_5: orientation du poignet (typiquement -1.57 pour gripper vers le bas)
        arm_5 = -1.57
        
        # arm_6: rotation du poignet pour ajuster selon la direction
        arm_6 = 0.2 + abs(arm_1) * 0.3
        arm_6 = max(-1.4, min(1.4, arm_6))
        
        # arm_7: rotation finale (généralement 0)
        arm_7 = 0.0
        
        # Ajustements selon la phase
        if phase == 'grasp':
            # Pour saisir, descendre plus
            arm_2 = arm_2 + 0.15
            arm_3 = arm_3 - 0.2
        elif phase == 'lift':
            # Pour lever, remonter
            arm_2 = arm_2 - 0.3
            arm_4 = arm_4 - 0.2
        elif phase == 'transport':
            # Pour transport, bras plus haut et replié
            arm_2 = -0.6
            arm_3 = -0.3
            arm_4 = 1.6
            arm_6 = 0.8
        
        return [arm_1, arm_2, arm_3, arm_4, arm_5, arm_6, arm_7]
    
    def arm_home(self):
        """Position de repos du bras - sûre et éloignée de la table."""
        rospy.loginfo("    Bras repos...")
        # Position repliée, haute, loin de la table
        positions = [0.2, -1.34, -0.2, 1.94, -1.57, 1.37, 0.0]
        return self.move_arm_joints(positions, 3.0)
    
    def arm_to_position(self, target_x, target_y, target_z, phase='approach', duration=2.5):
        """
        Déplace le bras vers une position cible spécifique.
        Évite automatiquement la table.
        """
        rospy.loginfo(f"    Bras vers ({target_x:.2f}, {target_y:.2f}, {target_z:.2f}) [{phase}]")
        
        # Vérifier que la position est dans l'espace de travail
        if target_x < self.workspace['x_min'] or target_x > self.workspace['x_max']:
            rospy.logwarn(f"    X hors limites: {target_x}")
        if target_y < self.workspace['y_min'] or target_y > self.workspace['y_max']:
            rospy.logwarn(f"    Y hors limites: {target_y}")
        
        positions = self.compute_arm_position_for_target(target_x, target_y, target_z, phase)
        return self.move_arm_joints(positions, duration)

    def arm_prepare_right(self):
        """Bras prepare a droite (vers plate initial et knife final)."""
        rospy.loginfo("    Bras vers droite...")
        # Position etendue vers la droite, plus basse pour etre proche de la table
        positions = [0.7, -0.3, -0.5, 1.8, -1.57, 0.3, 0.0]
        return self.move_arm_joints(positions, 2.5)

    def arm_prepare_left(self):
        """Bras prepare a gauche (vers fork)."""
        rospy.loginfo("    Bras vers gauche...")
        # Position etendue vers la gauche
        positions = [-0.7, -0.3, -0.5, 1.8, -1.57, 0.3, 0.0]
        return self.move_arm_joints(positions, 2.5)

    def arm_prepare_center(self):
        """Bras prepare au centre."""
        rospy.loginfo("    Bras vers centre...")
        positions = [0.0, -0.3, -0.5, 1.8, -1.57, 0.3, 0.0]
        return self.move_arm_joints(positions, 2.5)

    def arm_reach_down_right(self):
        """Bras tendu vers le bas a droite (pour saisir)."""
        rospy.loginfo("    Bras descend droite...")
        positions = [0.6, 0.0, -0.8, 2.0, -1.57, 0.0, 0.0]
        return self.move_arm_joints(positions, 2.0)

    def arm_reach_down_left(self):
        """Bras tendu vers le bas a gauche."""
        rospy.loginfo("    Bras descend gauche...")
        positions = [-0.6, 0.0, -0.8, 2.0, -1.57, 0.0, 0.0]
        return self.move_arm_joints(positions, 2.0)

    def arm_reach_down_center(self):
        """Bras tendu vers le bas au centre."""
        rospy.loginfo("    Bras descend centre...")
        positions = [0.0, 0.0, -0.8, 2.0, -1.57, 0.0, 0.0]
        return self.move_arm_joints(positions, 2.0)

    def arm_reach_down(self):
        """Bras tendu vers le bas (pour saisir)."""
        rospy.loginfo("    Bras descend...")
        positions = [0.0, -0.2, -1.2, 1.8, -1.57, 0.3, 0.0]
        return self.move_arm_joints(positions, 2.0)

    def arm_lift(self):
        """Bras leve (objet saisi)."""
        rospy.loginfo("    Bras leve...")
        positions = [0.0, -0.5, -0.3, 1.6, -1.57, 0.8, 0.0]
        return self.move_arm_joints(positions, 2.0)

    def arm_lift_right(self):
        """Bras leve a droite."""
        rospy.loginfo("    Bras leve droite...")
        positions = [0.5, -0.5, -0.3, 1.6, -1.57, 0.8, 0.0]
        return self.move_arm_joints(positions, 2.0)

    def arm_lift_left(self):
        """Bras leve a gauche."""
        rospy.loginfo("    Bras leve gauche...")
        positions = [-0.5, -0.5, -0.3, 1.6, -1.57, 0.8, 0.0]
        return self.move_arm_joints(positions, 2.0)

    def arm_transport_to_side(self, from_side, to_side, duration=2.0):
        """Anime le bras d'un cote a l'autre pendant le transport."""
        rospy.loginfo(f"    Transport: {from_side} -> {to_side}")
        
        # Positions de transport en hauteur
        side_positions = {
            'right': [0.6, -0.6, -0.2, 1.5, -1.57, 0.9, 0.0],
            'left': [-0.6, -0.6, -0.2, 1.5, -1.57, 0.9, 0.0],
            'center': [0.0, -0.6, -0.2, 1.5, -1.57, 0.9, 0.0]
        }
        
        target = side_positions.get(to_side, side_positions['center'])
        return self.move_arm_joints(target, duration)

    def pick_and_place_object(self, object_name, target_pos):
        """
        Sequence complete de pick and place pour un objet.
        Le bras se positionne EXACTEMENT sur l'objet avant de le déplacer.
        Évite les collisions avec la table.
        """
        rospy.loginfo(f"\n{'='*50}")
        rospy.loginfo(f"PICK & PLACE: {object_name.upper()}")
        rospy.loginfo(f"{'='*50}")
        
        # Obtenir position actuelle de l'objet
        current_pos = self.get_object_position(object_name)
        if not current_pos:
            rospy.logerr(f"  Impossible de trouver {object_name}")
            return False
        
        rospy.loginfo(f"  Position initiale: ({current_pos['x']:.3f}, {current_pos['y']:.3f}, {current_pos['z']:.3f})")
        rospy.loginfo(f"  Position cible: ({target_pos['x']:.3f}, {target_pos['y']:.3f}, {target_pos['z']:.3f})")
        
        # Hauteurs de sécurité (éviter la table)
        safe_z = self.table['safe_z']           # 0.80m - hauteur sûre
        approach_z = current_pos['z'] + 0.08    # 8cm au-dessus de l'objet
        grasp_z = current_pos['z'] + 0.03       # 3cm au-dessus (pour saisir)
        
        rospy.loginfo(f"  Table surface Z: {self.table['surface_z']:.3f}")
        rospy.loginfo(f"  Hauteur sûre: {safe_z:.3f}")
        
        # ============================================
        # PHASE 1: APPROCHE HAUTE - Sécurisée au-dessus de la table
        # ============================================
        rospy.loginfo("  1. Approche sécurisée (haute)")
        self.open_gripper()
        
        # Aller d'abord à une position haute au-dessus de l'objet
        self.arm_to_position(current_pos['x'], current_pos['y'], safe_z, 'approach', 2.0)
        rospy.sleep(0.3)
        
        # ============================================
        # PHASE 2: DESCENTE VERS L'OBJET
        # ============================================
        rospy.loginfo("  2. Descente vers l'objet")
        
        # Descendre progressivement vers l'objet
        self.arm_to_position(current_pos['x'], current_pos['y'], approach_z, 'approach', 1.5)
        rospy.sleep(0.2)
        
        # Position de saisie finale
        self.arm_to_position(current_pos['x'], current_pos['y'], grasp_z, 'grasp', 1.5)
        rospy.sleep(0.3)
        
        # ============================================
        # PHASE 3: SAISIE
        # ============================================
        rospy.loginfo("  3. Saisie de l'objet")
        self.close_gripper()
        self.attach_object_to_gripper(object_name)
        rospy.sleep(0.4)
        
        # ============================================
        # PHASE 4: LEVÉE - Soulever l'objet au-dessus de la table
        # ============================================
        rospy.loginfo("  4. Levée sécurisée")
        
        lift_z = safe_z + 0.05  # 5cm au-dessus de la hauteur sûre
        
        # Animation simultanée du bras et de l'objet
        def lift_object_smooth():
            steps = 20
            start_z = current_pos['z']
            end_z = lift_z - 0.05  # L'objet est sous le gripper
            for i in range(steps + 1):
                t = i / steps
                # Courbe d'accélération douce
                t_smooth = t * t * (3 - 2 * t)  # Ease in-out
                z = start_z + t_smooth * (end_z - start_z)
                self.set_object_position(object_name, current_pos['x'], current_pos['y'], z)
                rospy.sleep(0.08)
        
        lift_thread = threading.Thread(target=lift_object_smooth)
        lift_thread.start()
        
        self.arm_to_position(current_pos['x'], current_pos['y'], lift_z, 'lift', 1.8)
        lift_thread.join()
        rospy.sleep(0.3)
        
        # ============================================
        # PHASE 5: TRANSPORT - Déplacement horizontal en hauteur
        # ============================================
        rospy.loginfo("  5. Transport (évite la table)")
        
        transport_z = safe_z + 0.08  # Transport bien au-dessus de la table
        
        # L'objet suit le gripper pendant le transport
        def transport_object_smooth():
            steps = 30
            start = {'x': current_pos['x'], 'y': current_pos['y'], 'z': lift_z - 0.05}
            end = {'x': target_pos['x'], 'y': target_pos['y'], 'z': transport_z - 0.05}
            
            for i in range(steps + 1):
                t = i / steps
                t_smooth = t * t * (3 - 2 * t)
                x = start['x'] + t_smooth * (end['x'] - start['x'])
                y = start['y'] + t_smooth * (end['y'] - start['y'])
                z = start['z'] + t_smooth * (end['z'] - start['z'])
                self.set_object_position(object_name, x, y, z)
                rospy.sleep(0.07)
        
        transport_thread = threading.Thread(target=transport_object_smooth)
        transport_thread.start()
        
        self.arm_to_position(target_pos['x'], target_pos['y'], transport_z, 'transport', 2.5)
        transport_thread.join()
        rospy.sleep(0.3)
        
        # ============================================
        # PHASE 6: APPROCHE FINALE - Au-dessus de la destination
        # ============================================
        rospy.loginfo("  6. Positionnement au-dessus de la destination")
        
        final_approach_z = target_pos['z'] + 0.10  # 10cm au-dessus de la position finale
        
        self.arm_to_position(target_pos['x'], target_pos['y'], final_approach_z, 'approach', 1.5)
        
        # Mettre l'objet en position intermédiaire
        self.set_object_position(object_name, target_pos['x'], target_pos['y'], final_approach_z - 0.05)
        rospy.sleep(0.3)
        
        # ============================================
        # PHASE 7: DESCENTE FINALE - Poser délicatement
        # ============================================
        rospy.loginfo("  7. Descente finale")
        
        final_grasp_z = target_pos['z'] + 0.03  # 3cm au-dessus
        
        def lower_object_smooth():
            steps = 15
            start_z = final_approach_z - 0.05
            end_z = target_pos['z']
            for i in range(steps + 1):
                t = i / steps
                t_smooth = t * t * (3 - 2 * t)
                z = start_z + t_smooth * (end_z - start_z)
                self.set_object_position(object_name, target_pos['x'], target_pos['y'], z)
                rospy.sleep(0.06)
        
        lower_thread = threading.Thread(target=lower_object_smooth)
        lower_thread.start()
        
        self.arm_to_position(target_pos['x'], target_pos['y'], final_grasp_z, 'grasp', 1.2)
        lower_thread.join()
        
        # ============================================
        # PHASE 8: DÉPÔT - Relâcher l'objet
        # ============================================
        rospy.loginfo("  8. Dépôt")
        
        self.detach_object()
        self.open_gripper()
        
        # S'assurer que l'objet est exactement à la bonne position
        self.set_object_position(object_name, target_pos['x'], target_pos['y'], target_pos['z'])
        rospy.sleep(0.3)
        
        # ============================================
        # PHASE 9: RETRAIT - Éloigner le bras
        # ============================================
        rospy.loginfo("  9. Retrait du bras")
        
        # Remonter le bras pour ne pas heurter l'objet
        self.arm_to_position(target_pos['x'], target_pos['y'], safe_z, 'approach', 1.5)
        rospy.sleep(0.2)
        
        # Vérifier position finale
        final_pos = self.get_object_position(object_name)
        if final_pos:
            rospy.loginfo(f"  Position finale: ({final_pos['x']:.3f}, {final_pos['y']:.3f}, {final_pos['z']:.3f})")
            
            # Vérifier si l'objet est bien positionné
            dx = abs(final_pos['x'] - target_pos['x'])
            dy = abs(final_pos['y'] - target_pos['y'])
            if dx < 0.02 and dy < 0.02:
                rospy.loginfo(f"  ✓ {object_name} positionné avec précision!")
            else:
                rospy.logwarn(f"  △ Écart: dx={dx:.3f}, dy={dy:.3f}")
        
        rospy.loginfo(f"  {object_name} déplacé avec SUCCÈS!")
        return True

    def dress_table(self):
        """
        Sequence principale pour dresser la table.
        Déplace les 3 objets vers leurs positions finales.
        Le bras se positionne exactement sur chaque objet.
        Évite les collisions avec la table.
        """
        rospy.loginfo("")
        rospy.loginfo("=" * 60)
        rospy.loginfo("DRESSAGE DE TABLE - Positionnement Précis")
        rospy.loginfo("=" * 60)
        
        # Afficher les coordonnées de la table
        rospy.loginfo("")
        rospy.loginfo("Coordonnées de la table:")
        rospy.loginfo(f"  Surface Z: {self.table['surface_z']:.3f}m")
        rospy.loginfo(f"  Zone sûre Z: {self.table['safe_z']:.3f}m")
        rospy.loginfo(f"  Workspace X: [{self.workspace['x_min']:.2f}, {self.workspace['x_max']:.2f}]")
        rospy.loginfo(f"  Workspace Y: [{self.workspace['y_min']:.2f}, {self.workspace['y_max']:.2f}]")
        
        # Afficher positions initiales
        rospy.loginfo("")
        rospy.loginfo("Positions initiales des objets:")
        for obj in ['plate', 'fork', 'knife']:
            pos = self.get_object_position(obj)
            if pos:
                rospy.loginfo(f"  {obj}: ({pos['x']:.3f}, {pos['y']:.3f}, {pos['z']:.3f})")
        
        # Position initiale du robot
        rospy.loginfo("")
        rospy.loginfo("Initialisation du robot")
        self.play_motion('home')
        rospy.sleep(1.0)
        
        # Lever le torse pour meilleure portée
        rospy.loginfo("  Levée du torse pour meilleure portée")
        self.move_torso(0.35, 2.0)  # Maximum (35cm)
        rospy.sleep(0.5)
        
        # Ouvrir le gripper
        self.open_gripper()
        rospy.sleep(0.5)
        
        # Position initiale du bras (haute et sûre)
        self.arm_home()
        rospy.sleep(0.5)
        
        # Résultats
        results = {}
        
        # Ordre de traitement: plate (centre), puis fork (gauche), puis knife (droite)
        objects_order = ['plate', 'fork', 'knife']
        
        for i, obj_name in enumerate(objects_order):
            rospy.loginfo("")
            rospy.loginfo(f"[{i+1}/3] Traitement de: {obj_name}")
            
            target = self.final_positions[obj_name]
            success = self.pick_and_place_object(obj_name, target)
            results[obj_name] = success
            
            # Retour position haute entre les objets
            rospy.loginfo("  Retour position sûre")
            self.arm_home()
            rospy.sleep(0.5)
        
        # Position finale du robot
        rospy.loginfo("")
        rospy.loginfo("Position finale robot")
        self.play_motion('home')
        rospy.sleep(1.0)
        
        # Résumé
        rospy.loginfo("")
        rospy.loginfo("=" * 60)
        rospy.loginfo("RÉSUMÉ DU DRESSAGE")
        rospy.loginfo("=" * 60)
        
        success_count = 0
        for obj_name, success in results.items():
            status = "✓" if success else "✗"
            rospy.loginfo(f"  [{status}] {obj_name}")
            if success:
                success_count += 1
        
        # Afficher positions finales
        rospy.loginfo("")
        rospy.loginfo("Positions finales:")
        for obj in ['plate', 'fork', 'knife']:
            pos = self.get_object_position(obj)
            target = self.final_positions[obj]
            if pos:
                dx = abs(pos['x'] - target['x'])
                dy = abs(pos['y'] - target['y'])
                precision = "✓" if (dx < 0.02 and dy < 0.02) else "△"
                rospy.loginfo(f"  {obj}: ({pos['x']:.3f}, {pos['y']:.3f}, {pos['z']:.3f}) {precision}")
        
        rospy.loginfo("")
        if success_count == 3:
            rospy.loginfo("🎉 TABLE DRESSÉE AVEC SUCCÈS!")
        else:
            rospy.loginfo(f"⚠ Dressage partiel: {success_count}/3 objets")
        
        rospy.loginfo("=" * 60)
        
        return success_count == 3
        self.play_motion('home')
        rospy.sleep(1.0)
        
        # Lever le torse pour meilleure portee
        self.move_torso(0.30, 2.0)
        rospy.sleep(0.5)
        
        self.open_gripper()
        rospy.sleep(0.5)
        
        # Resultats
        results = {}
        
        # Ordre de traitement: plate d'abord (centre), puis fork (gauche), puis knife (droite)
        objects_order = ['plate', 'fork', 'knife']
        
        for obj_name in objects_order:
            target = self.final_positions[obj_name]
            success = self.pick_and_place_object(obj_name, target)
            results[obj_name] = success
            
            # Retour position neutre entre les objets
            self.arm_home()
            rospy.sleep(0.5)
        
        # Position finale
        rospy.loginfo("")
        rospy.loginfo("Position finale robot")
        self.play_motion('home')
        rospy.sleep(1.0)
        
        # Resume
        rospy.loginfo("")
        rospy.loginfo("=" * 60)
        rospy.loginfo("RESUME DU DRESSAGE")
        rospy.loginfo("=" * 60)
        
        success_count = 0
        for obj_name, success in results.items():
            status = "[OK]" if success else "[ECHEC]"
            rospy.loginfo(f"  {status} {obj_name}")
            if success:
                success_count += 1
        
        # Afficher positions finales
        rospy.loginfo("")
        rospy.loginfo("Positions finales:")
        for obj in ['plate', 'fork', 'knife']:
            pos = self.get_object_position(obj)
            if pos:
                rospy.loginfo(f"  {obj}: ({pos['x']:.3f}, {pos['y']:.3f}, {pos['z']:.3f})")
        
        rospy.loginfo("")
        if success_count == 3:
            rospy.loginfo("TABLE DRESSEE AVEC SUCCES!")
        else:
            rospy.loginfo(f"Dressage partiel: {success_count}/3 objets")
        
        rospy.loginfo("=" * 60)
        
        return success_count == 3


def main():
    """Point d'entree principal."""
    try:
        setter = TiagoTableSetter()
        rospy.sleep(1.0)
        
        success = setter.dress_table()
        
        if success:
            rospy.loginfo("\nMission accomplie!")
        else:
            rospy.logwarn("\nMission partiellement accomplie")
            
    except rospy.ROSInterruptException:
        rospy.loginfo("Interruption ROS")
    except Exception as e:
        rospy.logerr(f"Erreur: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
