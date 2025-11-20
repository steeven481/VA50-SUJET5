#!/usr/bin/env python
import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import copy

def main():
    # Initialisation de moveit_commander et du node ROS
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('tiago_arm_push', anonymous=True)

    # Création des interfaces RobotCommander et PlanningSceneInterface (facultatif ici)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    # Création du MoveGroupCommander pour le groupe 'arm_torso'
    arm = moveit_commander.MoveGroupCommander("arm_torso")

    # Définition des poses cibles (positions x,y ajustables ; z fixé à 0.85 m)
    # Fourchette (à ajuster en X et Y selon la position réelle de l'objet)
    pose_fork = geometry_msgs.msg.Pose()
    pose_fork.position.x = 0.6   # X de la fourchette (à ajuster)
    pose_fork.position.y = 0.2   # Y de la fourchette (à ajuster)
    pose_fork.position.z = 0.85
    pose_fork.orientation.x = 0.0
    pose_fork.orientation.y = 0.0
    pose_fork.orientation.z = 0.0
    pose_fork.orientation.w = 1.0  # orientation (pas de rotation) à ajuster si besoin

    # Couteau (à ajuster)
    pose_knife = geometry_msgs.msg.Pose()
    pose_knife.position.x = 0.6   # X du couteau (à ajuster)
    pose_knife.position.y = 0.0   # Y du couteau (à ajuster)
    pose_knife.position.z = 0.85
    pose_knife.orientation.x = 0.0
    pose_knife.orientation.y = 0.0
    pose_knife.orientation.z = 0.0
    pose_knife.orientation.w = 1.0

    # Assiette (à ajuster)
    pose_plate = geometry_msgs.msg.Pose()
    pose_plate.position.x = 0.6   # X de l'assiette (à ajuster)
    pose_plate.position.y = -0.2  # Y de l'assiette (à ajuster)
    pose_plate.position.z = 0.85
    pose_plate.orientation.x = 0.0
    pose_plate.orientation.y = 0.0
    pose_plate.orientation.z = 0.0
    pose_plate.orientation.w = 1.0

    # Liste des poses cibles à atteindre (fourchette, couteau, assiette)
    poses = [pose_fork, pose_knife, pose_plate]

    for pose in poses:
        # 1) Déplacement au-dessus de l'objet
        arm.set_pose_target(pose)
        arm.go(wait=True)  # exécute le plan de mouvement vers la cible:contentReference[oaicite:8]{index=8}
        arm.stop()
        arm.clear_pose_targets()
        rospy.sleep(0.5)  # courte pause

        # 2) Mouvement en avant de 10 cm pour pousser l'objet
        push_pose = copy.deepcopy(pose)
        push_pose.position.x += 0.10  # on avance de 0.10 m sur l'axe X
        arm.set_pose_target(push_pose)
        arm.go(wait=True)
        arm.stop()
        arm.clear_pose_targets()

        rospy.sleep(1.0)  # pause entre chaque action

    # Arrêt propre de MoveIt et du noeud ROS
    moveit_commander.roscpp_shutdown()  # fermeture de MoveIt:contentReference[oaicite:9]{index=9}
    rospy.signal_shutdown("Fin de la séquence")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
