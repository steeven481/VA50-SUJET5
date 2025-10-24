#!/usr/bin/env python3

import rospy
import cv2
import os
import time
from datetime import datetime
from sensor_msgs.msg import Image
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cv_bridge import CvBridge


class TiagoSimpleImageCapture:
    def __init__(self):
        rospy.init_node('tiago_simple_image_capture', anonymous=True)
        self.bridge = CvBridge()
        self.latest_image = None
        self.image_received = False

        # Dossier de sauvegarde
        self.save_path = os.path.join(os.path.expanduser('~'), 'ros_ws', 'data', 'images')
        os.makedirs(self.save_path, exist_ok=True)

        # S'abonner au topic de la caméra
        self.camera_topic = '/xtion/rgb/image_raw'
        self.sub = rospy.Subscriber(self.camera_topic, Image, self.image_callback)

        # Publishers pour contrôler la tête et le torse
        self.head_pub = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=1)
        self.torso_pub = rospy.Publisher('/torso_controller/command', JointTrajectory, queue_size=1)

        # Attendre que les publishers soient connectés
        rospy.sleep(1.0)

    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_received = True
        except Exception as e:
            rospy.logerr(f"Erreur conversion: {e}")

    def move_head(self, pan=0.0, tilt=-0.5):
        """Déplacer la tête via topic"""
        msg = JointTrajectory()
        msg.joint_names = ['head_1_joint', 'head_2_joint']

        point = JointTrajectoryPoint()
        point.positions = [pan, tilt]
        point.time_from_start = rospy.Duration(2.0)
        msg.points.append(point)

        self.head_pub.publish(msg)
        rospy.loginfo(f"Tête déplacée: pan={pan}, tilt={tilt}")

    def move_torso(self, height=0.15):
        """Déplacer le torse via topic"""
        msg = JointTrajectory()
        msg.joint_names = ['torso_lift_joint']

        point = JointTrajectoryPoint()
        point.positions = [height]
        point.time_from_start = rospy.Duration(3.0)
        msg.points.append(point)

        self.torso_pub.publish(msg)
        rospy.loginfo(f"Torse déplacé: height={height}")

    def capture_panoramic(self):
        """Capturer plusieurs images de la table sous différents angles"""
        positions = [
            # (pan, tilt, torso_height, delay)
            (0.0, -0.7, 0.15, 3.0),  # Devant
            (0.4, -0.6, 0.15, 3.0),  # Droite
            (-0.4, -0.6, 0.15, 3.0),  # Gauche
            (0.0, -0.4, 0.20, 3.0),  # Plus haut
        ]

        for i, (pan, tilt, torso_height, delay) in enumerate(positions):
            rospy.loginfo(f"Position {i + 1}/{len(positions)}")

            # Déplacer le torse
            self.move_torso(torso_height)
            rospy.sleep(2.0)

            # Déplacer la tête
            self.move_head(pan, tilt)
            rospy.sleep(delay)

            # Capturer l'image
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"table_position_{i + 1}_{timestamp}.png"
            self.capture_and_save(filename)

            rospy.sleep(1.0)

    def capture_and_save(self, filename=None):
        """Capturer et sauvegarder une image"""
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"capture_{timestamp}.png"

        # Attendre une image fraîche
        self.image_received = False
        start_time = time.time()

        while not self.image_received and (time.time() - start_time) < 5.0:
            rospy.sleep(0.1)

        if self.latest_image is not None:
            full_path = os.path.join(self.save_path, filename)
            cv2.imwrite(full_path, self.latest_image)
            rospy.loginfo(f"Image sauvegardée: {full_path}")
            return True

        rospy.logwarn("Aucune image capturée")
        return False


def main():
    capture = TiagoSimpleImageCapture()
    rospy.sleep(2.0)  # Initialisation

    rospy.loginfo("Début de la capture panoramique")
    capture.capture_panoramic()
    rospy.loginfo("Capture terminée")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass