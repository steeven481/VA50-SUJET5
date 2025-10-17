#!/usr/bin/env python3

import rospy
import cv2
import os
import time
from datetime import datetime
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class TiagoImageCapture:
    def __init__(self):
        rospy.init_node('tiago_image_capture', anonymous=True)
        self.bridge = CvBridge()
        self.latest_image = None
        self.image_received = False

        # Dossier de sauvegarde
        self.save_path = os.path.join(os.path.expanduser('~'), 'ros_ws', 'data', 'images')
        os.makedirs(self.save_path, exist_ok=True)
        rospy.loginfo(f"📁 Dossier de sauvegarde: {self.save_path}")

        # S'abonner au topic spécifique de TIAGo
        self.camera_topic = '/xtion/rgb/image_raw'
        rospy.loginfo(f"🔍 Abonnement au topic: {self.camera_topic}")
        self.sub = rospy.Subscriber(self.camera_topic, Image, self.image_callback)

    def image_callback(self, msg):
        """Réception des images de la caméra"""
        try:
            # Conversion de l'image ROS en image OpenCV
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_received = True
            rospy.loginfo("📸 Image reçue avec succès")
        except Exception as e:
            rospy.logerr(f"❌ Erreur de conversion: {e}")

    def capture_image(self, timeout=10):
        """Capturer une image avec timeout"""
        rospy.loginfo(f"⏳ Attente d'une image (timeout: {timeout}s)...")

        # Réinitialiser le flag
        self.image_received = False
        self.latest_image = None

        # Attendre une image
        start_time = time.time()
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown() and (time.time() - start_time) < timeout:
            if self.image_received and self.latest_image is not None:
                return self.save_image()
            rate.sleep()

        rospy.logerr(f"❌ Timeout après {timeout} secondes")
        return False

    def save_image(self, filename=None):
        """Sauvegarder l'image capturée"""
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"tiago_capture_{timestamp}.png"

        full_path = os.path.join(self.save_path, filename)

        if self.latest_image is not None:
            success = cv2.imwrite(full_path, self.latest_image)
            if success:
                height, width, channels = self.latest_image.shape
                rospy.loginfo(f"💾 Image sauvegardée: {full_path}")
                rospy.loginfo(f"📐 Taille: {width}x{height}, Canaux: {channels}")
                return True
            else:
                rospy.logerr("❌ Échec de l'écriture de l'image")
                return False
        else:
            rospy.logwarn("⚠️  Aucune image disponible pour sauvegarde")
            return False

def main():
    rospy.loginfo("🚀 Démarrage du captureur d'images TIAGo...")

    # Créer le captureur
    capture = TiagoImageCapture()

    # Petite pause pour l'initialisation
    rospy.sleep(1.0)

    # Capturer une image
    if capture.capture_image(timeout=10):
        rospy.loginfo("✅ Capture et sauvegarde réussies!")
    else:
        rospy.logerr("❌ Échec de la capture")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Interruption du programme")