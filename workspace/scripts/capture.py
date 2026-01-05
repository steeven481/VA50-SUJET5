#!/usr/bin/env python3

import rospy
import cv2
import os
import time
import numpy as np
from datetime import datetime
from sensor_msgs.msg import Image
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cv_bridge import CvBridge


class TiagoDepthCapture:
    def __init__(self):
        rospy.init_node('tiago_depth_capture', anonymous=True)
        self.bridge = CvBridge()

        # Images RGB et profondeur
        self.rgb_image = None
        self.depth_image = None
        self.rgb_received = False
        self.depth_received = False

        # Dossiers de sauvegarde
        self.base_path = os.path.join(os.path.expanduser('~'), 'ros_ws', 'data')
        self.rgb_path = os.path.join(self.base_path, 'rgb_images')
        self.depth_path = os.path.join(self.base_path, 'depth_images')
        self.calibration_path = os.path.join(self.base_path, 'calibration')

        os.makedirs(self.rgb_path, exist_ok=True)
        os.makedirs(self.depth_path, exist_ok=True)
        os.makedirs(self.calibration_path, exist_ok=True)

        # S'abonner aux topics
        self.rgb_sub = rospy.Subscriber('/xtion/rgb/image_raw', Image, self.rgb_callback)
        self.depth_sub = rospy.Subscriber('/xtion/depth_registered/image_raw', Image, self.depth_callback)

        # Publishers pour contrôler la tête et le torse
        self.head_pub = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=1)
        self.torso_pub = rospy.Publisher('/torso_controller/command', JointTrajectory, queue_size=1)

        rospy.sleep(1.0)

        # Informations de calibration de la caméra (à adapter selon votre setup)
        self.camera_info = {
            'fx': 525.0,  # Focale X
            'fy': 525.0,  # Focale Y
            'cx': 320.0,  # Centre X
            'cy': 240.0,  # Centre Y
            'width': 640,
            'height': 480
        }

        self.save_camera_calibration()

    def rgb_callback(self, msg):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.rgb_received = True
        except Exception as e:
            rospy.logerr(f"Erreur conversion RGB: {e}")

    def depth_callback(self, msg):
        try:
            # Convertir en format 32FC1 (float32, 1 canal)
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
            self.depth_received = True
        except Exception as e:
            rospy.logerr(f"Erreur conversion profondeur: {e}")

    def save_camera_calibration(self):
        """Sauvegarder les paramètres de calibration de la caméra"""
        calib_file = os.path.join(self.calibration_path, 'camera_calibration.json')
        import json
        with open(calib_file, 'w') as f:
            json.dump(self.camera_info, f, indent=2)
        rospy.loginfo(f"✅ Calibration caméra sauvegardée: {calib_file}")

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

    def capture_scene(self):
        """Capturer une scène avec RGB et profondeur"""
        rospy.loginfo("🎯 Capture de la scène...")

        # Position par défaut pour une vue de la table
        self.move_torso(0.15)
        rospy.sleep(2.0)

        self.move_head(0.0, -0.7)  # Regarder vers la table
        rospy.sleep(3.0)

        # Attendre que les images soient reçues
        self.rgb_received = False
        self.depth_received = False
        start_time = time.time()

        while (not self.rgb_received or not self.depth_received) and (time.time() - start_time) < 5.0:
            rospy.sleep(0.1)

        if self.rgb_image is not None and self.depth_image is not None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

            # Sauvegarder RGB
            rgb_filename = f"scene_{timestamp}_rgb.png"
            rgb_path = os.path.join(self.rgb_path, rgb_filename)
            cv2.imwrite(rgb_path, self.rgb_image)

            # Sauvegarder profondeur (format 16-bit PNG pour meilleure précision)
            depth_filename = f"scene_{timestamp}_depth.png"
            depth_path = os.path.join(self.depth_path, depth_filename)

            # Convertir en format 16-bit (0-65535) pour sauvegarde
            depth_normalized = cv2.normalize(self.depth_image, None, 0, 65535, cv2.NORM_MINMAX)
            depth_16bit = np.uint16(depth_normalized)
            cv2.imwrite(depth_path, depth_16bit)

            # Sauvegarder aussi en format numpy pour les valeurs exactes
            npz_filename = f"scene_{timestamp}_depth.npz"
            npz_path = os.path.join(self.depth_path, npz_filename)
            np.savez_compressed(npz_path, depth=self.depth_image)

            # Sauvegarder les métadonnées
            self.save_metadata(timestamp, 0.0, -0.7, 0.15, rgb_filename, depth_filename)

            rospy.loginfo(f"✅ Scène capturée et sauvegardée: {timestamp}")
            rospy.loginfo(f"   RGB: {rgb_path}")
            rospy.loginfo(f"   Profondeur: {depth_path}")
            rospy.loginfo(f"   Données brutes: {npz_path}")

            # Afficher les statistiques de profondeur
            self.analyze_depth()

            return timestamp, rgb_path, depth_path, npz_path
        else:
            rospy.logerr("❌ Échec de la capture")
            return None

    def analyze_depth(self):
        """Analyser et afficher les statistiques de l'image de profondeur"""
        if self.depth_image is not None:
            valid_pixels = self.depth_image[self.depth_image > 0]
            if len(valid_pixels) > 0:
                rospy.loginfo(f"📊 Statistiques profondeur:")
                rospy.loginfo(f"   Min: {np.min(valid_pixels):.3f} m")
                rospy.loginfo(f"   Max: {np.max(valid_pixels):.3f} m")
                rospy.loginfo(f"   Moyenne: {np.mean(valid_pixels):.3f} m")
                rospy.loginfo(f"   Médiane: {np.median(valid_pixels):.3f} m")
                rospy.loginfo(f"   Pixels valides: {len(valid_pixels)}/{self.depth_image.size}")

    def save_metadata(self, timestamp, pan, tilt, torso_height, rgb_file, depth_file):
        """Sauvegarder les métadonnées de la capture"""
        metadata = {
            'timestamp': timestamp,
            'camera_position': {
                'head_pan': pan,
                'head_tilt': tilt,
                'torso_height': torso_height
            },
            'files': {
                'rgb': rgb_file,
                'depth': depth_file
            },
            'camera_calibration': self.camera_info
        }

        import json
        meta_file = os.path.join(self.calibration_path, f'metadata_{timestamp}.json')
        with open(meta_file, 'w') as f:
            json.dump(metadata, f, indent=2)

        rospy.loginfo(f"📄 Métadonnées sauvegardées: {meta_file}")

    def capture_panoramic(self):
        """Capturer plusieurs angles de vue"""
        positions = [
            # (pan, tilt, torso_height, delay, nom)
            (0.0, -0.7, 0.15, 3.0, "front"),
            (0.4, -0.6, 0.15, 3.0, "right"),
            (-0.4, -0.6, 0.15, 3.0, "left"),
            (0.0, -0.4, 0.20, 3.0, "up"),
        ]

        for pan, tilt, torso_height, delay, name in positions:
            rospy.loginfo(f"🎯 Position: {name}")

            # Déplacer
            self.move_torso(torso_height)
            rospy.sleep(2.0)
            self.move_head(pan, tilt)
            rospy.sleep(delay)

            # Capturer
            self.capture_scene()
            rospy.sleep(1.0)

    def get_depth_at_pixel(self, pixel_x, pixel_y):
        """Obtenir la valeur de profondeur à un pixel spécifique"""
        if self.depth_image is not None:
            x = int(pixel_x)
            y = int(pixel_y)

            if 0 <= x < self.depth_image.shape[1] and 0 <= y < self.depth_image.shape[0]:
                depth = self.depth_image[y, x]

                if depth > 0 and not np.isnan(depth):
                    # Vérifier la cohérence avec les pixels voisins
                    neighborhood = self.depth_image[
                        max(0, y - 2):min(self.depth_image.shape[0], y + 2),
                        max(0, x - 2):min(self.depth_image.shape[1], x + 2)
                    ]
                    neighborhood = neighborhood[neighborhood > 0]

                    if len(neighborhood) > 0:
                        # Utiliser la médiane pour éviter les outliers
                        median_depth = np.median(neighborhood)
                        rospy.loginfo(
                            f"📏 Profondeur à ({x}, {y}): {depth:.3f} m (médiane locale: {median_depth:.3f} m)")
                        return median_depth
                    else:
                        rospy.loginfo(f"📏 Profondeur à ({x}, {y}): {depth:.3f} m")
                        return depth
                else:
                    rospy.logwarn(f"⚠️  Profondeur invalide à ({x}, {y}): {depth}")
            else:
                rospy.logerr(f"❌ Coordonnées hors limites: ({x}, {y})")

        return None

    def pixel_to_3d(self, pixel_x, pixel_y, depth):
        """Convertir un pixel + profondeur en coordonnées 3D (repère caméra)"""
        if depth is None or depth <= 0:
            return None

        fx = self.camera_info['fx']
        fy = self.camera_info['fy']
        cx = self.camera_info['cx']
        cy = self.camera_info['cy']

        # Formule: X = (x - cx) * Z / fx
        #          Y = (y - cy) * Z / fy
        #          Z = depth
        x = (pixel_x - cx) * depth / fx
        y = (pixel_y - cy) * depth / fy
        z = depth

        return (x, y, z)


def main():
    capture = TiagoDepthCapture()
    rospy.sleep(2.0)

    rospy.loginfo("🚀 Début de la capture avec profondeur")

    # Option 1: Capture simple d'une scène
    capture.capture_scene()

    # Option 2: Capture panoramique (décommentez si besoin)
    # capture.capture_panoramic()

    rospy.loginfo("✅ Capture terminée")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass