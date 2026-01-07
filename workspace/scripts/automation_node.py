#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Script d'automatisation - Pipeline de réarrangement d'objets avec Tiago

Orchestre 3 étapes séquentielles:
1. Capture d'images (RGB + profondeur)
2. Pipeline IA (détection, génération, positions)
3. Commande robot (déplacement bras)

Usage: rosrun <package> automation_node.py [--mode full|capture|pipeline|command]
"""

import rospy
import subprocess
import sys
import os
import time
import signal
import argparse
from std_msgs.msg import String, Bool
from datetime import datetime

# Configuration

def get_workspace_path():
    """Détermine le chemin du workspace (paramètre ROS > env > défaut)"""
    # 1. Essayer le paramètre ROS si disponible
    try:
        import rospy
        if rospy.has_param('~workspace_path'):
            return rospy.get_param('~workspace_path')
    except Exception:
        pass
    
    # 2. Essayer la variable d'environnement ROS_WORKSPACE
    ros_workspace = os.environ.get('ROS_WORKSPACE')
    if ros_workspace and os.path.exists(ros_workspace):
        return ros_workspace
    
    # 3. Essayer de déduire depuis le chemin du script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    if 'scripts' in script_dir:
        workspace_candidate = os.path.dirname(script_dir)
        if os.path.exists(os.path.join(workspace_candidate, 'pipeline')):
            return workspace_candidate
    
    # 4. Chemin par défaut
    default_path = os.path.expanduser("~/ros_ws/workspace")
    if os.path.exists(default_path):
        return default_path
    
    # 5. Fallback: répertoire parent du script
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Chemins
WORKSPACE_PATH = get_workspace_path()
SCRIPTS_PATH = os.path.join(WORKSPACE_PATH, "scripts")
PIPELINE_PATH = os.path.join(WORKSPACE_PATH, "pipeline")

# Configuration des étapes
STEP_CONFIG = {
    "capture": {
        "script": os.path.join(SCRIPTS_PATH, "capture.py"),
        "name": "Capture d'images",
        "timeout": 60,
        "description": "Capture des images RGB et profondeur via la caméra Xtion"
    },
    "pipeline": {
        "script": os.path.join(PIPELINE_PATH, "run_pipeline.py"),
        "working_dir": PIPELINE_PATH,
        "name": "Pipeline IA",
        "timeout": 2400,
        "description": "Détection d'objets, génération d'image et calcul des positions"
    },
    "command": {
        "script": os.path.join(SCRIPTS_PATH, "command.py"),
        "name": "Commande robot",
        "timeout": 120,
        "description": "Déplacement du bras robotique vers les positions calculées"
    }
}


class TiagoAutomationNode:
    """Nœud ROS pour automatiser le pipeline de réarrangement d'objets."""

    def __init__(self):
        """Initialise le nœud d'automatisation."""
        rospy.init_node('tiago_automation_node', anonymous=False)
        
        # Publishers
        self.status_pub = rospy.Publisher('/tiago/automation_status', String, queue_size=10)
        self.step_pub = rospy.Publisher('/tiago/current_step', String, queue_size=10)
        self.complete_pub = rospy.Publisher('/tiago/automation_complete', Bool, queue_size=10)
        
        # État
        self.current_step = None
        self.is_running = False
        self.start_time = None
        self.step_results = {}
        
        # Signaux
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        rospy.loginfo("=" * 70)
        rospy.loginfo("TIAGO AUTOMATION NODE - INITIALISE")
        rospy.loginfo("=" * 70)
        
        rospy.sleep(1.0)

    def signal_handler(self, sig, frame):
        """Gère l'interruption propre."""
        rospy.logwarn("Signal d'interruption recu, arret en cours...")
        self.is_running = False
        self.publish_status("INTERRUPTED", "Pipeline interrompu par l'utilisateur")
        rospy.signal_shutdown("Interruption utilisateur")
        sys.exit(0)

    def publish_status(self, status, message=""):
        """Publie le statut sur le topic ROS."""
        timestamp = datetime.now().strftime("%H:%M:%S")
        status_msg = String()
        status_msg.data = f"[{timestamp}] {status}: {message}"
        self.status_pub.publish(status_msg)
        
    def publish_step(self, step_name):
        """Publie l'étape en cours."""
        step_msg = String()
        step_msg.data = step_name
        self.step_pub.publish(step_msg)
        self.current_step = step_name

    def print_header(self, title, emoji=""):
        """Affiche un en-tête."""
        rospy.loginfo("")
        rospy.loginfo("=" * 70)
        rospy.loginfo(f" {title}")
        rospy.loginfo("=" * 70)

    def print_step(self, step_num, total_steps, title):
        """Affiche le titre d'une étape."""
        rospy.loginfo("")
        rospy.loginfo("-" * 70)
        rospy.loginfo(f" ETAPE {step_num}/{total_steps} : {title}")
        rospy.loginfo("-" * 70)

    def run_script(self, step_key):
        """Exécute un script Python en sous-processus."""
        config = STEP_CONFIG[step_key]
        script_path = config["script"]
        timeout = config.get("timeout", 120)
        working_dir = config.get("working_dir", os.path.dirname(script_path))
        
        rospy.loginfo(f"Repertoire de travail: {working_dir}")
        rospy.loginfo(f"Script: {script_path}")
        rospy.loginfo(f"Timeout: {timeout}s")
        
        if not os.path.exists(script_path):
            rospy.logerr(f"Script non trouve: {script_path}")
            return False, 0, f"Script non trouvé: {script_path}"
        
        start_time = time.time()
        
        try:
            # Préparer l'environnement avec les sources PAL Robotics
            env = os.environ.copy()
            
            # S'assurer que l'environnement PAL est sourcé
            if 'ROS_PACKAGE_PATH' not in env or 'tiago_description' not in env.get('ROS_PACKAGE_PATH', ''):
                rospy.logwarn("Environnement PAL Robotics non détecté, vérifiez le sourcing")
            
            process = subprocess.Popen(
                ["python3", script_path],
                cwd=working_dir,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                env=env
            )
            
            output_lines = []
            while True:
                line = process.stdout.readline()
                if line:
                    line = line.rstrip()
                    output_lines.append(line)
                    rospy.loginfo(f"  │ {line}")
                    
                if process.poll() is not None:
                    for line in process.stdout.readlines():
                        line = line.rstrip()
                        output_lines.append(line)
                        rospy.loginfo(f"  │ {line}")
                    break
                    
                if time.time() - start_time > timeout:
                    process.terminate()
                    rospy.logerr(f"Timeout apres {timeout}s")
                    return False, time.time() - start_time, "Timeout"
                    
                if rospy.is_shutdown():
                    process.terminate()
                    return False, time.time() - start_time, "ROS shutdown"
            
            duration = time.time() - start_time
            output = "\n".join(output_lines)
            
            if process.returncode == 0:
                rospy.loginfo(f"Succes en {duration:.2f}s")
                return True, duration, output
            else:
                rospy.logerr(f"Echec avec code {process.returncode}")
                return False, duration, output
                
        except Exception as e:
            duration = time.time() - start_time
            rospy.logerr(f"Exception: {e}")
            return False, duration, str(e)

    def execute_step(self, step_num, total_steps, step_key):
        """Exécute une étape du pipeline."""
        config = STEP_CONFIG[step_key]
        step_name = config["name"]
        description = config.get("description", "")
        
        self.print_step(step_num, total_steps, step_name)
        rospy.loginfo(f"{description}")
        rospy.loginfo("")
        
        self.publish_step(step_name)
        self.publish_status("RUNNING", f"Exécution de {step_name}")
        
        success, duration, output = self.run_script(step_key)
        
        self.step_results[step_key] = {
            "success": success,
            "duration": duration,
            "step_name": step_name
        }
        
        if success:
            self.publish_status("SUCCESS", f"{step_name} termine en {duration:.2f}s")
            rospy.loginfo("")
            rospy.loginfo(f"{step_name} - TERMINE avec succes")
            return True
        else:
            self.publish_status("FAILED", f"{step_name} a echoue")
            rospy.logerr("")
            rospy.logerr(f"{step_name} - ECHEC")
            return False

    def run_full_pipeline(self):
        """Exécute le pipeline complet en séquence."""
        self.print_header("DEMARRAGE DU PIPELINE COMPLET")
        
        self.is_running = True
        self.start_time = time.time()
        self.step_results = {}
        
        steps = ["capture", "pipeline", "command"]
        total_steps = len(steps)
        
        rospy.loginfo(f"{total_steps} etapes a executer:")
        for i, step in enumerate(steps, 1):
            rospy.loginfo(f"   {i}. {STEP_CONFIG[step]['name']}")
        rospy.loginfo("")
        
        for i, step_key in enumerate(steps, 1):
            if rospy.is_shutdown():
                rospy.logwarn("Arret ROS detecte")
                return False
                
            success = self.execute_step(i, total_steps, step_key)
            
            if not success:
                rospy.logerr(f"Pipeline arrete a l'etape {i}/{total_steps}")
                self.print_summary(False)
                return False
            
            if i < total_steps:
                rospy.loginfo("Pause avant l'etape suivante...")
                rospy.sleep(2.0)
        
        self.is_running = False
        self.print_summary(True)
        
        complete_msg = Bool()
        complete_msg.data = True
        self.complete_pub.publish(complete_msg)
        
        return True

    def run_single_step(self, step_key):
        """Exécute une seule étape du pipeline."""
        if step_key not in STEP_CONFIG:
            rospy.logerr(f"Etape inconnue: {step_key}")
            return False
            
        self.print_header(f"EXECUTION : {STEP_CONFIG[step_key]['name']}")
        
        self.is_running = True
        self.start_time = time.time()
        self.step_results = {}
        
        success = self.execute_step(1, 1, step_key)
        
        self.is_running = False
        self.print_summary(success)
        
        return success

    def print_summary(self, overall_success):
        """Affiche le résumé de l'exécution."""
        total_time = time.time() - self.start_time if self.start_time else 0
        
        self.print_header("RESUME DE L'EXECUTION")
        
        if self.step_results:
            rospy.loginfo("Temps par etape:")
            rospy.loginfo("-" * 50)
            
            for step_key, result in self.step_results.items():
                status = "OK" if result["success"] else "ECHEC"
                rospy.loginfo(f"  [{status}] {result['step_name']:<30} : {result['duration']:6.2f}s")
            
            rospy.loginfo("-" * 50)
        
        rospy.loginfo(f"Temps total : {total_time:.2f}s")
        rospy.loginfo("")
        
        if overall_success:
            rospy.loginfo("=" * 50)
            rospy.loginfo("PIPELINE TERMINE AVEC SUCCES")
            rospy.loginfo("=" * 50)
            rospy.loginfo("")
            rospy.loginfo("Fichiers generes:")
            rospy.loginfo("   - ~/ros_ws/data/rgb_images/       (images RGB)")
            rospy.loginfo("   - ~/ros_ws/data/depth_images/     (images profondeur)")
            rospy.loginfo("   - ~/ros_ws/workspace/pipeline/outputs/detections_input.json")
            rospy.loginfo("   - ~/ros_ws/workspace/pipeline/outputs/img_generated.png")
            rospy.loginfo("   - ~/ros_ws/workspace/pipeline/outputs/final_positions.json")
        else:
            rospy.logerr("=" * 50)
            rospy.logerr("PIPELINE ECHOUE")
            rospy.logerr("=" * 50)
            rospy.logerr("Verifiez les logs ci-dessus pour identifier le probleme")

    def wait_for_gazebo(self, timeout=30):
        """Attend que Gazebo soit prêt (vérifie les topics ROS)."""
        rospy.loginfo("Verification de la disponibilite de Gazebo...")
        
        required_topics = [
            '/xtion/rgb/image_raw',
            '/xtion/depth_registered/image_raw'
        ]
        
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            if rospy.is_shutdown():
                return False
                
            published_topics = [topic for topic, _ in rospy.get_published_topics()]
            
            missing = [t for t in required_topics if t not in published_topics]
            
            if not missing:
                rospy.loginfo("Gazebo est pret - Tous les topics requis sont disponibles")
                return True
            
            rospy.loginfo(f"En attente des topics: {missing}")
            rospy.sleep(2.0)
        
        rospy.logwarn("Timeout atteint - Gazebo peut ne pas etre completement pret")
        return False


def main():
    """Point d'entrée principal."""
    
    parser = argparse.ArgumentParser(
        description="Script d'automatisation du pipeline de réarrangement d'objets avec Tiago",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Exemples d'utilisation:
  rosrun scripts automation_node.py                  # Pipeline complet
  rosrun scripts automation_node.py --mode capture   # Capture uniquement
  rosrun scripts automation_node.py --mode pipeline  # Pipeline IA uniquement
  rosrun scripts automation_node.py --mode command   # Commande robot uniquement
  rosrun scripts automation_node.py --no-wait        # Sans attendre Gazebo
        """
    )
    
    parser.add_argument(
        '--mode',
        choices=['full', 'capture', 'pipeline', 'command'],
        default='full',
        help='Mode d\'exécution (default: full)'
    )
    
    parser.add_argument(
        '--no-wait',
        action='store_true',
        help='Ne pas attendre que Gazebo soit prêt'
    )
    
    args, _ = parser.parse_known_args()
    
    try:
        node = TiagoAutomationNode()
        
        ros_mode = rospy.get_param('~mode', args.mode)
        ros_wait_for_gazebo = rospy.get_param('~wait_for_gazebo', not args.no_wait)
        
        mode = args.mode if args.mode != 'full' else ros_mode
        wait_for_gazebo = not args.no_wait and ros_wait_for_gazebo
        
        rospy.loginfo(f"Mode d'execution: {mode}")
        rospy.loginfo(f"Attente Gazebo: {wait_for_gazebo}")
        rospy.loginfo(f"Workspace: {WORKSPACE_PATH}")
        
        if wait_for_gazebo and mode in ['full', 'capture']:
            if not node.wait_for_gazebo(timeout=60):
                rospy.logwarn("Gazebo peut ne pas etre pret, tentative de continuer...")
        
        if mode == 'full':
            success = node.run_full_pipeline()
        else:
            success = node.run_single_step(mode)
        
        sys.exit(0 if success else 1)
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Interruption ROS")
        sys.exit(1)
    except Exception as e:
        rospy.logerr(f"Erreur fatale: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
