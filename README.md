# Lancement du projet

## Prérequis
- Un **token Hugging Face** est nécessaire pour charger le modèle de génération d’images.

> Exemple :
> ```bash
> export HUGGINGFACE_HUB_TOKEN="votre_token"
> ```

---

## Démarrage (Terminal 1) : serveur + simulation Gazebo
```bash
./server.sh
./client.sh
cd src
roslaunch tiago_gazebo.launch

./client.sh
cd scripts
python3 automation_node.py
