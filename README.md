Instructions pour lancer le projet : 
Il faut un token hugging face pour pouvoir charger le modèle de génération de l'image.

./server.sh
./client.sh
cd src
roslaunch tiago_gazebo.launch

créer un autre terminal
./client.sh
cd scripts
python automation_node.py ou ./run_automation.sh
