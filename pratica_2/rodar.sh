#!/bin/bash

# Script para lançar todos os terminais da prática de robótica
# Intervalo de 5 segundos entre Gazebo e RViz

# Função para abrir terminal com comando
open_terminal() {
    gnome-terminal --tab --title="$1" -- bash -c "echo 'Executando: $2'; $2; exec bash"
}


echo "Matando processos antigos do Gazebo e ROS..."
# Mata todos os processos antigos que podem interferir
pkill -f gazebo
pkill -f gzserver
pkill -f gzclient
pkill -f ros2
pkill -f rviz2
sleep 2

echo "Iniciando lançamento dos terminais..."

# Terminal 1: Description
open_terminal "Description" "source install/setup.bash && ros2 launch robotics_class robot_description.launch.py"

sleep 2

# Terminal 2: Simulation (Gazebo)
open_terminal "Simulation" "source install/setup.bash && ros2 launch robotics_class simulation_world.launch.py"

echo "Aguardando 5 segundos para Gazebo inicializar..."
sleep 10 # Espera crítica para o Gazebo

# Terminal 3: EKF
open_terminal "EKF" "source install/setup.bash && ros2 launch robotics_class ekf.launch.py"

sleep 2

# Terminal 4: RViz
open_terminal "RViz" "source install/setup.bash && ros2 launch robotics_class rviz.launch.py"

sleep 2

# Terminal 5: Teleop
open_terminal "Teleop" "source install/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/jetauto/cmd_vel"

sleep 5

# Terminal 6: Localization
open_terminal "Localization" "source install/setup.bash && ros2 launch robotics_class localization.launch.py map:=./class_map.yaml"

sleep 10

# Terminal 7: NAVIGATION 
open_terminal "NAVIGATION" "source install/setup.bash && ros2 launch robotics_class navigation.launch.py"

echo "Todos os terminais foram abertos!"
echo "Aguardem 10-15 segundos para tudo inicializar completamente."
echo "Depois execute no Terminal 7: ros2 launch robotics_class navigation.launch.py"
