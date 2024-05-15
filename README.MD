# Panda clik demo

[![N|Solid](https://www.unicampania.it/doc/img/logo_vanvitelli.jpg)](https://www.ingegneria.unicampania.it/roboticslab)

Demo rete ros con generatori di traiettoria e inversione cinematica (CLIK) per il corso di Programmazione dei Robot.

## Install

- Prerequisiti: Installare Moveit!

- Installare i questo pacchetto e il simulatore
Nella cartella src del ros workspace
```bash
git clone https://github.com/Vanvitelli-Robotics/uclv_aipr_panda_sim.git #Simulatore
git clone https://github.com/Vanvitelli-Robotics/uclv_aipr_panda_clik_demo.git
sudo apt update && rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
```

## Avviare la demo

Avviare il simulatore:
```bash
ros2 launch uclv_aipr_panda_sim demo.launch.py
```

In un nuovo terminale avviare i server del clik e generatori di traiettoria:
```bash
ros2 launch uclv_aipr_panda_clik_demo clik.launch.py
```

In un nuovo terminale avviare il nodo task:
```bash
ros2 run uclv_aipr_panda_clik_demo task
```
Ogni volta che il terminale si ferma su:
```bash
Press a key to continue...
```
Inserire un qualsiasi **singolo** carattere e premere invio per continuare.


## License

GNU General Public License v3.0