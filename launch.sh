#!/bin/bash
SESSION="aeros"
SIM_VEHICLE="$HOME/ardupilot/Tools/autotest/sim_vehicle.py"

# Mata sesión anterior si existe
tmux kill-session -t $SESSION 2>/dev/null

tmux new-session -d -s $SESSION -n "sitl"

# Ventana 1: SITL
tmux send-keys -t $SESSION:sitl "source ~/.zshrc && source ~/venv-ardupilot/bin/activate && $SIM_VEHICLE -v ArduCopter -f gazebo-iris --model JSON --map --console" Enter

# Ventana 2: Gazebo
tmux new-window -t $SESSION -n "gazebo"
tmux send-keys -t $SESSION:gazebo "source ~/.zshrc && sleep 3 && gz sim -v4 -r -s iris_runway.sdf" Enter

# Ventana 3: Bridge de cámara (usa Python del sistema con gz bindings)
tmux new-window -t $SESSION -n "bridge"
tmux send-keys -t $SESSION:bridge "cd ~/aeros_navigation && sleep 8 && /usr/bin/python3 vision/gz_camera_bridge.py" Enter

# Ventana 4: Script de control
tmux new-window -t $SESSION -n "control"
tmux send-keys -t $SESSION:control "cd ~/aeros_navigation" Enter

# Ventana 5: Terminal libre para debug/logs
tmux new-window -t $SESSION -n "debug"
tmux send-keys -t $SESSION:debug "cd ~/aeros_navigation" Enter

# Seleccionar ventana de control
tmux select-window -t $SESSION:control

tmux attach -t $SESSION
