#!/bin/bash
# Lanza N instancias de SITL para entrenamiento paralelo.
# Cada instancia -IN escucha en puerto 5762 + N*10
#
# Uso: ./rl/launch_sitl_cluster.sh 4

NUM_INSTANCES=${1:-4}
SIM_VEHICLE="$HOME/ardupilot/Tools/autotest/sim_vehicle.py"
SESSION="sitl-cluster"

tmux kill-session -t $SESSION 2>/dev/null
tmux new-session -d -s $SESSION

for i in $(seq 0 $((NUM_INSTANCES - 1))); do
    PORT=$((5762 + i * 10))
    if [ $i -gt 0 ]; then
        tmux new-window -t $SESSION
    fi
    tmux rename-window -t $SESSION "sitl-$i"
    tmux send-keys -t $SESSION "source ~/venv-ardupilot/bin/activate && $SIM_VEHICLE -v ArduCopter -f gazebo-iris --model JSON -I$i --no-mavproxy --no-extra-ports" Enter
    echo "SITL instancia $i -> puerto $PORT"
done

echo ""
echo "Cluster SITL lanzado con $NUM_INSTANCES instancias."
echo "  tmux attach -t $SESSION  (para ver)"
echo "  tmux kill-session -t $SESSION  (para matar)"
