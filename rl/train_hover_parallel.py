"""
Entrenamiento paralelo con múltiples instancias SITL.
Diseñado para DGX Spark — N entornos en paralelo aceleran N veces.

Uso:
    uv run rl/train_hover_parallel.py --num-envs 4 --timesteps 200000

Requisitos:
    - N instancias de SITL corriendo en puertos diferentes:
      Terminal 1: sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON -I0
      Terminal 2: sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON -I1
      Terminal 3: sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON -I2
      Terminal 4: sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON -I3

    - Puertos: instancia -IN escucha en tcp:127.0.0.1:5762 + N*10
      -I0 -> 5762, -I1 -> 5772, -I2 -> 5782, -I3 -> 5792
"""

import argparse
import os
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import torch
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv
from stable_baselines3.common.callbacks import CheckpointCallback


def make_env(port, idx):
    def _init():
        from rl.envs.drone_hover_env import DroneHoverEnv
        return DroneHoverEnv(connection_string=f"tcp:127.0.0.1:{port}")
    return _init


def main():
    parser = argparse.ArgumentParser(description="AEROS — Parallel Hover Training")
    parser.add_argument("--num-envs", type=int, default=4)
    parser.add_argument("--timesteps", type=int, default=200_000)
    parser.add_argument("--base-port", type=int, default=5762)
    parser.add_argument("--save-path", type=str, default="rl/models/hover_ppo_parallel")
    args = parser.parse_args()

    os.makedirs(os.path.dirname(args.save_path), exist_ok=True)

    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"[AEROS] Device: {device}")
    print(f"[AEROS] Lanzando {args.num_envs} entornos en paralelo...")

    ports = [args.base_port + i * 10 for i in range(args.num_envs)]
    print(f"[AEROS] Puertos SITL: {ports}")

    env = SubprocVecEnv([make_env(port, i) for i, port in enumerate(ports)])

    model = PPO(
        "MlpPolicy",
        env,
        verbose=1,
        learning_rate=3e-4,
        n_steps=512,
        batch_size=128,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        device=device,
        tensorboard_log="rl/logs/hover_parallel",
    )

    checkpoint_cb = CheckpointCallback(
        save_freq=10000,
        save_path="rl/models/checkpoints/",
        name_prefix="hover_ppo_par",
    )

    print(f"[AEROS] Entrenando PPO por {args.timesteps} timesteps ({args.num_envs}x paralelo)...")

    try:
        model.learn(total_timesteps=args.timesteps, callback=checkpoint_cb)
        model.save(args.save_path)
        print(f"\n[AEROS] Modelo guardado en {args.save_path}")
    except KeyboardInterrupt:
        model.save(args.save_path + "_interrupted")
        print(f"\n[AEROS] Interrumpido — modelo guardado")
    finally:
        env.close()


if __name__ == "__main__":
    main()
