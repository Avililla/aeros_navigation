"""
Entrena un agente PPO para hover estable.

Uso:
    uv run rl/train_hover.py
    uv run rl/train_hover.py --timesteps 50000 --connection tcp:127.0.0.1:5762
"""

import argparse
import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback
from rl.envs.drone_hover_env import DroneHoverEnv


def main():
    parser = argparse.ArgumentParser(description="AEROS — Train Hover Agent")
    parser.add_argument("--timesteps", type=int, default=50_000)
    parser.add_argument("--connection", type=str, default="tcp:127.0.0.1:5762")
    parser.add_argument("--save-path", type=str, default="rl/models/hover_ppo")
    args = parser.parse_args()

    os.makedirs(os.path.dirname(args.save_path), exist_ok=True)

    env = DroneHoverEnv(connection_string=args.connection)

    model = PPO(
        "MlpPolicy",
        env,
        verbose=1,
        learning_rate=3e-4,
        n_steps=512,
        batch_size=64,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        tensorboard_log="rl/logs/hover",
    )

    checkpoint_cb = CheckpointCallback(
        save_freq=5000,
        save_path="rl/models/checkpoints/",
        name_prefix="hover_ppo",
    )

    print(f"[AEROS] Entrenando PPO por {args.timesteps} timesteps...")
    print(f"[AEROS] Conexión: {args.connection}")
    print(f"[AEROS] Modelo se guardará en: {args.save_path}")

    try:
        model.learn(total_timesteps=args.timesteps, callback=checkpoint_cb)
        model.save(args.save_path)
        print(f"\n[AEROS] Modelo guardado en {args.save_path}")
    except KeyboardInterrupt:
        model.save(args.save_path + "_interrupted")
        print(f"\n[AEROS] Interrumpido — modelo guardado en {args.save_path}_interrupted")
    finally:
        env.close()


if __name__ == "__main__":
    main()
