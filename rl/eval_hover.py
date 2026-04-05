"""
Evalúa un agente PPO entrenado para hover.

Uso:
    uv run rl/eval_hover.py
    uv run rl/eval_hover.py --model rl/models/hover_ppo.zip --episodes 3
"""

import argparse
import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
from stable_baselines3 import PPO
from rl.envs.drone_hover_env import DroneHoverEnv


def main():
    parser = argparse.ArgumentParser(description="AEROS — Eval Hover Agent")
    parser.add_argument("--model", type=str, default="rl/models/hover_ppo.zip")
    parser.add_argument("--episodes", type=int, default=3)
    parser.add_argument("--connection", type=str, default="tcp:127.0.0.1:5762")
    args = parser.parse_args()

    env = DroneHoverEnv(connection_string=args.connection)
    model = PPO.load(args.model)

    print(f"[AEROS] Evaluando {args.model} por {args.episodes} episodios...")

    for ep in range(args.episodes):
        obs, _ = env.reset()
        total_reward = 0.0
        steps = 0

        while True:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, _ = env.step(action)
            total_reward += reward
            steps += 1

            dist = np.linalg.norm(obs[:3])
            print(f"  [Ep {ep+1}] Step {steps:4d} | Dist: {dist:5.2f}m | Reward: {reward:+6.2f}", end="\r")

            if terminated or truncated:
                break

        print(f"\n  [Ep {ep+1}] Fin — Steps: {steps} | Reward total: {total_reward:+.1f}")

    env.close()
    print("[AEROS] Evaluación completada.")


if __name__ == "__main__":
    main()
