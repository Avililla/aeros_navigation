"""
Test rápido del entorno con acciones aleatorias.
Verifica que el env funciona antes de entrenar.

Uso:
    uv run rl/test_env.py
"""

import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
from rl.envs.drone_hover_env import DroneHoverEnv


def main():
    env = DroneHoverEnv(connection_string="tcp:127.0.0.1:5762", max_steps=100)

    print("[Test] Reseteando entorno...")
    obs, _ = env.reset()
    print(f"[Test] Obs inicial: {obs}")

    total_reward = 0.0
    for step in range(100):
        action = env.action_space.sample()
        obs, reward, terminated, truncated, _ = env.step(action)
        total_reward += reward

        dist = np.linalg.norm(obs[:3])
        alt = env.target[2] + obs[2]
        print(f"  Step {step+1:3d} | Alt: {alt:5.1f}m | Dist: {dist:5.2f}m | R: {reward:+6.2f}")

        if terminated or truncated:
            reason = "CRASH/OUT" if terminated else "MAX_STEPS"
            print(f"\n[Test] Episodio terminado: {reason}")
            break

    print(f"[Test] Reward total: {total_reward:+.1f}")
    env.close()
    print("[Test] Entorno cerrado.")


if __name__ == "__main__":
    main()
