"""
DroneHoverEnv — Entorno Gymnasium para hover estable.

Objetivo: el dron despega a (0, 0, 10) y debe mantenerse ahí.

Observación (6D):
    [dx, dy, dz, vx, vy, vz]
    dx/dy/dz = distancia al objetivo en cada eje (metros)
    vx/vy/vz = velocidad en cada eje (m/s)

Acción (4D continuo):
    [vx_cmd, vy_cmd, vz_cmd, yaw_rate]
    Comandos de velocidad NED normalizados [-1, 1] -> [-2, 2] m/s
    yaw_rate normalizado [-1, 1] -> [-30, 30] deg/s

Reward:
    +1 por step si está dentro de 1m del objetivo
    -distancia si está más lejos
    -100 si se aleja >20m o baja de 2m (crash)

Episodio:
    Termina si crash, se aleja >20m, o pasan max_steps (1000)
"""

import time
import numpy as np
import gymnasium as gym
from gymnasium import spaces
from dronekit import connect, VehicleMode, LocationGlobalRelative


class DroneHoverEnv(gym.Env):
    metadata = {"render_modes": ["human"]}

    def __init__(
        self,
        connection_string="tcp:127.0.0.1:5762",
        target_alt=10.0,
        max_steps=1000,
        step_duration=0.1,
    ):
        super().__init__()

        self.connection_string = connection_string
        self.target = np.array([0.0, 0.0, target_alt])
        self.max_steps = max_steps
        self.step_duration = step_duration
        self.vehicle = None
        self.steps = 0
        self.origin = None

        # Observación: [dx, dy, dz, vx, vy, vz]
        self.observation_space = spaces.Box(
            low=-50.0, high=50.0, shape=(6,), dtype=np.float32
        )

        # Acción: [vx, vy, vz, yaw_rate] normalizado [-1, 1]
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(4,), dtype=np.float32
        )

    def _get_obs(self):
        loc = self.vehicle.location.local_frame
        vel = self.vehicle.velocity

        pos = np.array([
            loc.north or 0.0,
            loc.east or 0.0,
            -(loc.down or 0.0),  # NED -> altitud positiva
        ])

        velocity = np.array([
            vel[0] if vel else 0.0,
            vel[1] if vel else 0.0,
            vel[2] if vel else 0.0,
        ])

        delta = pos - self.target
        obs = np.concatenate([delta, velocity]).astype(np.float32)
        return obs

    def _get_reward(self, obs):
        delta = obs[:3]
        distance = np.linalg.norm(delta)

        if distance < 1.0:
            return 1.0
        return -distance * 0.1

    def _is_terminated(self, obs):
        delta = obs[:3]
        distance = np.linalg.norm(delta)
        alt = self.target[2] + delta[2]

        if distance > 20.0:
            return True
        if alt < 2.0:
            return True
        return False

    def _is_truncated(self):
        return self.steps >= self.max_steps

    def _send_velocity(self, action):
        from dronekit import mavutil

        vx = float(action[0]) * 2.0   # [-2, 2] m/s
        vy = float(action[1]) * 2.0
        vz = float(-action[2]) * 2.0  # invertir: acción positiva = subir, NED positivo = bajar
        yaw_rate = float(action[3]) * 30.0  # [-30, 30] deg/s

        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000_0100_0111_00_00,  # velocity + yaw_rate
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            0, yaw_rate,
        )
        self.vehicle.send_mavlink(msg)

    def _arm_and_takeoff(self):
        print("[Env] Esperando armable...")
        while not self.vehicle.is_armable:
            time.sleep(0.5)

        self.vehicle.mode = VehicleMode("GUIDED")
        while self.vehicle.mode.name != "GUIDED":
            time.sleep(0.3)

        self.vehicle.armed = True
        while not self.vehicle.armed:
            time.sleep(0.3)

        print(f"[Env] Despegando a {self.target[2]}m...")
        self.vehicle.simple_takeoff(self.target[2])

        while True:
            alt = self.vehicle.location.global_relative_frame.alt
            if alt >= self.target[2] * 0.90:
                break
            time.sleep(0.5)

        print("[Env] Altitud alcanzada — episodio iniciado.")

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.steps = 0

        if self.vehicle is None:
            print(f"[Env] Conectando a {self.connection_string}...")
            self.vehicle = connect(self.connection_string, wait_ready=True)

        # Si el dron ya estaba volando, aterrizar primero
        if self.vehicle.armed:
            self.vehicle.mode = VehicleMode("LAND")
            while self.vehicle.armed:
                time.sleep(1)
            time.sleep(2)

        self._arm_and_takeoff()
        time.sleep(1)

        obs = self._get_obs()
        return obs, {}

    def step(self, action):
        self.steps += 1

        self._send_velocity(action)
        time.sleep(self.step_duration)

        obs = self._get_obs()
        reward = self._get_reward(obs)
        terminated = self._is_terminated(obs)
        truncated = self._is_truncated()

        if terminated:
            reward = -100.0
            self.vehicle.mode = VehicleMode("LAND")

        return obs, reward, terminated, truncated, {}

    def close(self):
        if self.vehicle:
            if self.vehicle.armed:
                self.vehicle.mode = VehicleMode("LAND")
                time.sleep(5)
            self.vehicle.close()
            self.vehicle = None
