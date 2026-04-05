"""
DroneHoverEnv — Entorno Gymnasium para hover estable.
Usa pymavlink directamente (compatible con Python 3.10+).

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
from pymavlink import mavutil


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
        self.conn = None
        self.steps = 0

        # Estado actual del vehículo
        self._pos = np.zeros(3)  # north, east, up
        self._vel = np.zeros(3)  # vx, vy, vz
        self._armed = False
        self._mode = ""
        self._alt = 0.0

        # Observación: [dx, dy, dz, vx, vy, vz]
        self.observation_space = spaces.Box(
            low=-50.0, high=50.0, shape=(6,), dtype=np.float32
        )

        # Acción: [vx, vy, vz, yaw_rate] normalizado [-1, 1]
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(4,), dtype=np.float32
        )

    def _connect(self):
        print(f"[Env] Conectando a {self.connection_string}...")
        self.conn = mavutil.mavlink_connection(self.connection_string)
        self.conn.wait_heartbeat()
        print(f"[Env] Heartbeat recibido (system {self.conn.target_system})")

        # Pedir mensajes a mayor frecuencia
        for msg_id, interval_us in [
            (mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 50000),  # 20Hz
            (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 100000),  # 10Hz
        ]:
            self.conn.mav.command_long_send(
                self.conn.target_system, self.conn.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                msg_id, interval_us, 0, 0, 0, 0, 0,
            )

    def _update_state(self):
        """Lee todos los mensajes disponibles y actualiza el estado."""
        while True:
            msg = self.conn.recv_match(blocking=False)
            if msg is None:
                break
            msg_type = msg.get_type()

            if msg_type == "LOCAL_POSITION_NED":
                self._pos[0] = msg.x   # north
                self._pos[1] = msg.y   # east
                self._pos[2] = -msg.z  # NED down -> up
                self._vel[0] = msg.vx
                self._vel[1] = msg.vy
                self._vel[2] = msg.vz

            elif msg_type == "GLOBAL_POSITION_INT":
                self._alt = msg.relative_alt / 1000.0  # mm -> m

            elif msg_type == "HEARTBEAT":
                self._armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                mode_map = mavutil.mode_mapping_acm if hasattr(mavutil, 'mode_mapping_acm') else {}
                if not mode_map:
                    mode_map = self.conn.mode_mapping()
                inv_map = {v: k for k, v in mode_map.items()} if mode_map else {}
                self._mode = inv_map.get(msg.custom_mode, str(msg.custom_mode))

    def _get_obs(self):
        self._update_state()
        delta = self._pos - self.target
        obs = np.concatenate([delta, self._vel]).astype(np.float32)
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
        vx = float(action[0]) * 2.0   # [-2, 2] m/s
        vy = float(action[1]) * 2.0
        vz = float(-action[2]) * 2.0  # acción positiva = subir, NED positivo = bajar
        yaw_rate = float(action[3]) * 30.0  # [-30, 30] deg/s

        self.conn.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000_0100_0111_0000,  # velocity + yaw_rate
            0, 0, 0,       # x, y, z (ignorados)
            vx, vy, vz,   # velocidades
            0, 0, 0,       # aceleraciones (ignoradas)
            0, yaw_rate,   # yaw, yaw_rate
        )

    def _set_mode(self, mode):
        mode_id = self.conn.mode_mapping().get(mode)
        if mode_id is None:
            print(f"[Env] Modo {mode} no encontrado")
            return
        self.conn.mav.set_mode_send(
            self.conn.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )

    def _arm(self):
        self.conn.mav.command_long_send(
            self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1, 0, 0, 0, 0, 0, 0,
        )

    def _takeoff(self, alt):
        self.conn.mav.command_long_send(
            self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
            0, 0, 0, 0, 0, 0, alt,
        )

    def _wait_armed(self, timeout=30):
        t0 = time.time()
        while time.time() - t0 < timeout:
            self._update_state()
            if self._armed:
                return True
            time.sleep(0.3)
        return False

    def _wait_disarmed(self, timeout=60):
        t0 = time.time()
        while time.time() - t0 < timeout:
            self._update_state()
            if not self._armed:
                return True
            time.sleep(0.5)
        return False

    def _wait_alt(self, target, threshold=0.90, timeout=30):
        t0 = time.time()
        while time.time() - t0 < timeout:
            self._update_state()
            if self._pos[2] >= target * threshold:
                return True
            time.sleep(0.3)
        return False

    def _arm_and_takeoff(self):
        # Esperar que SITL esté listo (pre-arm checks)
        print("[Env] Esperando pre-arm...")
        time.sleep(2)
        self._update_state()

        self._set_mode("GUIDED")
        time.sleep(1)

        print("[Env] Armando...")
        self._arm()
        if not self._wait_armed():
            print("[Env] Warning: timeout armando")

        print(f"[Env] Despegando a {self.target[2]}m...")
        self._takeoff(self.target[2])

        if not self._wait_alt(self.target[2]):
            print("[Env] Warning: timeout en despegue")

        print("[Env] Altitud alcanzada — episodio iniciado.")

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.steps = 0

        if self.conn is None:
            self._connect()

        # Si el dron estaba volando, aterrizar
        self._update_state()
        if self._armed:
            self._set_mode("LAND")
            self._wait_disarmed()
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
            self._set_mode("LAND")

        return obs, reward, terminated, truncated, {}

    def close(self):
        if self.conn:
            self._update_state()
            if self._armed:
                self._set_mode("LAND")
                time.sleep(5)
            self.conn.close()
            self.conn = None
