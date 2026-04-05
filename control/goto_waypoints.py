import time
import math
import argparse
import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pymavlink import mavutil
from control.config.settings import SITL_CONNECTION, DEFAULT_TAKEOFF_ALT, DEFAULT_SPEED


def connect_vehicle(connection_string):
    print(f"\n[AEROS] Conectando a {connection_string}...")
    conn = mavutil.mavlink_connection(connection_string)
    conn.wait_heartbeat()
    print(f"  System ID: {conn.target_system}")
    return conn


def set_mode(conn, mode):
    mode_id = conn.mode_mapping().get(mode)
    if mode_id is None:
        print(f"  Modo {mode} no encontrado")
        return
    conn.mav.set_mode_send(
        conn.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id,
    )


def arm(conn):
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, 0, 0, 0, 0, 0, 0,
    )


def takeoff(conn, alt):
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
        0, 0, 0, 0, 0, 0, alt,
    )


def get_position(conn):
    """Devuelve (lat, lon, relative_alt) en grados y metros."""
    msg = conn.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
    if msg:
        return msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1000.0
    return 0, 0, 0


def is_armed(conn):
    msg = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
    if msg:
        return bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    return False


def get_distance(lat1, lon1, lat2, lon2):
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    return math.sqrt((dlat * 1.113195e5) ** 2 + (dlon * 1.113195e5) ** 2)


def arm_and_takeoff(conn, altitude):
    print(f"\n[AEROS] Despegando a {altitude}m...")
    time.sleep(3)

    set_mode(conn, "GUIDED")
    time.sleep(1)

    arm(conn)
    while not is_armed(conn):
        time.sleep(0.5)

    print("  Motores armados ✓")
    takeoff(conn, altitude)

    while True:
        _, _, alt = get_position(conn)
        print(f"  Altitud: {alt:.1f}m")
        if alt >= altitude * 0.95:
            print("  En posición ✓")
            break
        time.sleep(0.5)


def goto_waypoint(conn, lat, lon, alt, speed):
    print(f"\n[AEROS] Navegando a ({lat:.6f}, {lon:.6f}) a {speed} m/s...")

    conn.mav.mission_item_int_send(
        conn.target_system, conn.target_component,
        0,  # seq
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        2,  # current = 2 -> guided mode goto
        0,  # autocontinue
        0, 0, 0, 0,  # params
        int(lat * 1e7),
        int(lon * 1e7),
        alt,
    )

    # Configurar velocidad
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0,
        0,  # speed type: ground speed
        speed, -1, 0, 0, 0, 0,
    )

    while True:
        cur_lat, cur_lon, cur_alt = get_position(conn)
        dist = get_distance(cur_lat, cur_lon, lat, lon)
        print(f"  Distancia al objetivo: {dist:.1f}m")
        if dist < 2.0:
            print("  Waypoint alcanzado ✓")
            break
        time.sleep(1)


def main():
    parser = argparse.ArgumentParser(description="AEROS — Goto Waypoints")
    parser.add_argument("--alt", type=float, default=DEFAULT_TAKEOFF_ALT)
    parser.add_argument("--speed", type=float, default=DEFAULT_SPEED)
    parser.add_argument("--connection", type=str, default=SITL_CONNECTION)
    args = parser.parse_args()

    conn = None
    try:
        conn = connect_vehicle(args.connection)
        arm_and_takeoff(conn, args.alt)

        # Obtener posición home
        home_lat, home_lon, _ = get_position(conn)
        print(f"\n[AEROS] Home: ({home_lat:.6f}, {home_lon:.6f})")

        # Waypoints: cuadrado de ~11m
        waypoints = [
            (home_lat + 0.0001, home_lon,          args.alt),
            (home_lat + 0.0001, home_lon + 0.0001, args.alt),
            (home_lat,          home_lon + 0.0001, args.alt),
            (home_lat,          home_lon,          args.alt),
        ]

        print(f"\n[AEROS] Iniciando misión — {len(waypoints)} waypoints")
        for i, (lat, lon, alt) in enumerate(waypoints, 1):
            print(f"\n[AEROS] Waypoint {i}/{len(waypoints)}")
            goto_waypoint(conn, lat, lon, alt, args.speed)
            time.sleep(2)

        print("\n[AEROS] Misión completada — RTL")
        set_mode(conn, "RTL")

        while is_armed(conn):
            _, _, alt = get_position(conn)
            print(f"  Regresando... {alt:.1f}m")
            time.sleep(2)

        print("[AEROS] En tierra ✓")

    except KeyboardInterrupt:
        print("\n[AEROS] Cancelado — RTL de emergencia")
        if conn:
            set_mode(conn, "RTL")

    except Exception as e:
        print(f"\n[AEROS] Error: {e}")
        if conn:
            set_mode(conn, "RTL")

    finally:
        if conn:
            conn.close()
            print("[AEROS] Conexión cerrada.")


if __name__ == "__main__":
    main()
