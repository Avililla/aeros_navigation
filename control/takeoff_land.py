import time
import argparse
import sys
import os

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__))))

from pymavlink import mavutil
from config.settings import SITL_CONNECTION, DEFAULT_TAKEOFF_ALT


def connect_vehicle(connection_string):
    print(f"\nConnecting to vehicle on: {connection_string}")
    conn = mavutil.mavlink_connection(connection_string)
    conn.wait_heartbeat()
    print(f" System ID: {conn.target_system}")

    # Pedir datos a mayor frecuencia
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 100000,
        0, 0, 0, 0, 0,
    )

    # Leer estado inicial
    msg = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
    armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) if msg else False
    mode_map = conn.mode_mapping()
    inv_map = {v: k for k, v in mode_map.items()} if mode_map else {}
    mode = inv_map.get(msg.custom_mode, "UNKNOWN") if msg else "UNKNOWN"

    # GPS info
    gps = conn.recv_match(type="GPS_RAW_INT", blocking=True, timeout=5)
    fix = gps.fix_type if gps else 0
    sats = gps.satellites_visible if gps else 0

    # Battery
    bat = conn.recv_match(type="SYS_STATUS", blocking=True, timeout=5)
    bat_pct = bat.battery_remaining if bat else None

    print(f" Modo de vuelo: {mode}")
    print(f" Armed: {armed}")
    print(f" GPS fix: {fix} ({sats} satélites visibles)")
    print(f" Bateria: {bat_pct}%")
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


def get_alt(conn):
    msg = conn.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
    if msg:
        return msg.relative_alt / 1000.0
    return 0.0


def is_armed(conn):
    msg = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
    if msg:
        return bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    return False


def arm_and_takeoff(conn, target_altitude):
    print(f"\n[AEROS] Preparando despegue a {target_altitude}m...")

    print("  Esperando que el vehículo sea armable...")
    time.sleep(3)

    set_mode(conn, "GUIDED")
    print("  Esperando modo GUIDED...")
    time.sleep(1)

    arm(conn)
    while not is_armed(conn):
        print("  Armando motores...")
        arm(conn)
        time.sleep(0.5)

    print("  Motores armados ✓")
    takeoff(conn, target_altitude)
    print("  Despegando...")

    while True:
        alt = get_alt(conn)
        print(f"  Altitud: {alt:6.2f}m  |  Objetivo: {target_altitude}m")
        if alt >= target_altitude * 0.95:
            print(f"\n  Altitud objetivo alcanzada ✓")
            break
        time.sleep(0.5)


def land(conn):
    print("\n[AEROS] Iniciando aterrizaje...")
    set_mode(conn, "LAND")
    while is_armed(conn):
        alt = get_alt(conn)
        print(f"  Aterrizando... {alt:.2f}m")
        time.sleep(1)
    print("  Aterrizaje completado ✓")


def main():
    parser = argparse.ArgumentParser(description="AEROS — Takeoff & Land")
    parser.add_argument("--alt", type=float, default=DEFAULT_TAKEOFF_ALT,
                        help=f"Altitud en metros (default: {DEFAULT_TAKEOFF_ALT})")
    parser.add_argument("--hold", type=float, default=5.0,
                        help="Segundos manteniendo posición (default: 5)")
    parser.add_argument("--connection", type=str, default=SITL_CONNECTION,
                        help=f"Conexión MAVLink (default: {SITL_CONNECTION})")
    args = parser.parse_args()

    conn = None
    try:
        conn = connect_vehicle(args.connection)
        arm_and_takeoff(conn, args.alt)

        print(f"\n[AEROS] Manteniendo posición {args.hold}s...")
        for i in range(int(args.hold), 0, -1):
            alt = get_alt(conn)
            print(f"  {i}s restantes — altitud: {alt:.2f}m")
            time.sleep(1)

        land(conn)

    except KeyboardInterrupt:
        print("\n\n[AEROS] Interrupción — aterrizando de emergencia...")
        if conn:
            set_mode(conn, "LAND")
            time.sleep(5)

    except Exception as e:
        print(f"\n[AEROS] Error: {e}")
        if conn:
            print("  Activando RTL por seguridad...")
            set_mode(conn, "RTL")

    finally:
        if conn:
            conn.close()
            print("[AEROS] Conexión cerrada.")


if __name__ == "__main__":
    main()
