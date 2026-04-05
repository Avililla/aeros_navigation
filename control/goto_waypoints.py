import time
import math
import argparse
import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from dronekit import connect, VehicleMode, LocationGlobalRelative
from config.settings import SITL_CONNECTION, DEFAULT_TAKEOFF_ALT, DEFAULT_SPEED

def connect_vehicle(connection_string):
    print(f"\n[AEROS] Conectando a {connection_string}...")
    vehicle = connect(connection_string, wait_ready=True)
    print(f"  Home: {vehicle.home_location}")
    return vehicle

def arm_and_takeoff(vehicle, altitude):
    print(f"\n[AEROS] Despegando a {altitude}m...")
    while not vehicle.is_armable:
        time.sleep(1)
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(0.5)
    vehicle.simple_takeoff(altitude)
    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f"  Altitud: {alt:.1f}m")
        if alt >= altitude * 0.95:
            print("  En posición ✓")
            break
        time.sleep(0.5)

def get_distance_metres(loc1, loc2):
    """Distancia aproximada entre dos puntos GPS en metros."""
    dlat = loc2.lat - loc1.lat
    dlon = loc2.lon - loc1.lon
    return math.sqrt((dlat * 1.113195e5) ** 2 + (dlon * 1.113195e5) ** 2)


def goto_waypoint(vehicle, target_location, groundspeed):
    print(f"\n[AEROS] Navegando a waypoint: {target_location.lat}, {target_location.lon} a {groundspeed} m/s...")
    vehicle.simple_goto(target_location, groundspeed=groundspeed)
    while True:
        current_location = vehicle.location.global_relative_frame
        distance = get_distance_metres(current_location, target_location)
        print(f"  Distancia al objetivo: {distance:.1f}m")
        if distance < 2.0:
            print("  Waypoint alcanzado ✓")
            break
        time.sleep(1)

def main():
    parser = argparse.ArgumentParser(description="AEROS — Goto Waypoints")
    parser.add_argument("--alt", type=float, default=DEFAULT_TAKEOFF_ALT)
    parser.add_argument("--speed", type=float, default=DEFAULT_SPEED)
    parser.add_argument("--connection", type=str, default=SITL_CONNECTION)
    args = parser.parse_args()

    vehicle = None
    try:
        vehicle = connect_vehicle(args.connection)
        arm_and_takeoff(vehicle, args.alt)

        # Esperar a que home_location esté disponible
        while not vehicle.home_location:
            vehicle.commands.download()
            vehicle.commands.wait_ready()
            time.sleep(1)

        home = vehicle.home_location
        print(f"\n[AEROS] Home: ({home.lat:.6f}, {home.lon:.6f})")

        # Definir waypoints relativos al home (offsets en grados)
        # 1 grado lat ≈ 111km → 0.0001 grados ≈ 11 metros
        waypoints = [
            LocationGlobalRelative(home.lat + 0.0001, home.lon,          args.alt),  # 11m Norte
            LocationGlobalRelative(home.lat + 0.0001, home.lon + 0.0001, args.alt),  # 11m Norte + 11m Este
            LocationGlobalRelative(home.lat,          home.lon + 0.0001, args.alt),  # 11m Este
            LocationGlobalRelative(home.lat,          home.lon,          args.alt),  # Home (cuadrado)
        ]

        print(f"\n[AEROS] Iniciando misión — {len(waypoints)} waypoints")
        for i, wp in enumerate(waypoints, 1):
            print(f"\n[AEROS] Waypoint {i}/{len(waypoints)}")
            goto_waypoint(vehicle, wp, args.speed)
            time.sleep(2)  # pausa breve en cada punto

        print("\n[AEROS] Misión completada — RTL")
        vehicle.mode = VehicleMode("RTL")

        # Esperar aterrizaje
        while vehicle.armed:
            alt = vehicle.location.global_relative_frame.alt
            print(f"  Regresando... {alt:.1f}m")
            time.sleep(2)

        print("[AEROS] En tierra ✓")

    except KeyboardInterrupt:
        print("\n[AEROS] Cancelado — RTL de emergencia")
        if vehicle:
            vehicle.mode = VehicleMode("RTL")

    except Exception as e:
        print(f"\n[AEROS] Error: {e}")
        if vehicle:
            vehicle.mode = VehicleMode("RTL")

    finally:
        if vehicle:
            vehicle.close()
            print("[AEROS] Conexión cerrada.")


if __name__ == "__main__":
    main()