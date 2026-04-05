import time
import argparse
import sys
import os

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__))))

from dronekit import connect, VehicleMode
from config.settings import SITL_CONNECTION, DEFAULT_TAKEOFF_ALT

def connect_vehicle(connection_string: str):
    print(f"\nConnecting to vehicle on: {connection_string}")
    vehicle = connect(connection_string, wait_ready=True)
    print(f" Firmware version: {vehicle.version}")
    print(f" Modo de vuelo: {vehicle.mode.name}")
    print(f" Armed: {vehicle.armed}")
    print(f" GPS fix: {vehicle.gps_0.fix_type} ({vehicle.gps_0.satellites_visible} satélites visibles)")
    print(f" Bateria: {vehicle.battery.level}%")
    return vehicle

def arm_and_takeoff(vehicle, target_altitude):
    print(f"\n[AEROS] Preparando despegue a {target_altitude}m...")

    print("  Esperando que el vehículo sea armable...")
    while not vehicle.is_armable:
        print("    ...", end="\r")
        time.sleep(1)
    
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode.name != "GUIDED":
        print("  Esperando modo GUIDED...")
        time.sleep(0.5)
    
    vehicle.armed = True
    while not vehicle.armed:
        print("  Armando motores...")
        time.sleep(0.5)
    
    print("  Motores armados ✓")
    vehicle.simple_takeoff(target_altitude)
    print(f"  Despegando...")

    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f"  Altitud: {alt:6.2f}m  |  Objetivo: {target_altitude}m")
        if alt >= target_altitude * 0.95:
            print(f"\n  Altitud objetivo alcanzada ✓")
            break
        time.sleep(0.5)

def land(vehicle):
    print("\n[AEROS] Iniciando aterrizaje...")
    vehicle.mode = VehicleMode("LAND")
    while vehicle.armed:
        alt = vehicle.location.global_relative_frame.alt
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
 
    vehicle = None
    try:
        vehicle = connect_vehicle(args.connection)
        arm_and_takeoff(vehicle, args.alt)
 
        print(f"\n[AEROS] Manteniendo posición {args.hold}s...")
        for i in range(int(args.hold), 0, -1):
            alt = vehicle.location.global_relative_frame.alt
            print(f"  {i}s restantes — altitud: {alt:.2f}m")
            time.sleep(1)
 
        land(vehicle)
 
    except KeyboardInterrupt:
        print("\n\n[AEROS] Interrupción — aterrizando de emergencia...")
        if vehicle:
            vehicle.mode = VehicleMode("LAND")
            time.sleep(5)
 
    except Exception as e:
        print(f"\n[AEROS] Error: {e}")
        if vehicle:
            print("  Activando RTL por seguridad...")
            vehicle.mode = VehicleMode("RTL")
 
    finally:
        if vehicle:
            vehicle.close()
            print("[AEROS] Conexión cerrada.")
 
 
if __name__ == "__main__":
    main()