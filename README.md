# AEROS Navigation

Proyecto de navegación autónoma para drones usando ArduPilot SITL, visión por computador y RL.

## Estructura

```
aeros_navigation/
├── sim/          # Configuración SITL y Gazebo
├── control/      # Scripts Python de control MAVLink
├── vision/       # YOLOv8, OpenCV, procesado de cámara
├── rl/           # Entorno Gym + agentes RL
├── training/     # Scripts de entrenamiento para el DGX
├── missions/     # Misiones programadas (.plan, scripts)
└── docs/         # Notas, diagramas
```

## Requisitos previos

- Python 3.9
- [uv](https://docs.astral.sh/uv/) (gestor de paquetes)
- ArduPilot clonado en `~/ardupilot`

## Instalación

```bash
cd ~/aeros_navigation
uv sync  # Crea .venv e instala dependencias
```

## Activar el entorno virtual

```bash
source ~/aeros_navigation/.venv/bin/activate
```

Para desactivar:

```bash
deactivate
```

> Si usas `uv run <script>` no hace falta activar el venv, `uv` lo gestiona automáticamente.

## Arrancar ArduPilot SITL

### 1. Compilar el firmware de simulación

```bash
cd ~/ardupilot
./waf configure --board sitl
./waf copter
```

> Usa `./waf plane` o `./waf rover` según el vehículo.

### 2. Lanzar el simulador SITL

```bash
# Desde la raíz de ardupilot (drone/multicopter)
cd ~/ardupilot
Tools/autotest/sim_vehicle.py -v ArduCopter --console --map

# Para un avión
# Tools/autotest/sim_vehicle.py -v ArduPlane --console --map
```

Opciones útiles:
- `--console` → abre la consola MAVProxy
- `--map` → abre el mapa
- `-I 0` → instancia de simulación (por defecto 0)
- `--speedup 3` → acelera la simulación
- `-w` → resetea EEPROM al arrancar
- `--custom-location=LAT,LON,ALT,HDG` → posición inicial personalizada

### 3. Conectar con Python (pymavlink / dronekit)

Por defecto SITL expone:
- **TCP 5760** (MAVProxy principal)
- **UDP 14550** (conexión para scripts)
- **TCP 5762** (segunda conexión)

```python
# Ejemplo con pymavlink
from pymavlink import mavutil
conn = mavutil.mavlink_connection('udp:127.0.0.1:14550')
conn.wait_heartbeat()
print("Conectado al dron")
```

```python
# Ejemplo con dronekit
from dronekit import connect
vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)
print(f"Modo: {vehicle.mode.name}")
```

### 4. Comandos útiles en MAVProxy

```
mode guided          # Cambiar a modo GUIDED
arm throttle         # Armar motores
takeoff 10           # Despegar a 10m
wp load mission.plan # Cargar misión
wp list              # Ver waypoints
mission start        # Iniciar misión
status               # Estado del vehículo
```

## Parar SITL

`Ctrl+C` en la terminal de `sim_vehicle.py` o:

```bash
killall arducopter
```
