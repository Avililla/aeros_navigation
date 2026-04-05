#!/usr/bin/env python3
"""Bridge: lee el topic de cámara de Gazebo y lo sirve por TCP para OpenCV."""

import socket
import struct
import threading
import time

from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

TOPIC = "/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image"
HOST = "127.0.0.1"
PORT = 5601

lock = threading.Lock()
latest_frame = None


def on_image(msg: Image):
    global latest_frame
    with lock:
        latest_frame = (msg.width, msg.height, msg.pixel_format_type, bytes(msg.data))


def serve_client(conn, addr):
    print(f"[Bridge] Cliente conectado: {addr}")
    last_sent = None
    try:
        while True:
            with lock:
                frame = latest_frame

            if frame is None or frame is last_sent:
                time.sleep(0.01)
                continue

            last_sent = frame
            w, h, fmt, data = frame
            header = struct.pack("<III", w, h, fmt)
            length = struct.pack("<I", len(data))
            conn.sendall(header + length + data)
    except (BrokenPipeError, ConnectionResetError, OSError):
        print(f"[Bridge] Cliente desconectado: {addr}")
    finally:
        conn.close()


def main():
    node = Node()
    ok = node.subscribe(Image, TOPIC, on_image)
    if not ok:
        print(f"[Bridge] Error: no se pudo suscribir a {TOPIC}")
        return

    print(f"[Bridge] Suscrito a {TOPIC}")
    print(f"[Bridge] Sirviendo frames en tcp://{HOST}:{PORT}")

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen(2)

    try:
        while True:
            conn, addr = server.accept()
            t = threading.Thread(target=serve_client, args=(conn, addr), daemon=True)
            t.start()
    except KeyboardInterrupt:
        print("\n[Bridge] Cerrado.")
    finally:
        server.close()


if __name__ == "__main__":
    main()
